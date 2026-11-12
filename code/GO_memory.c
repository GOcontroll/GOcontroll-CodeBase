/**************************************************************************************
 * \file   GO_memory.c
 * \brief  Non-volatile memory and diagnostic code storage for GOcontroll hardware.
 *         Combines MemoryEmulation (key-value NVM) and MemoryDiagnostic (DTC storage).
 *
 *         Platform selection via preprocessor define:
 *           GOCONTROLL_STM  →  STM32H5 (Moduline S1): TODO — Flash/EEPROM backend
 *           (default)       →  Linux/IMX8 (Moduline L4 / Moduline M1): filesystem
 * \internal
 *----------------------------------------------------------------------------------------
 *                          C O P Y R I G H T
 *----------------------------------------------------------------------------------------
 * Copyright 2024 (c) by GOcontroll http://www.gocontroll.com All rights reserved
 *
 *----------------------------------------------------------------------------------------
 *                            L I C E N S E
 *----------------------------------------------------------------------------------------
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * \endinternal
 ****************************************************************************************/

/****************************************************************************************
 * Include files — common
 ****************************************************************************************/
#include "GO_memory.h"

#include <stdint.h>
#include <string.h>

/****************************************************************************************
 ****************************************************************************************
 * STM32H5 (GOCONTROLL_STM) specific implementations
 ****************************************************************************************
 ****************************************************************************************/
#ifdef GOCONTROLL_STM

#include "stm32h5xx_hal.h"
#include "cmsis_os2.h"

/***************  Flash NVM layout  *******************************************
 * Bank 2, last 4 sectors (each 8 KB, 16-byte quad-word write granularity).
 * Sector N of Bank 2 starts at: 0x08100000 + N * 0x2000.
 *
 * NOTE: the linker script reserves Flash up to 0x081F8000 (LENGTH = 2016K)
 *       so the compiler never places code or read-only data in these sectors.
 *
 *   0x081F8000  Sector 124 — MemoryEmulation  page A
 *   0x081FA000  Sector 125 — MemoryEmulation  page B
 *   0x081FC000  Sector 126 — MemoryDiagnostic page A
 *   0x081FE000  Sector 127 — MemoryDiagnostic page B
 *****************************************************************************/
#define NVM_SECTOR_SIZE         0x2000U          /* 8 KB per sector          */
#define NVM_SECTOR_BANK         FLASH_BANK_2
#define NVM_ENTRIES_PER_SECTOR  (NVM_SECTOR_SIZE / 16U)  /* 512 quad-words   */
#define NVM_VALID_MAGIC         0xA5A5A5A5U      /* entry is active          */
#define NVM_ERASED              0xFFFFFFFFU      /* Flash erased state       */

#define NVM_EMU_PAGE_A_SECTOR   124U
#define NVM_EMU_PAGE_B_SECTOR   125U
#define NVM_EMU_PAGE_A_ADDR     (0x08100000U + NVM_EMU_PAGE_A_SECTOR * NVM_SECTOR_SIZE)
#define NVM_EMU_PAGE_B_ADDR     (0x08100000U + NVM_EMU_PAGE_B_SECTOR * NVM_SECTOR_SIZE)

#define NVM_DTC_PAGE_A_SECTOR   126U
#define NVM_DTC_PAGE_B_SECTOR   127U
#define NVM_DTC_PAGE_A_ADDR     (0x08100000U + NVM_DTC_PAGE_A_SECTOR * NVM_SECTOR_SIZE)
#define NVM_DTC_PAGE_B_ADDR     (0x08100000U + NVM_DTC_PAGE_B_SECTOR * NVM_SECTOR_SIZE)

/***************  Entry structures (each exactly 16 bytes = one quad-word)  ***/

/* NVM key-value entry */
typedef struct {
	uint32_t key_hash;  /* djb2 hash of the key string               */
	float    value;     /* stored floating-point value                */
	uint32_t valid;     /* NVM_VALID_MAGIC when active                */
	uint32_t seq;       /* write sequence; highest index = most recent */
} nvm_emu_entry_t;

/* DTC entry */
typedef struct {
	uint32_t dtc_code;      /* encoded SPN/FMI/OC                    */
	uint32_t valid;         /* NVM_VALID_MAGIC = active; 0 = deleted  */
	uint32_t reserved[2];
} nvm_dtc_entry_t;

/***************  Module state  ***********************************************/

static uint32_t s_emu_active;     /* base address of the active NVM page  */
static uint32_t s_emu_write_idx;  /* next free slot index in active page   */
static uint32_t s_emu_seq;        /* global write-sequence counter         */

static uint32_t s_dtc_active;     /* base address of the active DTC page  */
static uint32_t s_dtc_write_idx;  /* next free slot index in active page   */

/* Serializes every flash-touching emulation operation. The NVM read/write API
 * is called concurrently from many FreeRTOS tasks; HAL_FLASH_Unlock/Program/
 * Erase is global and non-reentrant, the s_emu_write_idx append counter is a
 * shared RMW, and on the STM32H5 a read of a flash bank mid-program/erase
 * stalls or bus-faults. Without this lock, two tasks touching NVM at once
 * corrupt the page index or wedge the flash controller -> the system hangs.
 * A recursive + priority-inheriting mutex is used so a future internal caller
 * may re-enter; created in GO_memory_emulation_initialize(). The NULL guard in
 * the lock/unlock helpers keeps the pre-scheduler init calls (single-threaded,
 * before osKernelStart) safe and lock-free. */
static osMutexId_t s_emu_mutex = NULL;

static const osMutexAttr_t s_emu_mutex_attr = {
	.name      = "nvm_emu",
	.attr_bits = osMutexRecursive | osMutexPrioInherit,
	.cb_mem    = NULL,
	.cb_size   = 0U,
};

static inline void emu_lock(void) {
	if (s_emu_mutex != NULL) { (void)osMutexAcquire(s_emu_mutex, osWaitForever); }
}

static inline void emu_unlock(void) {
	if (s_emu_mutex != NULL) { (void)osMutexRelease(s_emu_mutex); }
}

/***************  Internal helpers  *******************************************/

/* djb2 hash — maps a key string to a 32-bit identifier */
static uint32_t nvm_hash(const char *str) {
	uint32_t hash = 5381U;
	unsigned char c;
	while ((c = (unsigned char)*str++) != 0U) {
		hash = ((hash << 5U) + hash) + (uint32_t)c;
	}
	return hash;
}

/* Write one 16-byte quad-word to Flash (address must be 16-byte aligned) */
static HAL_StatusTypeDef nvm_write_qword(uint32_t addr, const void *data) {
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
	HAL_StatusTypeDef rc = HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD,
	                                          addr, (uint32_t)data);
	HAL_FLASH_Lock();
	return rc;
}

/* Erase one 8 KB sector in Bank 2 */
static HAL_StatusTypeDef nvm_erase_sector(uint32_t sector) {
	FLASH_EraseInitTypeDef cfg = {
		.TypeErase  = FLASH_TYPEERASE_SECTORS,
		.Banks      = NVM_SECTOR_BANK,
		.Sector     = sector,
		.NbSectors  = 1U,
	};
	uint32_t err;
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
	HAL_StatusTypeDef rc = HAL_FLASHEx_Erase(&cfg, &err);
	HAL_FLASH_Lock();
	return rc;
}

/* Return the index of the first erased (all-0xFF) NVM emu slot */
static uint32_t emu_find_write_idx(uint32_t page) {
	for (uint32_t i = 0U; i < NVM_ENTRIES_PER_SECTOR; i++) {
		const nvm_emu_entry_t *e =
			(const nvm_emu_entry_t *)(page + i * 16U);
		if (e->key_hash == NVM_ERASED) {
			return i;
		}
	}
	return NVM_ENTRIES_PER_SECTOR;  /* page full */
}

/* Return the index of the first erased (all-0xFF) DTC slot */
static uint32_t dtc_find_write_idx(uint32_t page) {
	for (uint32_t i = 0U; i < NVM_ENTRIES_PER_SECTOR; i++) {
		const nvm_dtc_entry_t *e =
			(const nvm_dtc_entry_t *)(page + i * 16U);
		if (e->dtc_code == NVM_ERASED) {
			return i;
		}
	}
	return NVM_ENTRIES_PER_SECTOR;  /* page full */
}

/* Return the highest seq value of all valid entries on a NVM emu page */
static uint32_t emu_max_seq(uint32_t page) {
	uint32_t max_seq = 0U;
	for (uint32_t i = 0U; i < NVM_ENTRIES_PER_SECTOR; i++) {
		const nvm_emu_entry_t *e =
			(const nvm_emu_entry_t *)(page + i * 16U);
		if (e->valid == NVM_VALID_MAGIC &&
		    e->key_hash != NVM_ERASED &&
		    e->seq != NVM_ERASED &&
		    e->seq > max_seq) {
			max_seq = e->seq;
		}
	}
	return max_seq;
}

/*
 * Compact NVM emu page: for each unique key in src, copy the most-recent
 * entry to dst, then erase src.  Updates s_emu_active / s_emu_write_idx /
 * s_emu_seq.
 *
 * The caller must ensure dst is already erased before calling.
 */
static void emu_compact(uint32_t src, uint32_t dst) {
	uint32_t dst_idx = 0U;

	for (uint32_t i = 0U;
	     i < NVM_ENTRIES_PER_SECTOR && dst_idx < NVM_ENTRIES_PER_SECTOR;
	     i++) {
		const nvm_emu_entry_t *e =
			(const nvm_emu_entry_t *)(src + i * 16U);

		if (e->valid != NVM_VALID_MAGIC || e->key_hash == NVM_ERASED) {
			continue;
		}

		/* Skip if a newer entry with the same hash exists later in src */
		uint8_t has_newer = 0U;
		for (uint32_t j = i + 1U; j < NVM_ENTRIES_PER_SECTOR; j++) {
			const nvm_emu_entry_t *e2 =
				(const nvm_emu_entry_t *)(src + j * 16U);
			if (e2->valid == NVM_VALID_MAGIC &&
			    e2->key_hash == e->key_hash) {
				has_newer = 1U;
				break;
			}
		}
		if (has_newer) {
			continue;
		}

		nvm_emu_entry_t ne = { e->key_hash, e->value,
		                       NVM_VALID_MAGIC, dst_idx };
		nvm_write_qword(dst + dst_idx * 16U, &ne);
		dst_idx++;
	}

	uint32_t src_sector = (src == NVM_EMU_PAGE_A_ADDR)
	                      ? NVM_EMU_PAGE_A_SECTOR
	                      : NVM_EMU_PAGE_B_SECTOR;
	nvm_erase_sector(src_sector);

	s_emu_active    = dst;
	s_emu_write_idx = dst_idx;
	s_emu_seq       = dst_idx;
}

/*
 * Compact DTC page: copy all valid entries from src to dst, erase src.
 * The caller must ensure dst is already erased before calling.
 */
static void dtc_compact(uint32_t src, uint32_t dst) {
	uint32_t dst_idx = 0U;

	for (uint32_t i = 0U;
	     i < NVM_ENTRIES_PER_SECTOR && dst_idx < NVM_ENTRIES_PER_SECTOR;
	     i++) {
		const nvm_dtc_entry_t *e =
			(const nvm_dtc_entry_t *)(src + i * 16U);
		if (e->dtc_code == NVM_ERASED) {
			break;  /* reached the unwritten region */
		}
		if (e->valid != NVM_VALID_MAGIC) {
			continue;  /* deleted entry */
		}

		nvm_dtc_entry_t ne = { e->dtc_code, NVM_VALID_MAGIC, {0U, 0U} };
		nvm_write_qword(dst + dst_idx * 16U, &ne);
		dst_idx++;
	}

	uint32_t src_sector = (src == NVM_DTC_PAGE_A_ADDR)
	                      ? NVM_DTC_PAGE_A_SECTOR
	                      : NVM_DTC_PAGE_B_SECTOR;
	nvm_erase_sector(src_sector);

	s_dtc_active    = dst;
	s_dtc_write_idx = dst_idx;
}

/***************  MemoryEmulation  ********************************************/

/**************************************************************************************
** \brief     Initialize the memory emulation storage.
**            Linux:  creates /usr/mem-sim/ and /etc/go-simulink/ if absent.
**            STM32:  TODO — initialize Flash/EEPROM backend.
** \return    none
***************************************************************************************/
void GO_memory_emulation_initialize(void) {
	/* Create the NVM lock before any task can touch flash (init runs in main(),
	 * before osKernelStart). Idempotent via the NULL guard. */
	if (s_emu_mutex == NULL) {
		s_emu_mutex = osMutexNew(&s_emu_mutex_attr);
	}

	uint32_t idx_a = emu_find_write_idx(NVM_EMU_PAGE_A_ADDR);
	uint32_t idx_b = emu_find_write_idx(NVM_EMU_PAGE_B_ADDR);

	if (idx_a == 0U && idx_b == 0U) {
		/* Both pages empty — start fresh on page A */
		s_emu_active    = NVM_EMU_PAGE_A_ADDR;
		s_emu_write_idx = 0U;
		s_emu_seq       = 0U;
	} else if (idx_b == 0U) {
		/* Only page A has data — page A is active */
		s_emu_active    = NVM_EMU_PAGE_A_ADDR;
		s_emu_write_idx = idx_a;
		s_emu_seq       = emu_max_seq(NVM_EMU_PAGE_A_ADDR);
	} else if (idx_a == 0U) {
		/* Only page B has data — page B is active */
		s_emu_active    = NVM_EMU_PAGE_B_ADDR;
		s_emu_write_idx = idx_b;
		s_emu_seq       = emu_max_seq(NVM_EMU_PAGE_B_ADDR);
	} else {
		/*
		 * Both pages contain data: a previous compaction was
		 * interrupted by a reset.  Recover by treating the page with
		 * the higher max-seq as the most-recent source of truth and
		 * re-compacting into the other page.
		 */
		uint32_t seq_a = emu_max_seq(NVM_EMU_PAGE_A_ADDR);
		uint32_t seq_b = emu_max_seq(NVM_EMU_PAGE_B_ADDR);
		if (seq_a >= seq_b) {
			nvm_erase_sector(NVM_EMU_PAGE_B_SECTOR);
			emu_compact(NVM_EMU_PAGE_A_ADDR, NVM_EMU_PAGE_B_ADDR);
		} else {
			nvm_erase_sector(NVM_EMU_PAGE_A_SECTOR);
			emu_compact(NVM_EMU_PAGE_B_ADDR, NVM_EMU_PAGE_A_ADDR);
		}
	}
}

/**************************************************************************************
** \brief     Write a key-value pair to persistent storage.
**            Linux:  key is a file path ("/usr/mem-sim/key" for NVM,
**                    "/dev/shm/key" for volatile).
**            STM32:  TODO — key interpretation to be defined (e.g. index or label).
** \param     key       storage identifier
** \param     value     value to store
** \param     oldValue  previous value; updated on write. Pass NULL on first init.
** \return    none
***************************************************************************************/
static void emu_read_unlocked(char *key, float *value);

/* Unlocked core — callers must hold s_emu_mutex (see GO_memory_emulation_write). */
static void emu_write_unlocked(char *key, float value, float *oldValue) {
	if (oldValue != NULL) {
		if (value == *oldValue) {
			return;  /* value unchanged — skip write */
		}
		*oldValue = value;
	} else {
		/* Initialisation call: skip write if a value is already stored */
		float stored = (float)0xffffffff;
		emu_read_unlocked(key, &stored);
		if (stored != (float)0xffffffff) {
			return;
		}
	}

	uint32_t h = nvm_hash(key);

	if (s_emu_write_idx >= NVM_ENTRIES_PER_SECTOR) {
		/* Active page full — compact to the alternate page */
		uint32_t other = (s_emu_active == NVM_EMU_PAGE_A_ADDR)
		                 ? NVM_EMU_PAGE_B_ADDR
		                 : NVM_EMU_PAGE_A_ADDR;
		uint32_t other_sector = (other == NVM_EMU_PAGE_A_ADDR)
		                        ? NVM_EMU_PAGE_A_SECTOR
		                        : NVM_EMU_PAGE_B_SECTOR;
		nvm_erase_sector(other_sector);
		emu_compact(s_emu_active, other);

		if (s_emu_write_idx >= NVM_ENTRIES_PER_SECTOR) {
			return;  /* NVM full (> 512 unique keys) — cannot store */
		}
	}

	nvm_emu_entry_t ne = { h, value, NVM_VALID_MAGIC, ++s_emu_seq };
	nvm_write_qword(s_emu_active + s_emu_write_idx * 16U, &ne);
	s_emu_write_idx++;
}

void GO_memory_emulation_write(char *key, float value, float *oldValue) {
	emu_lock();
	emu_write_unlocked(key, value, oldValue);
	emu_unlock();
}

/**************************************************************************************
** \brief     Read a key-value pair from persistent storage.
**            Linux:  key is a file path.
**            STM32:  TODO — key interpretation to be defined.
** \param     key    storage identifier
** \param     value  output pointer; unchanged if the key is not found
** \return    none
***************************************************************************************/
/* Unlocked core — callers must hold s_emu_mutex (see GO_memory_emulation_read). */
static void emu_read_unlocked(char *key, float *value) {
	uint32_t h = nvm_hash(key);

	/* Scan from end to beginning — last written entry for this key wins */
	for (int i = (int)s_emu_write_idx - 1; i >= 0; i--) {
		const nvm_emu_entry_t *e =
			(const nvm_emu_entry_t *)(s_emu_active + (uint32_t)i * 16U);
		if (e->valid == NVM_VALID_MAGIC && e->key_hash == h) {
			*value = e->value;
			return;
		}
	}
	/* Key not found — leave *value unchanged */
}

void GO_memory_emulation_read(char *key, float *value) {
	emu_lock();
	emu_read_unlocked(key, value);
	emu_unlock();
}

/***************  MemoryDiagnostic  *******************************************/

/* Encode SPN / FMI / OC into the same 32-bit format used on Linux */
static uint32_t dtc_encode(uint32_t spn, uint8_t fmi, uint8_t oc) {
	uint32_t code  = (spn & 0xFFFFU);
	code          += (spn & 0x70000U) << 5U;
	code          += ((uint32_t)fmi & 0x1FU) << 16U;
	code          += ((uint32_t)oc  & 0x7FU) << 24U;
	return code;
}

/**************************************************************************************
** \brief     Initialize the diagnostic code storage.
**            Linux:  creates /usr/mem-diag/ directory if absent.
**            STM32:  TODO — initialize Flash/EEPROM DTC region.
** \return    none
***************************************************************************************/
void GO_memory_diagnostic_initialize(void) {
	uint32_t idx_a = dtc_find_write_idx(NVM_DTC_PAGE_A_ADDR);
	uint32_t idx_b = dtc_find_write_idx(NVM_DTC_PAGE_B_ADDR);

	if (idx_a == 0U && idx_b == 0U) {
		s_dtc_active    = NVM_DTC_PAGE_A_ADDR;
		s_dtc_write_idx = 0U;
	} else if (idx_b == 0U) {
		s_dtc_active    = NVM_DTC_PAGE_A_ADDR;
		s_dtc_write_idx = idx_a;
	} else if (idx_a == 0U) {
		s_dtc_active    = NVM_DTC_PAGE_B_ADDR;
		s_dtc_write_idx = idx_b;
	} else {
		/* Both pages have data — recover by compacting page A into page B */
		nvm_erase_sector(NVM_DTC_PAGE_B_SECTOR);
		dtc_compact(NVM_DTC_PAGE_A_ADDR, NVM_DTC_PAGE_B_ADDR);
	}
}

/**************************************************************************************
** \brief     Write or update a diagnostic trouble code entry.
** \param     spn               Suspect Parameter Number (J1939)
** \param     fmi               Failure Mode Identifier (J1939)
** \param     oc                Occurrence count
** \param     freezedDescription label string for the freeze-frame parameter
** \param     freezedParameter   freeze-frame value
** \param     messageType       DIAGNOSTICSTART, DIAGNOSTICFREEZE or DIAGNOSTICSTOP
** \return    none
***************************************************************************************/
void GO_memory_diagnostic_write(uint32_t spn, uint8_t fmi, uint8_t oc,
                              char *freezedDescription,
                              float freezedParameter,
                              uint8_t messageType) {
	(void)freezedDescription;
	(void)freezedParameter;

	/* Only DIAGNOSTICSTART triggers a Flash write; freeze-frame data is
	 * not stored (no file system available for arbitrary text on STM32) */
	if (messageType != DIAGNOSTICSTART) {
		return;
	}

	uint32_t code = dtc_encode(spn, fmi, oc);

	/* Do not store duplicates */
	for (uint32_t i = 0U; i < s_dtc_write_idx; i++) {
		const nvm_dtc_entry_t *e =
			(const nvm_dtc_entry_t *)(s_dtc_active + i * 16U);
		if (e->valid == NVM_VALID_MAGIC && e->dtc_code == code) {
			return;
		}
	}

	if (s_dtc_write_idx >= NVM_ENTRIES_PER_SECTOR) {
		uint32_t other = (s_dtc_active == NVM_DTC_PAGE_A_ADDR)
		                 ? NVM_DTC_PAGE_B_ADDR
		                 : NVM_DTC_PAGE_A_ADDR;
		uint32_t other_sector = (other == NVM_DTC_PAGE_A_ADDR)
		                        ? NVM_DTC_PAGE_A_SECTOR
		                        : NVM_DTC_PAGE_B_SECTOR;
		nvm_erase_sector(other_sector);
		dtc_compact(s_dtc_active, other);

		if (s_dtc_write_idx >= NVM_ENTRIES_PER_SECTOR) {
			return;  /* DTC storage full — cannot add entry */
		}
	}

	nvm_dtc_entry_t ne = { code, NVM_VALID_MAGIC, {0U, 0U} };
	nvm_write_qword(s_dtc_active + s_dtc_write_idx * 16U, &ne);
	s_dtc_write_idx++;
}

/**************************************************************************************
** \brief     Count the number of stored diagnostic codes.
** \return    number of stored DTCs
***************************************************************************************/
uint16_t GO_memory_diagnostic_count_codes(void) {
	uint16_t count = 0U;
	for (uint32_t i = 0U; i < s_dtc_write_idx; i++) {
		const nvm_dtc_entry_t *e =
			(const nvm_dtc_entry_t *)(s_dtc_active + i * 16U);
		if (e->valid == NVM_VALID_MAGIC) {
			count++;
		}
	}
	return count;
}

/**************************************************************************************
** \brief     Delete all stored diagnostic codes.
** \return    none
***************************************************************************************/
void GO_memory_diagnostic_delete_all(void) {
	uint32_t sector_a = (s_dtc_active == NVM_DTC_PAGE_A_ADDR)
	                    ? NVM_DTC_PAGE_A_SECTOR
	                    : NVM_DTC_PAGE_B_SECTOR;
	uint32_t sector_b = (s_dtc_active == NVM_DTC_PAGE_A_ADDR)
	                    ? NVM_DTC_PAGE_B_SECTOR
	                    : NVM_DTC_PAGE_A_SECTOR;
	nvm_erase_sector(sector_a);
	nvm_erase_sector(sector_b);
	s_dtc_active    = NVM_DTC_PAGE_A_ADDR;
	s_dtc_write_idx = 0U;
}

/**************************************************************************************
** \brief     Return the raw DTC value at a given index.
** \param     index  zero-based index into the stored DTC list
** \return    encoded DTC value, or 0 if not found
***************************************************************************************/
uint32_t GO_memory_diagnostic_code_on_index(uint16_t index) {
	uint16_t valid_count = 0U;
	for (uint32_t i = 0U; i < s_dtc_write_idx; i++) {
		const nvm_dtc_entry_t *e =
			(const nvm_dtc_entry_t *)(s_dtc_active + i * 16U);
		if (e->valid == NVM_VALID_MAGIC) {
			if (valid_count == index) {
				return e->dtc_code;
			}
			valid_count++;
		}
	}
	return 0U;
}

/**************************************************************************************
** \brief     Delete a single diagnostic code entry.
** \param     spn  Suspect Parameter Number
** \param     fmi  Failure Mode Identifier
** \param     oc   Occurrence count
** \return    none
***************************************************************************************/
void GO_memory_diagnostic_delete_single(uint32_t spn, uint8_t fmi, uint8_t oc) {
	uint32_t code = dtc_encode(spn, fmi, oc);

	for (uint32_t i = 0U; i < s_dtc_write_idx; i++) {
		const nvm_dtc_entry_t *e =
			(const nvm_dtc_entry_t *)(s_dtc_active + i * 16U);
		if (e->valid == NVM_VALID_MAGIC && e->dtc_code == code) {
			/*
			 * Overwrite the entry with all-zeros.  On STM32H5 Flash,
			 * programming can only flip bits 1→0.  Clearing an entry
			 * that was programmed with NVM_VALID_MAGIC (0xA5A5A5A5)
			 * and a non-zero dtc_code only involves 1→0 transitions,
			 * so this is a valid Flash write.
			 */
			const uint64_t zeros[2] = { 0U, 0U };
			nvm_write_qword(s_dtc_active + i * 16U, zeros);
			return;
		}
	}
}

/****************************************************************************************
 * Application configuration — not available on the S1 (no filesystem)
 ****************************************************************************************/

#include <errno.h>

void GO_config_set_path(const char *path) {
	(void)path;
}

int GO_config_read_string(const char *key, char *value, size_t size) {
	(void)key;
	if (value != NULL && size > 0U) {
		value[0] = '\0';
	}
	return -ENOSYS;
}

int GO_config_read_number(const char *key, float *value) {
	(void)key;
	(void)value;
	return -ENOSYS;
}

int GO_config_ensure(const char *key, const char *default_value) {
	(void)key;
	(void)default_value;
	return -ENOSYS;
}

int GO_config_changed(void) {
	return -ENOSYS;
}

/****************************************************************************************
 ****************************************************************************************
 * Linux specific implementations
 ****************************************************************************************
 ****************************************************************************************/
#elif defined(GOCONTROLL_LINUX)

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

/****************************************************************************************
 * MemoryEmulation — key-value NVM via filesystem
 ****************************************************************************************/

/**************************************************************************************
** \brief     Initialize the memory emulation storage.
**            Linux:  creates /usr/mem-sim/ and /etc/go-simulink/ if absent.
**            STM32:  TODO — initialize Flash/EEPROM backend.
** \return    none
***************************************************************************************/
void GO_memory_emulation_initialize(void) {
	struct stat st;
	if (stat("/usr/mem-sim", &st) == -1) {
		mkdir("/usr/mem-sim", 0555);
	}
	if (stat("/etc/go-simulink", &st) == -1) {
		mkdir("/etc/go-simulink", 0555);
	}
}

/****************************************************************************************/

/**************************************************************************************
** \brief     Write a key-value pair to persistent storage.
**            Linux:  key is a file path ("/usr/mem-sim/key" for NVM,
**                    "/dev/shm/key" for volatile).
**            STM32:  TODO — key interpretation to be defined (e.g. index or label).
** \param     key       storage identifier
** \param     value     value to store
** \param     oldValue  previous value; updated on write. Pass NULL on first init.
** \return    none
***************************************************************************************/
void GO_memory_emulation_write(char *key, float value, float *oldValue) {
	if (oldValue != NULL) {
		if (value == *oldValue) {
			return;
		}
		*oldValue = value;
	} else {
		/* Initialisation call: skip write if a value is already stored */
		float dummy = (float)0xffffffff;
		GO_memory_emulation_read(key, &dummy);
		if (dummy != (float)0xffffffff) {
			return;
		}
	}

	char valueString[15];
	gcvt(value, 10, valueString);

	int fileId = open(key, O_WRONLY | O_CREAT | O_NONBLOCK | O_TRUNC, 0555);
	write(fileId, valueString, strlen(valueString));
	close(fileId);
}

/****************************************************************************************/

/**************************************************************************************
** \brief     Read a key-value pair from persistent storage.
**            Linux:  key is a file path.
**            STM32:  TODO — key interpretation to be defined.
** \param     key    storage identifier
** \param     value  output pointer; unchanged if the key is not found
** \return    none
***************************************************************************************/
void GO_memory_emulation_read(char *key, float *value) {
	int fileId = open(key, O_RDONLY | O_NONBLOCK);
	if (fileId <= 0) {
		close(fileId);
		return;
	}

	char tempValue[15] = {0};
	read(fileId, &tempValue[0], 15);
	close(fileId);
	*value = strtof(tempValue, NULL);
}

/****************************************************************************************
 * Application configuration — flat JSON object, see GO_memory.h
 *
 * The file is small (a handful of keys) and read only at start-up and after a change,
 * so every call simply loads and scans the whole file: no cache that can go stale.
 ****************************************************************************************/

#define GO_CONFIG_MAX_SIZE	(64U * 1024U)
#define GO_CONFIG_KEY_MAX	128U

static const char *s_config_path = NULL;

static const char *config_path(void) {
	return (s_config_path != NULL) ? s_config_path : GO_CONFIG_PATH;
}

/****************************************************************************************/

void GO_config_set_path(const char *path) {
	s_config_path = path;
}

/****************************************************************************************/

/* Load the whole file into a NUL-terminated heap buffer; the caller frees it. */
static int config_load(char **text) {
	*text = NULL;

	FILE *f = fopen(config_path(), "rb");
	if (f == NULL) {
		return (errno == ENOENT) ? -ENOENT : -EIO;
	}

	char *buf = malloc(GO_CONFIG_MAX_SIZE + 1U);
	if (buf == NULL) {
		fclose(f);
		return -ENOMEM;
	}

	size_t n	  = fread(buf, 1U, GO_CONFIG_MAX_SIZE, f);
	int    read_error = ferror(f);
	int    too_large  = (n == GO_CONFIG_MAX_SIZE) && (fgetc(f) != EOF);
	fclose(f);

	if (read_error || too_large) {
		free(buf);
		return read_error ? -EIO : -EFBIG;
	}

	buf[n] = '\0';
	*text  = buf;
	return 0;
}

/****************************************************************************************/

static void json_skip_ws(const char **p) {
	while (**p == ' ' || **p == '\t' || **p == '\n' || **p == '\r') {
		(*p)++;
	}
}

/* Append one byte to a bounded output; *n counts the full length, also past size. */
static void json_put(char *out, size_t size, size_t *n, char c) {
	if (out != NULL && *n + 1U < size) {
		out[*n] = c;
	}
	(*n)++;
}

static void json_put_utf8(char *out, size_t size, size_t *n, uint32_t cp) {
	if (cp < 0x80U) {
		json_put(out, size, n, (char)cp);
	} else if (cp < 0x800U) {
		json_put(out, size, n, (char)(0xC0U | (cp >> 6)));
		json_put(out, size, n, (char)(0x80U | (cp & 0x3FU)));
	} else if (cp < 0x10000U) {
		json_put(out, size, n, (char)(0xE0U | (cp >> 12)));
		json_put(out, size, n, (char)(0x80U | ((cp >> 6) & 0x3FU)));
		json_put(out, size, n, (char)(0x80U | (cp & 0x3FU)));
	} else {
		json_put(out, size, n, (char)(0xF0U | (cp >> 18)));
		json_put(out, size, n, (char)(0x80U | ((cp >> 12) & 0x3FU)));
		json_put(out, size, n, (char)(0x80U | ((cp >> 6) & 0x3FU)));
		json_put(out, size, n, (char)(0x80U | (cp & 0x3FU)));
	}
}

static int json_hex4(const char *p, uint32_t *cp) {
	*cp = 0U;
	for (int i = 0; i < 4; i++) {
		char     c = p[i];
		uint32_t d;
		if (c >= '0' && c <= '9') {
			d = (uint32_t)(c - '0');
		} else if (c >= 'a' && c <= 'f') {
			d = (uint32_t)(c - 'a' + 10);
		} else if (c >= 'A' && c <= 'F') {
			d = (uint32_t)(c - 'A' + 10);
		} else {
			return -EBADMSG;
		}
		*cp = (*cp << 4) | d;
	}
	return 0;
}

/* Decode the JSON string *p points at (opening quote) into out, NUL-terminated when
 * size > 0. *len receives the full decoded length. Advances *p past the closing quote. */
static int json_string(const char **p, char *out, size_t size, size_t *len) {
	size_t n = 0U;
	const char *s = *p;

	if (*s != '"') {
		return -EBADMSG;
	}
	s++;

	while (*s != '"') {
		if (*s == '\0' || (unsigned char)*s < 0x20U) {
			return -EBADMSG;
		}
		if (*s != '\\') {
			json_put(out, size, &n, *s++);
			continue;
		}
		s++;
		switch (*s) {
			case '"':  json_put(out, size, &n, '"');  break;
			case '\\': json_put(out, size, &n, '\\'); break;
			case '/':  json_put(out, size, &n, '/');  break;
			case 'b':  json_put(out, size, &n, '\b'); break;
			case 'f':  json_put(out, size, &n, '\f'); break;
			case 'n':  json_put(out, size, &n, '\n'); break;
			case 'r':  json_put(out, size, &n, '\r'); break;
			case 't':  json_put(out, size, &n, '\t'); break;
			case 'u': {
				uint32_t cp;
				if (json_hex4(s + 1, &cp) != 0) {
					return -EBADMSG;
				}
				s += 4;
				/* A high surrogate followed by an escaped low surrogate is one code point. */
				if (cp >= 0xD800U && cp <= 0xDBFFU && s[1] == '\\' && s[2] == 'u') {
					uint32_t lo;
					if (json_hex4(s + 3, &lo) == 0 && lo >= 0xDC00U && lo <= 0xDFFFU) {
						cp = 0x10000U + ((cp - 0xD800U) << 10) + (lo - 0xDC00U);
						s += 6;
					}
				}
				json_put_utf8(out, size, &n, cp);
				break;
			}
			default:
				return -EBADMSG;
		}
		s++;
	}

	if (out != NULL && size > 0U) {
		out[(n < size) ? n : size - 1U] = '\0';
	}
	*len = n;
	*p   = s + 1;
	return 0;
}

/* Copy a bare scalar (number, true, false, null) verbatim. The Web UI always writes
 * strings, but a hand-edited file may hold plain numbers. Objects and arrays are not
 * part of the format. */
static int json_scalar(const char **p, char *out, size_t size, size_t *len) {
	size_t n = 0U;
	const char *s = *p;

	if (*s == '{' || *s == '[') {
		return -EBADMSG;
	}
	while (*s != '\0' && *s != ',' && *s != '}' && *s != ' ' && *s != '\t' &&
		   *s != '\n' && *s != '\r') {
		json_put(out, size, &n, *s++);
	}
	if (n == 0U) {
		return -EBADMSG;
	}
	if (out != NULL && size > 0U) {
		out[(n < size) ? n : size - 1U] = '\0';
	}
	*len = n;
	*p   = s;
	return 0;
}

/* Look up key in the flat object text. On a hit the value is decoded into value/size
 * and *len gets its full length. On -ENOENT, *close points at the closing brace. */
static int json_find(const char *text, const char *key, char *value, size_t size,
					 size_t *len, const char **close) {
	const char *p = text;
	size_t key_len = strlen(key);

	json_skip_ws(&p);
	if (*p != '{') {
		return -EBADMSG;
	}
	p++;
	json_skip_ws(&p);

	while (*p != '}') {
		char   name[GO_CONFIG_KEY_MAX];
		size_t name_len;

		if (json_string(&p, name, sizeof(name), &name_len) != 0) {
			return -EBADMSG;
		}
		json_skip_ws(&p);
		if (*p != ':') {
			return -EBADMSG;
		}
		p++;
		json_skip_ws(&p);

		int match = (name_len == key_len) && (name_len < sizeof(name)) &&
					(memcmp(name, key, key_len) == 0);
		int res = (*p == '"') ? json_string(&p, match ? value : NULL, size, len)
							  : json_scalar(&p, match ? value : NULL, size, len);
		if (res != 0) {
			return res;
		}
		if (match) {
			return 0;
		}

		json_skip_ws(&p);
		if (*p == ',') {
			p++;
			json_skip_ws(&p);
		} else if (*p != '}') {
			return -EBADMSG;
		}
	}

	if (close != NULL) {
		*close = p;
	}
	return -ENOENT;
}

/****************************************************************************************/

int GO_config_read_string(const char *key, char *value, size_t size) {
	char  *text;
	size_t len = 0U;

	if (value != NULL && size > 0U) {
		value[0] = '\0';
	}
	if (key == NULL || value == NULL || size == 0U) {
		return -EINVAL;
	}

	int res = config_load(&text);
	if (res != 0) {
		return res;
	}
	res = json_find(text, key, value, size, &len, NULL);
	free(text);

	if (res == 0 && len >= size) {
		return -ENOSPC;
	}
	return res;
}

/****************************************************************************************/

int GO_config_read_number(const char *key, float *value) {
	char buf[64];

	int res = GO_config_read_string(key, buf, sizeof(buf));
	if (res == -ENOSPC) {
		return -EINVAL;
	}
	if (res != 0) {
		return res;
	}

	char *end;
	errno = 0;
	float v = strtof(buf, &end);
	if (end == buf || errno == ERANGE) {
		return -EINVAL;
	}
	while (*end == ' ' || *end == '\t') {
		end++;
	}
	if (*end != '\0') {
		return -EINVAL;
	}

	*value = v;
	return 0;
}

/****************************************************************************************/

/* Write a JSON string literal (with quotes) for s. */
static void json_write_string(FILE *f, const char *s) {
	fputc('"', f);
	for (; *s != '\0'; s++) {
		unsigned char c = (unsigned char)*s;
		switch (c) {
			case '"':  fputs("\\\"", f); break;
			case '\\': fputs("\\\\", f); break;
			case '\n': fputs("\\n", f);  break;
			case '\r': fputs("\\r", f);  break;
			case '\t': fputs("\\t", f);  break;
			default:
				if (c < 0x20U) {
					fprintf(f, "\\u%04x", c);
				} else {
					fputc((int)c, f);
				}
				break;
		}
	}
	fputc('"', f);
}

/* Create the parent directory of the configuration file if it is missing. */
static void config_make_dir(void) {
	char dir[256];
	const char *path = config_path();
	const char *slash = strrchr(path, '/');

	if (slash == NULL || slash == path || (size_t)(slash - path) >= sizeof(dir)) {
		return;
	}
	memcpy(dir, path, (size_t)(slash - path));
	dir[slash - path] = '\0';
	mkdir(dir, 0755);
}

int GO_config_ensure(const char *key, const char *default_value) {
	char	   *text = NULL;
	const char *close = NULL;
	size_t	    len;

	if (key == NULL || key[0] == '\0' || default_value == NULL) {
		return -EINVAL;
	}

	int res = config_load(&text);
	if (res == 0) {
		res = json_find(text, key, NULL, 0U, &len, &close);
		if (res == 0) {
			free(text);
			return 0; /* already present: never touch an existing value */
		}
		if (res != -ENOENT) {
			free(text);
			return res; /* do not rewrite a file we cannot parse */
		}
	} else if (res == -ENOENT) {
		config_make_dir();
	} else {
		return res;
	}

	/* Keep the existing members byte for byte; append the new one before '}'. Same
	 * layout as the Web UI writes (json.dumps, indent 4). */
	char tmp_path[300];
	if (snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", config_path()) >=
		(int)sizeof(tmp_path)) {
		free(text);
		return -ENAMETOOLONG;
	}

	FILE *f = fopen(tmp_path, "wb");
	if (f == NULL) {
		free(text);
		return -EIO;
	}

	int has_members = 0;
	if (text != NULL) {
		/* Everything up to the last non-blank character before the closing brace. */
		const char *last = close;
		while (last > text && (last[-1] == ' ' || last[-1] == '\t' ||
							   last[-1] == '\n' || last[-1] == '\r')) {
			last--;
		}
		has_members = (last > text) && (last[-1] != '{');
		fwrite(text, 1U, (size_t)(last - text), f);
	} else {
		fputc('{', f);
	}

	fputs(has_members ? ",\n    " : "\n    ", f);
	json_write_string(f, key);
	fputs(": ", f);
	json_write_string(f, default_value);
	fputs("\n}\n", f);
	free(text);

	int write_error = ferror(f);
	if (fflush(f) != 0 || fsync(fileno(f)) != 0) {
		write_error = 1;
	}
	if (fclose(f) != 0) {
		write_error = 1;
	}
	if (write_error || rename(tmp_path, config_path()) != 0) {
		unlink(tmp_path);
		return -EIO;
	}
	return 1;
}

/****************************************************************************************/

int GO_config_changed(void) {
	static int	 s_seen = 0;
	static struct stat s_last;
	struct stat	 st;

	if (stat(config_path(), &st) != 0) {
		s_seen = 0;
		return -ENOENT;
	}

	/* The Web UI replaces the file (new inode); an editor may rewrite it in place. */
	int changed = !s_seen || st.st_ino != s_last.st_ino || st.st_dev != s_last.st_dev ||
				  st.st_size != s_last.st_size || st.st_mtime != s_last.st_mtime;

	s_last = st;
	s_seen = 1;
	return changed;
}

/****************************************************************************************
 * MemoryDiagnostic — DTC storage via filesystem
 ****************************************************************************************/

static int remove_directory(const char *path);

/****************************************************************************************/

/**************************************************************************************
** \brief     Initialize the diagnostic code storage.
**            Linux:  creates /usr/mem-diag/ directory if absent.
**            STM32:  TODO — initialize Flash/EEPROM DTC region.
** \return    none
***************************************************************************************/
void GO_memory_diagnostic_initialize(void) {
	struct stat st = {0};
	if (stat("/usr/mem-diag", &st) == -1) {
		mkdir("/usr/mem-diag", 0555);
	}
}

/****************************************************************************************/

/**************************************************************************************
** \brief     Write or update a diagnostic trouble code entry.
** \param     spn               Suspect Parameter Number (J1939)
** \param     fmi               Failure Mode Identifier (J1939)
** \param     oc                Occurrence count
** \param     freezedDescription label string for the freeze-frame parameter
** \param     freezedParameter   freeze-frame value
** \param     messageType       DIAGNOSTICSTART, DIAGNOSTICFREEZE or DIAGNOSTICSTOP
** \return    none
***************************************************************************************/
void GO_memory_diagnostic_write(uint32_t spn, uint8_t fmi, uint8_t oc,
							  char *freezedDescription,
							  float freezedParameter,
							  uint8_t messageType) {
	static int	 fileId		  = 0;
	static uint32_t diagnosticCode   = 0;

	if (messageType == DIAGNOSTICSTART) {
		/* Encode SPN/FMI/OC into a single 32-bit DTC value */
		diagnosticCode  = (spn & 0xffff);
		diagnosticCode += (spn & 0x70000) << 5;
		diagnosticCode += (fmi & 0x1f) << 16;
		diagnosticCode += (oc  & 0x7f) << 24;

		char path[50]		  = {0};
		char diagnosticCodeStr[15];
		sprintf(diagnosticCodeStr, "%u", (int)diagnosticCode);
		strcat(path, "/usr/mem-diag/");
		strcat(path, diagnosticCodeStr);
		fileId = open(path, O_WRONLY | O_CREAT | O_NONBLOCK | O_TRUNC, 0555);
	}

	if (messageType == DIAGNOSTICFREEZE) {
		char codeStr[50]   = {0};
		char valueString[15];
		sprintf(valueString, "%1.2f", freezedParameter);
		strcat(codeStr, freezedDescription);
		strcat(codeStr, " : ");
		strcat(codeStr, valueString);
		strcat(codeStr, "\n");
		write(fileId, codeStr, strlen(codeStr));
	}

	if (messageType == DIAGNOSTICSTOP) {
		write(fileId, "\n", strlen("\n"));
		close(fileId);
	}
}

/****************************************************************************************/

/**************************************************************************************
** \brief     Count the number of stored diagnostic codes.
** \return    number of stored DTCs
***************************************************************************************/
uint16_t GO_memory_diagnostic_count_codes(void) {
	uint16_t	file_count = 0;
	DIR	       *dirp;
	struct dirent  *entry;

	char path[20] = {0};
	strcat(path, "/usr/mem-diag/");

	dirp = opendir(path);
	while ((entry = readdir(dirp)) != NULL) {
		if (entry->d_type == DT_REG) {
			file_count++;
		}
	}
	closedir(dirp);
	return file_count;
}

/****************************************************************************************/

/**************************************************************************************
** \brief     Delete all stored diagnostic codes.
** \return    none
***************************************************************************************/
void GO_memory_diagnostic_delete_all(void) {
	char path[20] = {0};
	strcat(path, "/usr/mem-diag/");
	if (remove_directory(path) == 0) {
		mkdir(path, 0555);
	}
}

/****************************************************************************************/

/**************************************************************************************
** \brief     Return the raw DTC value at a given index.
** \param     index  zero-based index into the stored DTC list
** \return    encoded DTC value, or 0 if not found
***************************************************************************************/
uint32_t GO_memory_diagnostic_code_on_index(uint16_t index) {
	char	      path[20] = {0};
	DIR	     *d;
	struct dirent *dir;
	uint16_t       indexCounter = 0;

	strcat(path, "/usr/mem-diag/");
	d = opendir(path);

	if (d) {
		while ((dir = readdir(d)) != NULL) {
			if (!strcmp(dir->d_name, ".") || !strcmp(dir->d_name, ".."))
				continue;
			if (indexCounter == index) {
				uint32_t code = strtol(dir->d_name, NULL, 10);
				closedir(d);
				return code;
			}
			indexCounter++;
		}
		closedir(d);
	}
	return 0;
}

/****************************************************************************************/

/**************************************************************************************
** \brief     Delete a single diagnostic code entry.
** \param     spn  Suspect Parameter Number
** \param     fmi  Failure Mode Identifier
** \param     oc   Occurrence count
** \return    none
***************************************************************************************/
void GO_memory_diagnostic_delete_single(uint32_t spn, uint8_t fmi, uint8_t oc) {
	uint32_t diagnosticCode;

	diagnosticCode  = (spn & 0xffff);
	diagnosticCode += (spn & 0x70000) << 5;
	diagnosticCode += (fmi & 0x1f) << 16;
	diagnosticCode += (oc  & 0x7f) << 24;

	char diagCodeStr[20];
	sprintf(diagCodeStr, "%u", (int)diagnosticCode);
	char file[35] = {0};
	strcat(file, "/usr/mem-diag/");
	strcat(file, diagCodeStr);
	remove(file);
}

/****************************************************************************************/

static int remove_directory(const char *path) {
	DIR	   *d	     = opendir(path);
	size_t	    path_len = strlen(path);
	int	    r	     = -1;

	if (d) {
		struct dirent *p;
		r = 0;
		while (!r && (p = readdir(d))) {
			if (!strcmp(p->d_name, ".") || !strcmp(p->d_name, ".."))
				continue;

			size_t len = path_len + strlen(p->d_name) + 2;
			char  *buf = malloc(len);

			if (buf) {
				struct stat statbuf;
				snprintf(buf, len, "%s/%s", path, p->d_name);
				if (!stat(buf, &statbuf)) {
					if (S_ISDIR(statbuf.st_mode))
						r = remove_directory(buf);
					else
						r = unlink(buf);
				}
				free(buf);
			}
		}
		closedir(d);
	}

	if (!r)
		r = rmdir(path);

	return r;
}

#endif /* GOCONTROLL_STM / GOCONTROLL_LINUX */

/* end of GO_memory.c */
