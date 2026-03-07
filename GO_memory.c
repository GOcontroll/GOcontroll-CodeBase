/**************************************************************************************
 * \file   GO_memory.c
 * \brief  Non-volatile memory and diagnostic code storage for GOcontroll hardware.
 *         Combines MemoryEmulation (key-value NVM) and MemoryDiagnostic (DTC storage).
 *
 *         Platform selection via preprocessor define:
 *           GOCONTROLL_IOT  →  STM32H5 (Moduline IOT): TODO — Flash/EEPROM backend
 *           (default)       →  Linux/IMX8 (Moduline IV / Moduline Mini): filesystem
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
 * STM32H5 (GOCONTROLL_IOT) specific implementations
 ****************************************************************************************
 ****************************************************************************************/
#ifdef GOCONTROLL_IOT

#include "stm32h5xx_hal.h"

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
void GO_memory_emulation_write(char *key, float value, float *oldValue) {
	if (oldValue != NULL) {
		if (value == *oldValue) {
			return;  /* value unchanged — skip write */
		}
		*oldValue = value;
	} else {
		/* Initialisation call: skip write if a value is already stored */
		float stored = (float)0xffffffff;
		GO_memory_emulation_read(key, &stored);
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

/**************************************************************************************
** \brief     Read a key-value pair from persistent storage.
**            Linux:  key is a file path.
**            STM32:  TODO — key interpretation to be defined.
** \param     key    storage identifier
** \param     value  output pointer; unchanged if the key is not found
** \return    none
***************************************************************************************/
void GO_memory_emulation_read(char *key, float *value) {
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
 ****************************************************************************************
 * Linux specific implementations
 ****************************************************************************************
 ****************************************************************************************/
#else

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

#endif /* GOCONTROLL_IOT */

/* end of GO_memory.c */
