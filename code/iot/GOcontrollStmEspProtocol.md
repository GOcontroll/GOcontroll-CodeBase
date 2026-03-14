# ESP–STM32H Communicatieprotocol

UART: 115 200 baud, 8E1, volledige duplex.
Frame: `[0xAA][0x55][MSG_ID][LEN_LO][LEN_HI][PAYLOAD][CRC_LO][CRC_HI]`
CRC-16/CCITT (poly 0x1021, init 0xFFFF) over MSG_ID + LEN + PAYLOAD.

---

## Signalen

| MSG_ID | Naam | Richting | Payload |
|--------|------|----------|---------|
| 0x01 | HEARTBEAT | ↔ | — |
| 0x02 | ACK | ↔ | 1 byte: bevestigde MSG_ID |
| 0x10 | STATIC_INFO | STM → ESP | slot1(u32) + slot1_sw_major(u8) + slot1_sw_minor(u8) + slot1_sw_patch(u8) + slot2(u32) + slot2_sw_major(u8) + slot2_sw_minor(u8) + slot2_sw_patch(u8) + app_major(u8) + app_minor(u8) + app_patch(u8) — 17 bytes |
| 0x11 | CYCLIC_INFO | STM → ESP | k15_mv(u16) + k30_mv(u16) + temperature_x10(i16) + can1_bitrate(u8) + can2_bitrate(u8) + can1_busload(u8) + can2_busload(u8) + accel_x(i16) + accel_y(i16) + accel_z(i16) + cpu_load(u8) + heap_available(u16) + stack_available(u16) — 21 bytes |
| 0x20 | MODEM_CONFIG | STM → ESP | apn_len(u8) + apn + pin_len(u8) + pin |
| 0x21 | LTE_ENABLE | STM → ESP | 1 byte: 0=uit, 1=aan |
| 0x22 | MQTT_CONFIG | STM → ESP | port(u16) + keepalive(u16) + url_len(u8) + url + user_len(u8) + user + pass_len(u8) + pass + cid_len(u8) + client_id |
| 0x23 | MQTT_ENABLE | STM → ESP | 1 byte: 0=uit, 1=aan |
| 0x24 | MQTT_PUBLISH | STM → ESP | qos(u8) + retain(u8) + topic_len(u8) + topic + payload_len(u16) + payload |
| 0x25 | GPS_ENABLE | STM → ESP | 1 byte: 0=uit, 1=aan |
| 0x26 | MQTT_SUBSCRIBE | STM → ESP | qos(u8) + topic_len(u8) + topic |
| 0x27 | MQTT_UNSUBSCRIBE | STM → ESP | topic_len(u8) + topic |
| 0x30 | MQTT_STATUS | ESP → STM | 1 byte: 0=verbroken, 1=verbinden, 2=verbonden — verstuurd bij iedere statuswijziging |
| 0x31 | MQTT_RECEIVED | ESP → STM | topic_len(u8) + topic + payload_len(u16) + payload |
| 0x32 | TIME_SYNC | ESP → STM | year(u16) + month(u8) + day(u8) + hour(u8) + minute(u8) + second(u8) — 7 bytes |
| 0x40 | GPS_DATA | ESP → STM | lat(f32) + lon(f32) + alt(f32) + speed(f32) + utc_time(u32) + utc_date(u32) + valid(u8) — 25 bytes |
| 0x50 | MODEM_STATUS | ESP → STM | state(u8) + ip_len(u8) + ip |

### STATIC_INFO veldtoelichting
Eenmalig verstuurd door de STM32H na het opstarten.
- `slot1` / `slot2`: numeriek type van de geïnstalleerde Moduline module (0 = leeg)
- `slot1_sw_major/minor/patch`: semantische versie van de software op module slot 1
- `slot2_sw_major/minor/patch`: semantische versie van de software op module slot 2
- `app_major/minor/patch`: semantische versie van de STM32H applicatie

### CYCLIC_INFO veldtoelichting
Iedere 200 ms verstuurd door de STM32H.
- `k15_mv` / `k30_mv`: voedingsspanningen in millivolt
- `temperature_x10`: temperatuur × 10 in gehele getallen (bijv. 253 = 25,3 °C)
- `can1_bitrate` / `can2_bitrate`: 1 = 125 kbps, 2 = 250 kbps, 3 = 500 kbps, 4 = 1 Mbps
- `can1_busload` / `can2_busload`: actuele busbelasting in procenten (0–100)
- `accel_x` / `accel_y` / `accel_z`: ruwe accelerometerwaarden (signed 16-bit, LSB-first)
- `cpu_load`: CPU-belasting van de STM32H in procenten (0–100)
- `heap_available`: vrij heap-geheugen in bytes
- `stack_available`: vrij stack-geheugen van de meetende taak in bytes

### MQTT_STATUS statuswaarden
`0` = verbroken, `1` = verbinden, `2` = verbonden.
Verstuurd door de ESP bij iedere statuswijziging (niet periodiek).

### TIME_SYNC veldtoelichting
Verstuurd door de ESP als antwoord op een tijdsyncroniatieverzoek vanuit de app.
- `year`: volledig jaar (bijv. 2026), little-endian u16
- `month`: maand 1–12
- `day`: dag 1–31
- `hour`: uur 0–23
- `minute`: minuut 0–59
- `second`: seconde 0–59

De STM32H gebruikt deze waarden om de interne RTC in te stellen via de HAL_RTC_SetTime / HAL_RTC_SetDate functies.

### MODEM_STATUS state-waarden
`0` = uit, `1` = verbinden, `2` = verbonden

### GPS utc_time / utc_date formaat
`utc_time`: HHMMSS als integer (bijv. 143022 = 14:30:22)
`utc_date`: DDMMYY als integer (bijv. 230226 = 23-02-2026)

---

## BLE CONTROL commando's (app → ESP)

De app stuurt commando's via de CONTROL characteristic (F0001002). De eerste byte is altijd het commandobyte.

| Byte 0 | Naam | Totale lengte | Beschrijving |
|--------|------|--------------|--------------|
| 0x01 | START | 1 byte | Begin firmware-overdracht |
| 0x02 | END | 1 byte | Einde firmware-overdracht |
| 0x03 | ABORT | 1 byte | Annuleer lopende overdracht |
| 0x04 | TIME_SYNC | 8 bytes | Tijdsynchronisatie: `[0x04][year_lo][year_hi][month][day][hour][min][sec]` |

Bij ontvangst van TIME_SYNC stuurt de ESP direct een TIME_SYNC frame (0x32) naar de STM32H.

---

## Ideeënlijst — nog niet geïmplementeerd

| Richting | Naam | Beschrijving |
|----------|------|--------------|
| STM → ESP | TIME_REQUEST | Vraag UTC-tijd op (GPS of NTP) — ESP antwoordt dan met TIME_SYNC (0x32) |
| ESP → STM | SIGNAL_QUALITY | LTE signaalsterkte (RSSI/BER) |
| STM → ESP | RESET_ESP | Herstart de ESP32 |
| ESP → STM | BOOT_READY | ESP meldt dat alle modules klaar zijn |
| STM → ESP | LOG_FORWARD | Stuur een logbericht naar de ESP voor remote logging |
