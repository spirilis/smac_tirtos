# SMac ProgramID registry
This is a registry of my own personal program IDs and their packet format.  This is intended for my own IoT usage and potential future projects around the house.

## IoT applications

### Thermocouple
I use thermocouples for measuring anything related to fire; smoker/grill temperature, roast temperature, woodstove flue and exterior stove temps.
#### Program ID: 0x2001
Payload (5 bytes):

(Byte index)

| 0-1 | 2-3 | 4 |
|-----|-----|---|
| Thermocouple Temp | Ambient Temp | Error |

* Thermocouple Temp: 2-bytes, Little-Endian Signed 16-bit Integer corresponding to whole degrees Celsius
* Ambient Temp: 2-bytes, Little-Endian Signed 16-bit Integer corresponding to whole degrees Celsius
* Error: 1-byte, Bitfield:

(Bit index)

| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
|---|---|---|---|---|---|---|---|
|   |   |   |   |   |Short to VCC|Short to GND|Open Circuit|

### Temperature + Humidity
My current application for this involves a TI HDC1080 temp+humidity sensor.
#### Program ID: 0x2002
Payload:

(Byte index)

| 0-1 | 2 | 3 |
|-----|---|---|
| Temperature | Humidity | Status |

* Temperature: 2-bytes, Little-Endian Signed 16-bit Integer corresponding to degrees Celsius with 3 bits of decimal (i.e. signed fixed-point Q12.3)
* Humidity: 1-byte, Unsigned 8-bit Integer corresponding to percent relative humidity in fixed-point Q0.8 format.  (0=0% RH, 255=100% RH)
* Status: 1-byte, Bitfield:

(Bit index)

| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
|---|---|---|---|---|---|---|---|
|   |   |   |   |   |   |   |Heater On|

 * Heater On: Indicates the HDC1080's built-in heater was used when performing this measurement.
   This is recommended when relative humidity has been around 100% for an extended period of time.