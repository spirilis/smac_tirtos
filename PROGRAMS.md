# SMac ProgramID registry
This is a registry of my own personal program IDs and their packet format.  This is intended for my own IoT usage and potential future projects around the house.

## IoT applications

### Thermocouple
I use thermocouples for measuring anything related to fire; smoker/grill temperature, roast temperature, woodstove flue and exterior stove temps.
#### Program ID: 0x2001
Payload:

| 0-1 | 2-3 | 4 |
|-----|-----|---|
| Thermocouple Temp | Ambient Temp | Error |

* Thermocouple Temp: 2-bytes, Little-Endian Signed 16-bit Integer corresponding to whole degrees Celsius
* Ambient Temp: 2-bytes, Little-Endian Signed 16-bit Integer corresponding to whole degrees Celsius
* Error: 1-byte, Bitfield:

| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
|---|---|---|---|---|---|---|---|
|   |   |   |   |   |Short to VCC|Short to GND|Open Circuit|