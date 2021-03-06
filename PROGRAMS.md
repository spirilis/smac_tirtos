# SMac ProgramID registry
This is a registry of my own personal program IDs and their packet format.  This is intended for my own IoT usage and potential future projects around the house.

## IoT applications

### Device identification
Many individual microcontroller devices can have multiple instances of a sensor.  To quell ambiguity, a separate 16-bit Unsigned Integer addressing system
is used to give these sub-devices a unique ID.  This address is completely arbitrary, but it serves to separate the logical instance of a sensor
with the IEEE address of the microcontroller servicing it at that point in time (since, when a packet is received, the only unique ID the receiver
processing logic has at its disposal is the unique 32-bit SMac source address of the MCU who sent the packet, which is based on the sending MCU's
ROM-burned unique IEEE address).

E.g. my garage temp+humidity sensor might be serviced by a particular CC1310 gadget for a couple years, then something bad happens to that board and
I have to replace it with a new board... but the logical sensor should not change, since it's still giving me "garage temp+humidity" information.

#### Program ID: 0x2000
Payload: (variable):

(Byte index)

| 0-1 | 2+ |
|-----|----|
| Device ID | Canonical Name |

* Device ID: 2-bytes, Little-Endian 16-bit Unsigned Integer gives a unique numeric ID for this device
* Canonical Name: A variable-length character string describing the sensor, preferably including location.
A NUL terminating character is not expected.
An empty Canonical Name can be used by a base station to "query" a node's Device ID.

This device ID should be unique within the realm of the SMac star network in question.  For external reporting applications (e.g. IoT Cloud, MQTT et al), the
device ID should not be used alone; a unique identifier for the SMac star network in question should be included (e.g. RF center frequency, or a globally
unique arbitrary ID such as the IEEE address of the base station's CC1310)

If a node intends to transmit one of these frames, it is of utmost importance that the node verify its own Canonical Name is >0 bytes in size or else the
recipient may mistake the frame for a request to query the recipient's canonical name instead.  For a truly "null" canonical name, use a single space (UTF-8
character 32 decimal).

### Thermocouple
I use thermocouples for measuring anything related to fire; smoker/grill temperature, roast temperature, woodstove flue and exterior stove temps.
#### Program ID: 0x2001
Payload (7 bytes):

(Byte index)

| 0-1 | 2-3 | 4-5 | 6 |
|-----|-----|-----|---|
| Device ID | Thermocouple Temp | Ambient Temp | Error |

* Device ID: 2-bytes, Little-Endian 16-bit Unsigned Integer corresponding to a unique thermocouple instance (see Prog=0x2000 for details)
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

| 0-1 | 2-3 | 4 | 5 |
|-----|-----|---|---|
| Device ID | Temperature | Humidity | Status |

* Device ID: 2-bytes, Little-Endian Unsigned 16-bit Integer corresponding to unique sensor instance (see Prog=0x2000 for details)
* Temperature: 2-bytes, Little-Endian Signed 16-bit Integer corresponding to degrees Celsius with 3 bits of decimal (i.e. signed fixed-point Q12.3)
* Humidity: 1-byte, Unsigned 8-bit Integer corresponding to percent relative humidity in fixed-point Q0.8 format.  (0=0% RH, 255=100% RH)
* Status: 1-byte, Bitfield:

(Bit index)

| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
|---|---|---|---|---|---|---|---|
|   |   |   |   |   |   |   |Heater On|

 * Heater On: Indicates the HDC1080's built-in heater was used when performing this measurement.
   This is recommended when relative humidity has been around 100% for an extended period of time in order to
   boil off potential moisture infiltration that introduces persistent error in the humidity reading.  Obviously the
   heater   may skew the results a bit.

### Ping
Simple echo request, echo reply system especially useful for validating real-world RF link budget.
#### Echo Request Program ID: 0x2003
Payload:

(Byte index)

| 0-3 |
|-----|
| Sequence # |

* Sequence#: An unsigned 32-bit integer (Little-Endian) echoed back to the sender so lost pings can be tracked.

The Ping system uses the 32-bit IEEE address for its response so Device ID is not relevant here.

#### Echo Reply Program ID: 0x2004
Payload:

(Byte index)

| 0-3 |
|-----|
| Sequence # |

* Sequence#: The 4-byte payload given to this node during an initial echo-request (ProgID 0x2003)
