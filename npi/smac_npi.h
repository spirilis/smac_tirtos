/* smac_npi.h - SMac Network Processor Interface
 *
 * Header information
 */

#ifndef SMAC_NPI_H_
#define SMAC_NPI_H_


/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>

/* Board Header files */
#include "Board.h"

// LED definitions
#ifndef Board_RLED
#ifdef Board_PIN_RLED
#define Board_RLED Board_PIN_RLED
#endif
#endif

#ifndef Board_GLED
#ifdef Board_PIN_GLED
#define Board_GLED Board_PIN_GLED
#endif
#endif


/* SMac API */
#include <smac/smac.h>

// Identification string for this NPI controller instance (<=SMACNPI_CONTROL_DATA_MAXLEN, which is 8 by default)
#define SMACNPI_IDENTIFIER "RPIBSTN0"

// Control frame debugging
#define SMACNPI_CTRL_DEBUG 1
#define SMACNPI_CTRL_VERBOSE_DEBUG 0

/* NPI default parameters */
#define SMACNPI_RF_DEFAULT_CENTERFREQ (915000000)
#define SMACNPI_RF_DEFAULT_TXPOWER    (0)  // 0dBm, range is -10, 0 to 12 or 14 with CCFG_FORCE_VDDR_HH=1
#define SMACNPI_SERIAL_BAUDRATE       (115200)
#define SMACNPI_RF_DEFAULT_RX_ON      (0)

/* NPI internal buffering limits */
#define SMACNPI_UARTREAD_FRAME_RING_DEPTH (6)  // can't be higher than 31, FYI
#define SMACNPI_RFINBOUND_FRAME_RING_DEPTH (4) // this allocates a mailbox whose contents are ~7+SMAC_MAXIMUM_FRAMESIZE long.  Beware.
#define SMACNPI_UARTWRITE_RING_SIZE (1024)
#define SMACNPI_CONTROL_DATA_MAXLEN (8)
#define SMACNPI_CONTROL_FRAME_PENDING (2)





/* NPI Control Frame commands */
#define SMACNPI_CONTROL_UNSQUELCH_HOST     (0x00)
#define SMACNPI_CONTROL_SQUELCH_HOST       (0x01)
// GET_RF returns 8 bytes:
// #0: UInt8 boolean, RF ON/OFF (0 = off, !0 = on)
// #1-4: UInt32 Center Frequency, Little-Endian
// #5: Int8 TX power (dBm)
// #6-7: UInt16 SMac_requestTx() tick interval in milliseconds (Little-Endian)
#define SMACNPI_CONTROL_GET_RF             (0x02)
// SET_CENTERFREQ receives a UInt32 (4-byte) value in Little-Endian
#define SMACNPI_CONTROL_SET_CENTERFREQ     (0x03)
// SET_TXPOWER receives an Int8 (1-byte) value in dBm
#define SMACNPI_CONTROL_SET_TXPOWER        (0x04)
// SET_RF_ON receives a UInt8 boolean, 0 = off, !0 = on
#define SMACNPI_CONTROL_SET_RF_ON          (0x05)
// SET_ALTERNATE_ADDR configures a 2nd address the base station MCU will respond to besides its own IEEE address.
#define SMACNPI_CONTROL_SET_ALTERNATE_ADDR (0x06)
// GET_ADDRESSES returns 8 bytes, first 4 being local IEEE addr (UInt32 Little-Endian), last 4 being the Alternate address if present, or 0 if not.
#define SMACNPI_CONTROL_GET_ADDRESSES      (0x07)
// RUN_TX forces execution of SMac_requestTx() immediately
#define SMACNPI_CONTROL_RUN_TX             (0x08)
// SET_TX_TICK configures the automatic SMac_requestTx() interval; 0 disables it (thus requiring RUN_TX to transmit)
// Value received by host is UInt16 (little-endian) in milliseconds.
#define SMACNPI_CONTROL_SET_TX_TICK        (0x09)
#define SMACNPI_CONTROL_GET_IDENTIFIER     (0x10)
// SET_LEDS modifies the master LED_MASTER_ENABLE bit for the LED manager functions,
// then runs updateLeds() to enforce them.  LED status is preserved across disable/enable cycles
// and will be internally updated by the requisite events (RX on, off, packet TX or RX) regardless of enable status.
// Takes a 1-byte value, 0 = off, anything else = on
#define SMACNPI_CONTROL_SET_LEDS           (0x11)
// TODO: Add an instrumentation request control frame to read statistics

/* NPI Control frame - expected data payload size from host */
#define SMACNPI_CONTROL_UNSQUELCH_HOST__LEN       (0)
#define SMACNPI_CONTROL_SQUELCH_HOST__LEN         (0)
#define SMACNPI_CONTROL_GET_RF__LEN               (0)
#define SMACNPI_CONTROL_SET_CENTERFREQ__LEN       (4)
#define SMACNPI_CONTROL_SET_TXPOWER__LEN          (1)
#define SMACNPI_CONTROL_SET_RF_ON__LEN            (1)
#define SMACNPI_CONTROL_SET_ALTERNATE_ADDR__LEN   (4)
#define SMACNPI_CONTROL_GET_ADDRESSES__LEN        (8)
#define SMACNPI_CONTROL_RUN_TX__LEN               (0)
#define SMACNPI_CONTROL_SET_TX_TICK__LEN          (2)
#define SMACNPI_CONTROL_GET_IDENTIFIER__LEN       (0)
#define SMACNPI_CONTROL_SET_LEDS__LEN             (1)

/* NPI Control frame reply - status */
#define SMACNPI_CONTROL_STATUS_OK                      (0x00)
#define SMACNPI_CONTROL_STATUS_UNKNOWN_CMD             (0x01)
#define SMACNPI_CONTROL_STATUS_MALFORMED_CTRL          (0x02)
#define SMACNPI_CONTROL_STATUS_PARAMETER_OUT_OF_BOUNDS (0x03)
#define SMACNPI_CONTROL_STATUS_FEATURE_NOT_IMPLEMENTED (0x04)
#define SMACNPI_CONTROL_STATUS_ERROR                   (0x05)

/* NPI RTOS task threads */
Void smacnpi_outboundRfTaskFxn(UArg arg0, UArg arg1);
Void smacnpi_inboundRfTaskFxn(UArg arg0, UArg arg1);
Void smacnpi_controlTaskFxn(UArg arg0, UArg arg1);
#define SMACNPI_CONTROL_TASK_IS_SEPARATE_THREAD 0

// TODO: smacnpi_transmitTick Clock tick issues SMac_requestTx

/* NPI RTOS threads - stack sizes */
#define SMACNPI_THREAD_OUTBOUNDRF_STACKSIZE 768
#define SMACNPI_THREAD_INBOUNDRF_STACKSIZE 768
#define SMACNPI_THREAD_CONTROL_STACKSIZE 1536

/* NPI miscellaneous functions */
Void smacnpi_uartread_callback(UART_Handle, Void *, size_t);
Void smacnpi_uartwrite_callback(UART_Handle, Void *, size_t);
Void smacnpi_rfRx(Int8, UInt32, UInt16, UInt8, Void *);


#endif /* SMAC_NPI_H_ */
