/* SMac Network Processor Interface
 *
 * Main codebase
 */

#include "smac_npi.h"

/* NPI inter-thread communication and ring buffers */

// UART handle
UART_Handle uart;

// GPIO handle
static PIN_Handle ledPinHandle;

// GPIO status - used to cancel LED outputs
static UInt32 ledMasterStatus = 0;
#define LED_MASTER_ENABLED 0x8000
#define LED_RLED 0x01
#define LED_GLED 0x02

// Mailbox - UART_read callback to outboundRfTask
typedef struct {
	UInt32 dstAddr;
	UInt16 programID;
	UInt8 length;
	UInt8 ringID;
	Void * data;
} frameReadData_t;
Mailbox_Struct uartInputFrameStruct;
Mailbox_Handle uartInputFrame;

// The buffers within uartInputFrameRing[] will be values for frameReadData_t's *data pointer.
UInt8 uartInputFrameRing[SMACNPI_UARTREAD_FRAME_RING_DEPTH][SMAC_MAXIMUM_FRAMESIZE+8];
volatile UInt32 uartInputFrameRingMask = 0, uartInputFrameCurrentRing = 0;
volatile UInt32 uartInputFrameCurrentLen = 0, uartInputFrameTotalLen = 0;

// Outbound RF - Signal the TX tick that we may proceed to transmit
Semaphore_Struct rfTxReadyStruct;
Semaphore_Handle rfTxReady;

// Control data
typedef struct {
	UInt8 command;
	UInt8 length;
	UInt8 data[SMACNPI_CONTROL_DATA_MAXLEN];
} controlData_t;
Mailbox_Struct controlInputStruct;
Mailbox_Handle controlInput;

// Inbound RF - frame receiver
// typedef void(*SMac_RxCallback)(UInt32 srcAddr, UInt16 programID, UInt8 payloadLen, Void *payload);  // Packet callback function type
typedef struct {
	Int8 rssi;
	UInt32 srcAddr;
	UInt16 programID;
	UInt8 payloadLen;
	UInt8 data[SMAC_MAXIMUM_FRAMESIZE];
} rfInboundFrame_t;
Mailbox_Struct rfFrameReceiverStruct;
Mailbox_Handle rfFrameReceiver;

// Inbound RF - UART_write ring buffer
UInt8 uartOutputRing[SMACNPI_UARTWRITE_RING_SIZE];
volatile UInt8 *uartOutputWriteHead, *uartOutputReadHead;
volatile UInt32 uartOutputRingWritten = 0;
Semaphore_Struct uartOutputInactiveStruct;
Semaphore_Handle uartOutputInactive;
// Helper inline func's for managing the UART_write ring buffer
inline Int uartOutputRing_Written() {
	return uartOutputRingWritten;
}
inline Int uartOutputRing_Capacity() {
	return SMACNPI_UARTWRITE_RING_SIZE - uartOutputRingWritten;
}
inline Int uartOutputRing_Contiguous() {
	Int len = 0;
	if (uartOutputReadHead > uartOutputWriteHead) {
		len = (&uartOutputRing[SMACNPI_UARTWRITE_RING_SIZE] - uartOutputReadHead);
	} else {
		len = uartOutputWriteHead - uartOutputReadHead;
	}
	return len;
}
inline UInt32 uartOutputRing_submit(Void *data, UInt32 len) {
	UInt32 sz = 0;
	UInt8 *cdata = (UInt8 *)data;

	if (uartOutputRing_Capacity() < 1) {
		return 0;
	}
	for (sz=0; sz < len; sz++) {
		*uartOutputWriteHead = cdata[sz];
		uartOutputRingWritten++;
		uartOutputWriteHead++;
		if (uartOutputWriteHead == &uartOutputRing[SMACNPI_UARTWRITE_RING_SIZE]) {
			uartOutputWriteHead = &uartOutputRing[0];
		}
		if (uartOutputWriteHead == uartOutputReadHead) {
			// Ring buffer is full
			break;
		}
	}
	return sz;
}

/* Global state variables and RTOS thread information */
Task_Struct smacnpi_outboundRfStruct, smacnpi_inboundRfStruct, smacnpi_controlStruct;
UInt8 smacnpi_outboundRfStack[SMACNPI_THREAD_OUTBOUNDRF_STACKSIZE], smacnpi_inboundRfStack[SMACNPI_THREAD_INBOUNDRF_STACKSIZE];
#if defined(SMACNPI_CONTROL_TASK_IS_SEPARATE_THREAD) && SMACNPI_CONTROL_TASK_IS_SEPARATE_THREAD == 1
UInt8 smacnpi_controlStack[SMACNPI_THREAD_CONTROL_STACKSIZE];
#endif
volatile Bool smacnpi_rf_on, smacnpi_host_squelched;
UInt16 smacnpi_requesttx_tick;
UInt32 smacnpi_center_frequency;
Int8 smacnpi_tx_power;

/* Helper functions for Control thread */
static UInt composeControlReplyFrame(UInt8 *buf, UInt8 cmd, UInt8 status, UInt8 dataLen, const void *data);
static Bool submitControlReplyFrame(UInt frameLen, UInt8 *frame);
static Void sendSimpleCmdReply(UInt8 *buf, UInt8 cmd, UInt8 status); // Issues System_printf instrumenting UART output ring buffer overruns
static Void updateLeds();  // Uses ledMasterStatus
// --- None of these LED functions following actually update the LED GPIOs; you need to call updateLeds() after running them.
static Void setAllLeds(Bool);  // Toggle LED_MASTER_ENABLE
static Void setLed(UInt32, Bool); // Set specified LED (use LED_GLED or LED_RLED for the mask)
static Void toggleLed(UInt32); // Toggle specified LED
// ---

/* RTOS task - RF transmitter */
Void smacnpi_outboundRfTaskFxn(UArg arg0, UArg arg1)
{
	frameReadData_t rD;
	UInt8 tmpbuf[5+SMACNPI_CONTROL_DATA_MAXLEN];

	while (1) {
		// Get SMac frame from UART_read ring buffer
		Mailbox_pend(uartInputFrame, &rD, BIOS_WAIT_FOREVER);
		// Submit to SMac
		Bool ret = SMac_submitTx(NULL, rD.dstAddr, rD.programID, (UInt8)rD.length, rD.data);
		if (!ret) {
			// Send a control frame response back indicating overflow, then wait for TX queue to clear.
			System_printf("SMac_submitTx failed; overflow, squelching host and waiting for TX queue to clear\n"); System_flush();
			sendSimpleCmdReply(tmpbuf, SMACNPI_CONTROL_SQUELCH_HOST, SMACNPI_CONTROL_STATUS_OK);

			SMac_pendTxQueue(1000);  // Wait up to 1 second for TX queue to clear

			sendSimpleCmdReply(tmpbuf, SMACNPI_CONTROL_UNSQUELCH_HOST, SMACNPI_CONTROL_STATUS_OK);
			System_printf("Sending again-\n"); System_flush();
			SMac_submitTx(NULL, rD.dstAddr, rD.programID, (UInt8)rD.length, rD.data);  // Silently ignore if this fails
		}
		// Can't free up this buffer here since SMac needs it to stick around until TX;
		// the uartInputFrameRingMask will be cleared out by RUN_TX
	}
}

// TODO: Write smacnpi_transmitTicker for periodically issuing SMac_requestTx()

/* Helper function - compute checksum */
inline UInt8 xor_buffer(Void *buf, size_t len) {
	Int i;
	UInt8 *cbuf = (UInt8 *)buf, xor;

	if (len < 1) {
		return 0;
	}
	xor = cbuf[0];
	for (i=1; i < len; i++) {
		xor ^= cbuf[i];
	}
	return xor;
}

inline UInt8 xor_packet(UInt32 addr, UInt16 prID, Int8 rssi, UInt8 payloadLen, Void *payload) {
	Int i;
	UInt8 *cbuf = (UInt8 *)payload, xor;

	xor = (UInt8)addr;
	xor ^= (UInt8)(addr >> 8);
	xor ^= (UInt8)(addr >> 16);
	xor ^= (UInt8)(addr >> 24);
	xor ^= (UInt8)prID;
	xor ^= (UInt8)(prID >> 8);
	xor ^= (UInt8)rssi;
	xor ^= payloadLen;
	for (i=0; i < payloadLen; i++) {
		xor ^= cbuf[i];
	}
	return xor;
}

/* UART_read callback */
/* One thing that makes this more complicated is the fact that it's not a real "ring buffer", rather it's
 * a "ring" of position-sensitive buffers where UART_read should always dump data into the start of the buffer
 * UNLESS we have a valid NPI packet (StartChar of 0xAE or 0xBD) already (partially) stored in there.
 *
 * Following a successful Go implementation of this (for the host-side), we're using a big while() loop that
 * iterates through the read buffer one byte-at-a-time.
 */
Void smacnpi_uartread_callback(UART_Handle h, Void *data, size_t count)
{
	UInt8 *cdata = (UInt8 *)data, *readNext;
	UInt8 *bufStart;
	Int i;
	frameReadData_t frame;
	controlData_t ctrlFrame;

	while (count > 0) {
		bufStart = &uartInputFrameRing[uartInputFrameCurrentRing][0];
		if (uartInputFrameCurrentLen == 0) {
			if (*cdata == 0xAE || *cdata == 0xBD) {
				// Start of Frame discovered: Copy cdata[0:count] to start of buffer and reassign cdata to start of buffer
				for (i=0; i < count; i++) {
					bufStart[i] = cdata[i];
				}
				cdata = bufStart;
				// Advance cdata past the StartChar and loop back around
				cdata++;
				count--;
				uartInputFrameCurrentLen++;
				continue;
			}
			// If we see a valid StartChar, we'll never reach this next statement:
			readNext = bufStart;  // Without a valid StartChar, ensure we read into start of buffer next time
		}

		if (uartInputFrameCurrentLen > 0) {
			if (uartInputFrameTotalLen == 0) {
				if (uartInputFrameCurrentLen == 8 && bufStart[0] == 0xAE) {
					uartInputFrameTotalLen = 10 + *cdata;
				}
				if (uartInputFrameCurrentLen == 2 && bufStart[0] == 0xBD) {
					uartInputFrameTotalLen = 4 + *cdata;
				}
			}
			bufStart[uartInputFrameCurrentLen] = *cdata;
			uartInputFrameCurrentLen++;
			// With a valid StartChar, ensure we append reads to the valid running packet
			readNext = &bufStart[uartInputFrameCurrentLen];
		}

		// Check boundaries of uartInputFrameTotalLen for sanity's sake
		if (uartInputFrameTotalLen > 0) {
			if (bufStart[0] == 0xAE) {
				if (uartInputFrameTotalLen > 10+SMAC_MAXIMUM_FRAMESIZE) {
					uartInputFrameCurrentLen = 0;
					uartInputFrameTotalLen = 0;
					readNext = bufStart;
				}
			}
			if (bufStart[0] == 0xBD) {
				if (uartInputFrameTotalLen > 4+SMACNPI_CONTROL_DATA_MAXLEN) {
					uartInputFrameCurrentLen = 0;
					uartInputFrameTotalLen = 0;
					readNext = bufStart;
				}
			}
		}

		if (uartInputFrameTotalLen > 0 && uartInputFrameCurrentLen == uartInputFrameTotalLen) {
			// Completed frame; verify checksum and send it on its way
			UInt8 xor = xor_buffer(&bufStart[1], uartInputFrameCurrentLen-2); // ignore cksum char at the end & the StartChar when computing cksum
			if (bufStart[0] == 0xAE) {
				if (xor == bufStart[9+bufStart[8]]) {
					// If successful cksum, advance uartInputFrameCurrentRing to a new unused buffer, send current buffer to relevant Mailbox
					// Advance uartInputFrameCurrentRing to a new unused buffer
					frame.ringID = (UInt8)uartInputFrameCurrentRing;  // Save current buffer ring# before we modify it
					uartInputFrameRingMask |= 1 << uartInputFrameCurrentRing;
					UInt32 mask = 1;
					for (i=0; i < SMACNPI_UARTREAD_FRAME_RING_DEPTH; i++) {
						if ((uartInputFrameRingMask & mask) == 0) {
							uartInputFrameCurrentRing = i;
							readNext = &uartInputFrameRing[uartInputFrameCurrentRing][0];
							break;
						}
						mask *= 2;
					}
					if (i != SMACNPI_UARTREAD_FRAME_RING_DEPTH) {
						frame.dstAddr = *((UInt32 *)(bufStart+1));
						frame.programID = *((UInt16 *)(bufStart+5));
						frame.length = bufStart[8];
						frame.data = bufStart + 9;
						Mailbox_post(uartInputFrame, &frame, BIOS_NO_WAIT);
						// Silently discards frame if mailbox is full
						// TODO: Instrument any discards
					} else {
						// If we're all out of rings, just keep it as is, but don't post the Mailbox.
						// TODO: Instrument this (uartInputFrameRing full)
						uartInputFrameRingMask &= ~(1 << frame.ringID);
						readNext = bufStart;
					}
				} else {
					// If unsuccessful cksum, keep uartInputFrameCurrentRing the same and move to the next part
					//System_printf("XOR didn't check out\n"); System_flush();
					readNext = bufStart;
				}
			}
			if (bufStart[0] == 0xBD) {
				// Only post control frame if successful checksum & data isn't too long
				if (xor == bufStart[3+bufStart[2]]) {
					// Process control frame and post it to Control mailbox
					ctrlFrame.length = bufStart[2];
					ctrlFrame.command = bufStart[1];
					memcpy(&ctrlFrame.data[0], &bufStart[3], bufStart[2]);
					Mailbox_post(controlInput, &ctrlFrame, BIOS_NO_WAIT);
				}
				// Reuse the same buffer afterwards since we copy everything (including data) to the mailbox
				readNext = bufStart;
			}
			// Reset uartInputFrameCurrentLen, uartInputFrameTotalLen, copy remaining data to new buffer and issue UART_read
			uartInputFrameCurrentLen = uartInputFrameTotalLen = 0;

		}

		cdata++;
		count--;
	} // while (count > 0)

	UART_read(h, readNext, 8+SMAC_MAXIMUM_FRAMESIZE - uartInputFrameCurrentLen);
}

/* RTOS task - RF receiver */
#define TASK_SLEEP_QUARTER_FULL_FRAME
Void smacnpi_inboundRfTaskFxn(UArg arg0, UArg arg1)
{
	rfInboundFrame_t frame;
	UInt32 waitTime;

	// Code which estimates how long we should pause to wait for the UART output ring buffer to flush enough to hold our next frame,
	// if it's found to be full (or near-full) when we go to send an RX packet to the host.
	if ( (10+SMAC_MAXIMUM_FRAMESIZE) * 10 > SMACNPI_SERIAL_BAUDRATE ) {
		waitTime = 100000;  // If a full frame can't be sent in the span of 1 second, just default to sleeping 100ms
	} else {
		waitTime = (10+SMAC_MAXIMUM_FRAMESIZE) * 10;  // Bits per full frame including Start and Stop bits
		waitTime = SMACNPI_SERIAL_BAUDRATE / waitTime;  // Obtain it as a fraction of serial bitrate
		waitTime = 1000000 / waitTime;  // And divide into 1000000 to get # of microseconds per frame
		if (waitTime < Clock_tickPeriod) {
			waitTime = Clock_tickPeriod;
		} else {
			waitTime /= 4;
			if (waitTime < Clock_tickPeriod) {
				waitTime = Clock_tickPeriod;
			}
		}
	}
	// Main loop
	while (1) {
		// Get an inbound frame from the SMac_registerAllRx callback
		Mailbox_pend(rfFrameReceiver, &frame, BIOS_WAIT_FOREVER);
		// Wait until the UART Output Ring has enough capacity for the maximum size of a single RF frame-
		// (and until then, the Mailbox is our "buffer" for additional incoming RX frames)
		while (uartOutputRing_Capacity() < 9+SMAC_MAXIMUM_FRAMESIZE) {
			// Task_sleep for roughly 1/4 the time it'd take to transmit a whole "full" frame over the serial port
			Task_sleep(waitTime / Clock_tickPeriod);
		}
		// Submit to UART write ring buffer
		UInt32 i = 10+frame.payloadLen;  // total size of frame to be written
		if (uartOutputRing_Capacity() >= i) {  // Should always test true thanks to our while() Task_sleep loop above
			// Submit frame to UART output
			UInt8 xor = xor_packet(frame.srcAddr, frame.programID, frame.rssi, frame.payloadLen, frame.data);
			// XOR checksum calculated; submit packet
			UInt8 startChar = 0xAE;
			uartOutputRing_submit(&startChar, 1);
			uartOutputRing_submit(&frame.srcAddr, 4);  // Arch native little-endian
			uartOutputRing_submit(&frame.programID, 2); // Arch native little-endian
			uartOutputRing_submit(&frame.rssi, 1);
			uartOutputRing_submit(&frame.payloadLen, 1);
			uartOutputRing_submit(&frame.data[0], frame.payloadLen);
			uartOutputRing_submit(&xor, 1);

			Bool ret = Semaphore_pend(uartOutputInactive, BIOS_NO_WAIT);
			if (ret) {
				// No active UART_write process, so, start it up
				UART_write(uart, (const void *)uartOutputReadHead, uartOutputRing_Contiguous());
			}
		} else {
			// Silently drop the frame; we've overrun our UART output buffer
			// This shouldn't happen with the while(uartOutputRing...) { Task_sleep(...) } loop above.
		}
	}
}

/* UART_write callback */
Void smacnpi_uartwrite_callback(UART_Handle h, Void *data, size_t count)
{
	if (count > uartOutputRingWritten) {
		System_printf("smacnpi_uartwrite_callback ASSERT failed: count > uartOutputRingWritten"); System_flush();
		BIOS_exit(1);
	}
	uartOutputRingWritten -= count;
	// Advance uartOutputReadHead
	while (count > 0) {
		uartOutputReadHead++;
		if (uartOutputReadHead == &uartOutputRing[SMACNPI_UARTWRITE_RING_SIZE]) {
			uartOutputReadHead = &uartOutputRing[0];
		}
		count--;
	}
	// Begin the next UART_write() or signal that writes are done (inactive).
	if (uartOutputRingWritten > 0) {
		UART_write(h, (const void *)uartOutputReadHead, uartOutputRing_Contiguous());
	} else {
		Semaphore_post(uartOutputInactive);
	}
}

/* SMac callback - All RX frames */
Void smacnpi_rfRx(Int8 rssi, UInt32 srcAddr, UInt16 programID, UInt8 payloadLen, Void *payload)
{
	rfInboundFrame_t fr;
	fr.rssi = rssi;
	fr.srcAddr = srcAddr;
	fr.programID = programID;
	fr.payloadLen = payloadLen;
	memcpy(&fr.data[0], (UInt8 *)payload, fr.payloadLen);
	Bool ret = Mailbox_post(rfFrameReceiver, &fr, BIOS_NO_WAIT);  // Silently drop frame if the mailbox is full
	toggleLed(LED_RLED);
	updateLeds();
	if (!ret) {
		// TODO: Instrument the frame drop
	}
}

/* RTOS task - Control thread */
Void smacnpi_controlTaskFxn(UArg arg0, UArg arg1)
{
	ledPinHandle = (PIN_Handle)arg0;

	System_printf("smacnpi_controlTaskFxn begun\n"); System_flush();

	// Stuff to init-
	Mailbox_Params mP0, mP1, mP2;
	Error_Block eB0, eB1, eB2;

	// UART received OTA RF frames (OTA frames going from host -> airwaves)
	Mailbox_Params_init(&mP0);
	Error_init(&eB0);
	mP0.instance->name = "uartInputFrame";
	Mailbox_construct(&uartInputFrameStruct, sizeof(frameReadData_t), SMACNPI_UARTREAD_FRAME_RING_DEPTH, &mP0, &eB0);
	uartInputFrame = Mailbox_handle(&uartInputFrameStruct);

	// UART received control frames (host -> MCU)
	Mailbox_Params_init(&mP1);
	Error_init(&eB1);
	mP1.instance->name = "controlInput";
	Mailbox_construct(&controlInputStruct, sizeof(controlData_t), SMACNPI_CONTROL_FRAME_PENDING, &mP1, &eB1);
	controlInput = Mailbox_handle(&controlInputStruct);

	// RF inbound frames (airwaves -> host).  Beware, this is a biggie.  Any memory issues, take a peek at this one.
	Mailbox_Params_init(&mP2);
	Error_init(&eB2);
	mP2.instance->name = "rfFrameReceiver";
	Mailbox_construct(&rfFrameReceiverStruct, sizeof(rfInboundFrame_t), SMACNPI_RFINBOUND_FRAME_RING_DEPTH, &mP2, &eB2);
	rfFrameReceiver = Mailbox_handle(&rfFrameReceiverStruct);

	// Indicates whether we have an active UART_write() in progress (UART_write callback posts this when its ring buffer is empty)
	Semaphore_Params sP;
	Semaphore_Params_init(&sP);
	sP.mode = Semaphore_Mode_BINARY;
	Semaphore_construct(&uartOutputInactiveStruct, 1, &sP);  // Defaults to 1 to ensure we know UART_write is inactive by default
	uartOutputInactive = Semaphore_handle(&uartOutputInactiveStruct);

	// Set default runtime parameters
	smacnpi_rf_on = SMACNPI_RF_DEFAULT_RX_ON;
	smacnpi_host_squelched = false;
	smacnpi_requesttx_tick = 100;
	smacnpi_center_frequency = SMACNPI_RF_DEFAULT_CENTERFREQ;
	smacnpi_tx_power = SMACNPI_RF_DEFAULT_TXPOWER;
	uartOutputReadHead = &uartOutputRing[0];
	uartOutputWriteHead = &uartOutputRing[0];
	uartOutputRingWritten = 0;

	// Initialize the SMac library and RF engine
	if (!SMac_open(smacnpi_center_frequency, smacnpi_tx_power)) {
		System_printf("Error running SMac_open.  Cannot proceed.  Halting.\n");
		System_flush();

		System_abort("SMACNPI HALTED.");
		while(1) ;
	}
	System_printf("SMac opened\n"); System_flush();

	// Initialize a Clock ticker task to pace RF transmits
	// TODO

	// Open the UART
	UART_Params uP;
	UART_Params_init(&uP);
	uP.baudRate = SMACNPI_SERIAL_BAUDRATE;
	uP.writeDataMode = UART_DATA_BINARY;
	uP.readDataMode = UART_DATA_BINARY;
	uP.writeMode = UART_MODE_CALLBACK;
	uP.writeCallback = smacnpi_uartwrite_callback;
	uP.readMode = UART_MODE_CALLBACK;
	uP.readCallback = smacnpi_uartread_callback;
	uart = UART_open(0, &uP);
	if (uart == NULL) {
		System_printf("Error running UART_open.  Cannot proceed.  Halting.\n");
		System_flush();

		System_abort("SMACNPI HALTED.");
		while(1) ;
	}
	System_printf("UART open\n"); System_flush();
    // CC13xx/CC26xx custom UART control feature - RETURN PARTIAL
	#ifdef UARTCC26XX_CMD_RETURN_PARTIAL_ENABLE
    UART_control(uart, UARTCC26XX_CMD_RETURN_PARTIAL_ENABLE, NULL);
	#endif

	// Launch worker threads
	Task_Params tP;
	// Launch Outbound RF worker task
	Task_Params_init(&tP);
	tP.stackSize = SMACNPI_THREAD_OUTBOUNDRF_STACKSIZE;
	tP.stack = &smacnpi_outboundRfStack[0];
	tP.instance->name = "outboundRf";
	Task_construct(&smacnpi_outboundRfStruct, (Task_FuncPtr)smacnpi_outboundRfTaskFxn, &tP, NULL);

	// Launch Inbound RF worker task
	Task_Params_init(&tP);
	tP.stackSize = SMACNPI_THREAD_INBOUNDRF_STACKSIZE;
	tP.stack = &smacnpi_inboundRfStack[0];
	tP.instance->name = "inboundRf";
	Task_construct(&smacnpi_inboundRfStruct, (Task_FuncPtr)smacnpi_inboundRfTaskFxn, &tP, NULL);
	System_printf("smacnpi Worker threads launched\n"); System_flush();

	// Configure SMac RX all-programID callback to smacnpi_rfRx
	SMac_registerAllRx(smacnpi_rfRx);

	// Main loop, process received control frames.  RF link is off by default.
	controlData_t ctl;
	UInt8 ctlReplyBuffer[SMACNPI_CONTROL_DATA_MAXLEN + 5];  // StartChar, Cmd, Status, Len, (payload), Cksum
	UInt8 tmpbuf[SMACNPI_CONTROL_DATA_MAXLEN];
	UInt fLen;
	UInt32 addr;
	const Char *smacnpi_id = SMACNPI_IDENTIFIER;

	// Now that NPI startup initialization is (mostly) complete, clear the LEDs.
	ledMasterStatus = LED_MASTER_ENABLED;
	updateLeds();
	if (smacnpi_rf_on) {
		SMac_enableRx(0);
		setLed(LED_GLED, 1);  // Green means RX is ON
		updateLeds();
	}

	// Initiate UART reads
	System_printf("smacnpi Begin UART read\n"); System_flush();
	uartInputFrameCurrentRing = 0;
	uartInputFrameRingMask = 0;  // All ring buffers available for use
	uartInputFrameCurrentLen = uartInputFrameTotalLen = 0;
	UART_read(uart, &uartInputFrameRing[uartInputFrameCurrentRing][0], 8+SMAC_MAXIMUM_FRAMESIZE);

	System_printf("smacnpi Control Thread Main Loop Begin\n"); System_flush();

	while (1) {
		Mailbox_pend(controlInput, &ctl, BIOS_WAIT_FOREVER);
		//System_printf("CTRL frame: cmd=%x, dataLen=%d\n", ctl.command, ctl.length); System_flush();
		switch (ctl.command) {
		case SMACNPI_CONTROL_GET_RF:
			if (ctl.length == SMACNPI_CONTROL_GET_RF__LEN) {
				// Byte #0: On/Off
				// Byte #1-4: RF center frequency (UInt32, little-endian)
				// Byte #5: TX power (Int8)
				// Byte #6-7: SMac_requestTx() tick interval in milliseconds (UInt16, little-endian)
				tmpbuf[0] = smacnpi_rf_on;
				memcpy(tmpbuf+1, &smacnpi_center_frequency, 4);
				tmpbuf[5] = (UInt8)smacnpi_tx_power;
				memcpy(tmpbuf+6, &smacnpi_requesttx_tick, 2);
				fLen = composeControlReplyFrame(ctlReplyBuffer, ctl.command, SMACNPI_CONTROL_STATUS_OK, 8, tmpbuf);
				if (!submitControlReplyFrame(fLen, ctlReplyBuffer)) {
					System_printf("GET_RF reply: UART ring buffer overrun\n");
					System_flush();
				}
			} else {
				sendSimpleCmdReply(ctlReplyBuffer, ctl.command, SMACNPI_CONTROL_STATUS_MALFORMED_CTRL);
			}
			break;
		case SMACNPI_CONTROL_SET_CENTERFREQ:
			if (ctl.length == SMACNPI_CONTROL_SET_CENTERFREQ__LEN) {
				UInt32 freq = *((UInt32 *)&ctl.data[0]);
				// Check frequency boundaries based on CC1310 datasheet, Specifications, page 13 (SWRS181B)
				if (freq < 300000000 || (freq > 348000000 && freq < 400000000) ||
						(freq > 435000000 && freq < 470000000) ||
						(freq > 510000000 && freq < 779000000) ||
						(freq > 787000000 && freq < 863000000) ||
						freq > 930000000) {
					sendSimpleCmdReply(ctlReplyBuffer, ctl.command, SMACNPI_CONTROL_STATUS_PARAMETER_OUT_OF_BOUNDS);
				}
				smacnpi_center_frequency = freq;
				SMac_setFrequency(freq);
				sendSimpleCmdReply(ctlReplyBuffer, ctl.command, SMACNPI_CONTROL_STATUS_OK);
			} else {
				sendSimpleCmdReply(ctlReplyBuffer, ctl.command, SMACNPI_CONTROL_STATUS_MALFORMED_CTRL);
			}
			break;
		case SMACNPI_CONTROL_SET_TXPOWER:
			if (ctl.length == SMACNPI_CONTROL_SET_TXPOWER__LEN) {
				Int8 p = (Int8)ctl.data[0];
				if (p < -10 || (p > -10 && p < 0) || (p > 14)) {
					sendSimpleCmdReply(ctlReplyBuffer, ctl.command, SMACNPI_CONTROL_STATUS_PARAMETER_OUT_OF_BOUNDS);
				} else {
					smacnpi_tx_power = p;
					SMac_setTxPower(p);
					sendSimpleCmdReply(ctlReplyBuffer, ctl.command, SMACNPI_CONTROL_STATUS_OK);
				}
			} else {
				sendSimpleCmdReply(ctlReplyBuffer, ctl.command, SMACNPI_CONTROL_STATUS_MALFORMED_CTRL);
			}
			break;
		case SMACNPI_CONTROL_SET_RF_ON:
			if (ctl.length == SMACNPI_CONTROL_SET_RF_ON__LEN) {
				if (ctl.data[0] != 0) {
					SMac_enableRx(0);
					setLed(LED_GLED, 1);
					updateLeds();
				} else {
					SMac_disableRx();
					setLed(LED_GLED, 0);
					updateLeds();
				}
				sendSimpleCmdReply(ctlReplyBuffer, ctl.command, SMACNPI_CONTROL_STATUS_OK);
			} else {
				sendSimpleCmdReply(ctlReplyBuffer, ctl.command, SMACNPI_CONTROL_STATUS_MALFORMED_CTRL);
			}
			break;
		case SMACNPI_CONTROL_SET_ALTERNATE_ADDR:
			if (ctl.length == SMACNPI_CONTROL_SET_ALTERNATE_ADDR__LEN) {
				addr = *((UInt32 *)(&ctl.data[0]));
				SMac_setAdditionalRxAddress(addr);
				sendSimpleCmdReply(ctlReplyBuffer, ctl.command, SMACNPI_CONTROL_STATUS_OK);
			} else {
				sendSimpleCmdReply(ctlReplyBuffer, ctl.command, SMACNPI_CONTROL_STATUS_MALFORMED_CTRL);
			}
			break;
		case SMACNPI_CONTROL_GET_ADDRESSES:
			if (ctl.length == SMACNPI_CONTROL_GET_ADDRESSES__LEN) {
				addr = SMac_getIeeeAddress();
				memcpy(tmpbuf, &addr, 4);
				addr = SMac_getAdditionalRxAddress();
				memcpy(tmpbuf+4, &addr, 4);

				fLen = composeControlReplyFrame(ctlReplyBuffer, ctl.command, SMACNPI_CONTROL_STATUS_OK, 8, tmpbuf);
				if (!submitControlReplyFrame(fLen, ctlReplyBuffer)) {
					System_printf("GET_ADDRESSES reply: UART ring buffer overrun\n");
					System_flush();
				}
			} else {
				sendSimpleCmdReply(ctlReplyBuffer, ctl.command, SMACNPI_CONTROL_STATUS_MALFORMED_CTRL);
			}
			break;
		case SMACNPI_CONTROL_RUN_TX:
			if (ctl.length == SMACNPI_CONTROL_RUN_TX__LEN) {
				toggleLed(LED_RLED);
				updateLeds();
				SMac_requestTx();
				if (SMac_pendTxQueue(1000)) {
					sendSimpleCmdReply(ctlReplyBuffer, ctl.command, SMACNPI_CONTROL_STATUS_OK);
				} else {
					sendSimpleCmdReply(ctlReplyBuffer, ctl.command, SMACNPI_CONTROL_STATUS_ERROR);
				}
				uartInputFrameRingMask = 0;  // Clear all ring buffers since we've transmitted everything
			} else {
				sendSimpleCmdReply(ctlReplyBuffer, ctl.command, SMACNPI_CONTROL_STATUS_MALFORMED_CTRL);
			}
			break;
		case SMACNPI_CONTROL_SET_TX_TICK:
			if (ctl.length == SMACNPI_CONTROL_SET_TX_TICK__LEN) {
				// Not implemented yet
				// TODO
				sendSimpleCmdReply(ctlReplyBuffer, ctl.command, SMACNPI_CONTROL_STATUS_FEATURE_NOT_IMPLEMENTED);
			} else {
				sendSimpleCmdReply(ctlReplyBuffer, ctl.command, SMACNPI_CONTROL_STATUS_MALFORMED_CTRL);
			}
			break;
		case SMACNPI_CONTROL_GET_IDENTIFIER:
			if (ctl.length == SMACNPI_CONTROL_GET_IDENTIFIER__LEN) {
				size_t idlen = strlen(smacnpi_id);
				if (idlen > SMACNPI_CONTROL_DATA_MAXLEN) {
					idlen = SMACNPI_CONTROL_DATA_MAXLEN;
				}
				memcpy(tmpbuf, smacnpi_id, SMACNPI_CONTROL_DATA_MAXLEN);
				fLen = composeControlReplyFrame(ctlReplyBuffer, ctl.command, SMACNPI_CONTROL_STATUS_OK, idlen, tmpbuf);
				if (!submitControlReplyFrame(fLen, ctlReplyBuffer)) {
					System_printf("GET_IDENTIFIER reply: UART ring buffer overrun\n");
					System_flush();
				} else {
					System_printf("Sent back reply: %x %x %x %x ... %x\n",
							ctlReplyBuffer[0],
							ctlReplyBuffer[1],
							ctlReplyBuffer[2],
							ctlReplyBuffer[3],
							ctlReplyBuffer[fLen-1]); System_flush();
				}
			} else {
				sendSimpleCmdReply(ctlReplyBuffer, ctl.command, SMACNPI_CONTROL_STATUS_MALFORMED_CTRL);
			}
			break;
		case SMACNPI_CONTROL_SET_LEDS:
			if (ctl.length == SMACNPI_CONTROL_SET_LEDS__LEN) {
				if (ctl.data[0] != 0) {
					setAllLeds(1);
				} else {
					setAllLeds(0);
				}
				updateLeds();
				sendSimpleCmdReply(ctlReplyBuffer, ctl.command, SMACNPI_CONTROL_STATUS_OK);
			} else {
				sendSimpleCmdReply(ctlReplyBuffer, ctl.command, SMACNPI_CONTROL_STATUS_MALFORMED_CTRL);
			}
		default:
			// Complain back to the host that we don't know this control cmd
			sendSimpleCmdReply(ctlReplyBuffer, ctl.command, SMACNPI_CONTROL_STATUS_UNKNOWN_CMD);
			System_printf("Control thread received unknown cmd: %x\n", ctl.command);
			System_flush();
		}
	}
}

// Control frame replies
// Submit to UART write ring buffer

static UInt composeControlReplyFrame(UInt8 *buf, UInt8 cmd, UInt8 status, UInt8 dataLen, const void *data)
{
	UInt8 *cdata = (UInt8 *)data;
	UInt len = 0;
	Int i;

	buf[0] = 0xBA;  // Control reply frame
	buf[1] = cmd;
	buf[2] = status;
	buf[3] = dataLen;
	len = 4;
	for (i=0; i < dataLen; i++) {
		buf[len] = cdata[i];
		len++;
	}
	buf[len] = xor_buffer(&buf[1], len-1);
	len++;
	return len;
}

static Bool submitControlReplyFrame(UInt frameLen, UInt8 *frame)
{
	if (uartOutputRing_Capacity() >= frameLen) {
		// Submit frame to UART output ring buffer
		uartOutputRing_submit(frame, frameLen);

		// Check if an active UART_write() is in progress, if not, start it:
		Bool ret = Semaphore_pend(uartOutputInactive, BIOS_NO_WAIT);
		if (ret) {
			UART_write(uart, (const void *)uartOutputReadHead, uartOutputRing_Contiguous());
		}
		return true;
	}
	return false;
}

static Void sendSimpleCmdReply(UInt8 *buf, UInt8 cmd, UInt8 status)
{
	UInt len = composeControlReplyFrame(buf, cmd, status, 0, NULL);
	if (!submitControlReplyFrame(len, buf)) {
		System_printf("submitControlReplyFrame(cmd=%02x, status=%02x) UART output buffer overrun\n", cmd, status);
		System_flush();
	}
}

static Void updateLeds()
{
	if (ledMasterStatus & LED_MASTER_ENABLED) {
		if (ledMasterStatus & LED_RLED) {
			PIN_setOutputValue(ledPinHandle, Board_RLED, 1);
		} else {
			PIN_setOutputValue(ledPinHandle, Board_RLED, 0);
		}
		if (ledMasterStatus & LED_GLED) {
			PIN_setOutputValue(ledPinHandle, Board_GLED, 1);
		} else {
			PIN_setOutputValue(ledPinHandle, Board_GLED, 0);
		}
	} else {
		PIN_setOutputValue(ledPinHandle, Board_GLED, 0);
		PIN_setOutputValue(ledPinHandle, Board_RLED, 0);
	}
}

static Void setAllLeds(Bool onoff)
{
	if (onoff) {
		ledMasterStatus |= LED_MASTER_ENABLED;
	} else {
		ledMasterStatus &= ~LED_MASTER_ENABLED;
	}
}

static Void setLed(UInt32 mask, Bool onoff)
{
	if (onoff) {
		ledMasterStatus |= mask;
	} else {
		ledMasterStatus &= ~mask;
	}
}

static Void toggleLed(UInt32 mask)
{
	ledMasterStatus ^= mask;
}
