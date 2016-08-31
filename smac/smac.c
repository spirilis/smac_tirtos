/**
 * @brief SMac Codebase - All Radio and SMac API functions, no Base Station code here.
 *
 */

#include "smac.h"

// All stateful information goes in here
static SMac_Struct sMacState, *mac = &sMacState;
static Semaphore_Struct _binsem_TxEmpty_Struct, _binsem_rxSchedUpdate_Struct, _binsem_regUpdate_Struct, _binsem_RxOvf_Struct;
static Mailbox_Struct _queue_TxSubmit_Struct;
static Event_Struct _mainTaskEvents_Struct;
static Task_Struct _mainTask_Struct;
static UInt8 _maintask_Stack[SMAC_MAINTASK_STACKSIZE];
static UInt8 _txBuffer[SMAC_FRAMESIZE_ALLOCATION];

// Function prototypes for TI-RTOS Task fxn's relevant to SMac along with helper func's
static Void SMac_MainTaskFxn(UArg, UArg);
static Void SMac_RxCallbackFxn(RF_Handle, RF_CmdHandle, RF_EventMask);
static inline UInt SMac_util_getTxQueueDepth();
static Void SMac_util_clearTxQueue();
static inline SMac_PacketInternal * SMac_util_getFreeTxSlot();
static inline SMac_PacketInternal * SMac_util_getFirstTxEntry();
static inline SMac_PacketInternal * SMac_util_getNextTxEntryForDestAddr(UInt32 dstAddr);
// Accepts the start of a frame buffer, current index within it, copies packet in and returns # bytes LEFT in buffer or -1 if not enough space.
static inline Int SMac_util_StuffPacket(Void *frameBuf, UInt startPos, SMac_PacketInternal *pkt);
static inline SMac_RxCallback SMac_util_getCallbackByProgramID(UInt16);
static inline Bool SMac_util_ValidateOurAddress(UInt32);
static Void SMac_util_PrepareRxCmd(UInt32 timeoutMillis);
static Void SMac_util_ConfigureRxQueues();
static inline UInt SMac_util_GetRxQueueFreeCount();

// Ripped from EasyLink - TX power settings
typedef struct {
	int8_t dBm;
	uint16_t txPower;
} OutputConfig;

#define SMAC_TXPOWER_TABLE_SIZE 16
static const OutputConfig outputPower[] = {
    {  0, 0x0041 },
    {  1, 0x10c3 },
    {  2, 0x1042 },
    {  3, 0x14c4 },
    {  4, 0x18c5 },
    {  5, 0x18c6 },
    {  6, 0x1cc7 },
    {  7, 0x20c9 },
    {  8, 0x24cb },
    {  9, 0x2ccd },
    { 10, 0x38d3 },
    { 11, 0x50da },
    { 12, 0xb818 },
    { 13, 0xa73f }, /* 12.5 */
    { 14, 0xa73f },
    {-10, 0x08c0 },
};

// API functions
Bool SMac_registerRx(UInt16 prID, SMac_RxCallback cb)
{
	int i;

	Semaphore_pend(mac->binsem_regUpdate, BIOS_WAIT_FOREVER);
	for (i=0; i < SMAC_MAXIMUM_PROGRAMID_REGISTRATIONS; i++) {
		if (mac->programCallbacks[i].programID == prID) {
			#ifdef SMAC_DEBUG
			System_printf("SMac_registerRx replacing a cb @ prID=%x (newcb=%p)\n", prID, cb); System_flush();
			#endif
			mac->programCallbacks[i].callback = cb;
			Semaphore_post(mac->binsem_regUpdate);
			return true;
		}
		if (mac->programCallbacks[i].callback == NULL) {
			#ifdef SMAC_DEBUG
			System_printf("SMac_registerRx adding a cb @ prID=%x (cb=%p)\n", prID, cb); System_flush();
			#endif
			mac->programCallbacks[i].programID = prID;
			mac->programCallbacks[i].callback = cb;
			Semaphore_post(mac->binsem_regUpdate);
			return true;
		}
	}
	Semaphore_post(mac->binsem_regUpdate);
	#ifdef SMAC_DEBUG
	System_printf("SMac_registerRx: No more unused registration slots\n"); System_flush();
	#endif
	return false;  // No available program ID registration slots
}

Bool SMac_deregisterRx(UInt16 prID)
{
	int i;
	Bool didRemove = false;

	Semaphore_pend(mac->binsem_regUpdate, BIOS_WAIT_FOREVER);
	for (i=0; i < SMAC_MAXIMUM_PROGRAMID_REGISTRATIONS; i++) {
		if (mac->programCallbacks[i].programID == prID) {
			mac->programCallbacks[i].callback = NULL;
			mac->programCallbacks[i].programID = 0;
			didRemove = true;
			#ifdef SMAC_DEBUG
			System_printf("SMac_deregisterRx: Removed callback for prID=%x\n", prID); System_flush();
			#endif
		}
	}
	Semaphore_post(mac->binsem_regUpdate);
	return didRemove;
}

Bool SMac_registerAllRx(SMac_RxCallback cb)
{
	if (cb != NULL) {
		Semaphore_pend(mac->binsem_regUpdate, BIOS_WAIT_FOREVER);
		mac->allProgramCallback = cb;
		Semaphore_post(mac->binsem_regUpdate);
		return true;
	}
	return false;  // Callback passed is invalid
}

Bool SMac_deregisterAllRx(Void)
{
	Semaphore_pend(mac->binsem_regUpdate, BIOS_WAIT_FOREVER);
	if (mac->allProgramCallback != NULL) {
		mac->allProgramCallback = NULL;
		Semaphore_post(mac->binsem_regUpdate);
		return true;
	}
	Semaphore_post(mac->binsem_regUpdate);
	return false;  // No callback was previously defined
}

#define RXQUEUEBUFSIZE RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(SMAC_RXQUEUE_MAXDEPTH, SMAC_FRAMESIZE_ALLOCATION, 0)
UInt8 SMac_RxRFQueueBuffer[RXQUEUEBUFSIZE];

Void SMac_init(Void)
{
	mac->Frequency = 915000000;
	mac->TxPower = -10;
	mac->txCsWait = 500;  // Defaults to 1 second wait for carrier sense to go idle
	mac->rfEnable = false;
	mac->rxSched = SMAC_RXTYPE_DISABLED;
	mac->rxSched_Millis = 0;

	int i;
	for (i=0; i < SMAC_TXQUEUE_MAXDEPTH; i++) {
		mac->txQueue[i].data = NULL;
		mac->txQueue[i].length = 0;
		mac->txQueue[i].binsem_NotifySent = NULL;
		mac->txQueue[i].destAddr = 0;
		mac->txQueue[i].programID = 0;
	}

	for (i=0; i < SMAC_MAXIMUM_PROGRAMID_REGISTRATIONS; i++) {
		mac->programCallbacks[i].programID = 0;
		mac->programCallbacks[i].callback = NULL;
	}

	mac->allProgramCallback = NULL;

	mac->RxQueueBuffer = &SMac_RxRFQueueBuffer[0];

	mac->rxAddresses[0] = SMac_getIeeeAddress();  // Set this to chip-specific IEEE address (lower 32-bits should be unique among chips)
	mac->rxAddresses[1] = 0;
}

Bool SMac_open(UInt32 freq, Int8 txpwr)
{
	// Construct semaphores, event, mailbox and tasks
	Semaphore_Params sP;

	Semaphore_Params_init(&sP);
	sP.mode = Semaphore_Mode_BINARY;
	Semaphore_construct(&_binsem_TxEmpty_Struct, 0, &sP);
	mac->binsem_TxEmpty = Semaphore_handle(&_binsem_TxEmpty_Struct);
	Semaphore_construct(&_binsem_rxSchedUpdate_Struct, 1, &sP);
	mac->binsem_rxSchedUpdate = Semaphore_handle(&_binsem_rxSchedUpdate_Struct);
	Semaphore_construct(&_binsem_regUpdate_Struct, 1, &sP);
	mac->binsem_regUpdate = Semaphore_handle(&_binsem_regUpdate_Struct);
	Semaphore_construct(&_binsem_RxOvf_Struct, 0, &sP);
	mac->binsem_RxOvf = Semaphore_handle(&_binsem_RxOvf_Struct);

	Event_Params eP;
	Event_Params_init(&eP);
	Event_construct(&_mainTaskEvents_Struct, &eP);
	mac->mainTaskEvents = Event_handle(&_mainTaskEvents_Struct);

	Mailbox_Params mP;
	Error_Block eB;
	Error_init(&eB);
	Mailbox_Params_init(&mP);
	Mailbox_construct(&_queue_TxSubmit_Struct, sizeof(SMac_PacketInternal), SMAC_TXSUBMISSION_MAXDEPTH, &mP, &eB);
	mac->queue_TxSubmit = Mailbox_handle(&_queue_TxSubmit_Struct);

	mac->Frequency = freq;
	mac->TxPower = txpwr;
	Task_Params tP;
	Task_Params_init(&tP);
	tP.stackSize = SMAC_MAINTASK_STACKSIZE;
	tP.stack = _maintask_Stack;

	Task_construct(&_mainTask_Struct, (Task_FuncPtr)SMac_MainTaskFxn, &tP, NULL);

	// binsem_TxEmpty is used temporarily to indicate the RTOS task is ready for events.
	if (!Semaphore_pend(mac->binsem_TxEmpty, 2000000 / Clock_tickPeriod)) {  // Wait up to 2 seconds for main task to respond
		#ifdef SMAC_DEBUG
		System_printf("SMac_open: Timeout waiting for RTOS task to signal its successful initialization\n"); System_flush();
		#endif
		return false;
	}

	return true;
}

Bool SMac_close(Void)
{
	Event_post(mac->mainTaskEvents, SMAC_EVT_END);  // Signal main task to shut down RF and kill its task
	// Poll waiting for task to complete, return true if task ended or false if we timed out.
	int i = 0;
	while (Task_getMode(Task_handle(&_mainTask_Struct)) != Task_Mode_TERMINATED) {
		Task_sleep(10000 / Clock_tickPeriod);
		i++;
		if (i > 200) {  // Wait 2 seconds for it to terminate
			#ifdef SMAC_DEBUG
			System_printf("SMac_close: Timed out waiting for task (mode = %d)\n", (Int)Task_getMode(Task_handle(&_mainTask_Struct))); System_flush();
			#endif
			return false;
		}
	}
	return true;
}

Bool SMac_setRxAfterTx(UInt32 ms)
{
	if (!Semaphore_pend(mac->binsem_rxSchedUpdate, 50000 / Clock_tickPeriod)) {  // Wait a maximum of 50ms for this
		return false;
	}

	if (ms > 0) {
		mac->rxSched = SMAC_RXTYPE_AFTERTX;
	} else {
		mac->rxSched = SMAC_RXTYPE_DISABLED;
	}
	mac->rxSched_Millis = ms;
	Semaphore_post(mac->binsem_rxSchedUpdate);
	return true;
}

Bool SMac_enableRx(UInt32 maxMs)
{
	if (!Semaphore_pend(mac->binsem_rxSchedUpdate, 50000 / Clock_tickPeriod)) {  // Wait a maximum of 50ms for this
		#ifdef SMAC_DEBUG
		System_printf("SMac_enableRx: Timeout pending on binsem_rxSchedUpdate\n"); System_flush();
		#endif
		return false;
	}

	mac->rxSched = SMAC_RXTYPE_CONTINUAL;
	mac->rxSched_Millis = maxMs;
	Semaphore_post(mac->binsem_rxSchedUpdate);

	// Signal to RTOS task that RX mode needs to be entered immediately or after TxQueue is empty.
	Event_post(mac->mainTaskEvents, SMAC_EVT_UPDATERX);
	return true;
}

Bool SMac_disableRx(Void)
{
	if (!Semaphore_pend(mac->binsem_rxSchedUpdate, 50000 / Clock_tickPeriod)) {  // Wait a maximum of 50ms for this
		#ifdef SMAC_DEBUG
		System_printf("SMac_disableRx: Timeout pending on binsem_rxSchedUpdate\n"); System_flush();
		#endif
		return false;
	}

	if (mac->rxSched != SMAC_RXTYPE_AFTERTX) {
		mac->rxSched = SMAC_RXTYPE_DISABLED;
		mac->rxSched_Millis = 0;
	}
	Semaphore_post(mac->binsem_rxSchedUpdate);

	// Signal to RTOS task that RX mode needs to be halted immediately.
	Event_post(mac->mainTaskEvents, SMAC_EVT_UPDATERX);
	return true;
}

Bool SMac_submitTx(Semaphore_Handle binsem_Notifier, UInt32 dstAddr, UInt16 prID, UInt8 len, Void *data)
{
	SMac_PacketInternal req;

	req.binsem_NotifySent = binsem_Notifier;
	req.destAddr = dstAddr;
	req.programID = prID;
	req.length = len;
	req.data = data;
	#ifdef SMAC_DEBUG
	System_printf("SMac_submitTx: destAddr=%x, prID=%x, len=%d\n", dstAddr, prID, len); System_flush();
	#endif

	Bool ret = Mailbox_post(mac->queue_TxSubmit, &req, BIOS_WAIT_FOREVER);
	if (ret) {
		/* I originally handled this Event_post by setting readerEvent, readerEventId in the queue_TxSubmit mailbox so it
		 * would automatically post the event when the mailbox was posted.  However, this won't work on CC13xx devices using ROM
		 * or any non-ROM installs where Semaphore.supportsEvents = false (which is the case with ROM-based TI-RTOS installs)
		 * because the automatic event post uses the Semaphore event post feature inside one of the Mailbox module's internal
		 * semaphores to carry out the automatic event post..... which obviously won't work if Semaphore is set to not support
		 * events.
	     *
		 * So we have to post this event manually...
		 */
		Event_post(mac->mainTaskEvents, SMAC_EVT_TXSUBMITTED);
	}
	return ret;
}

Void SMac_requestTx()
{
	Event_post(mac->mainTaskEvents, SMAC_EVT_TXREQUESTED);
}

Bool SMac_pendTxQueue(UInt32 waitMs)
{
	return Semaphore_pend(mac->binsem_TxEmpty, (waitMs * 1000) / Clock_tickPeriod);
}

Bool SMac_isRxOverflow()
{
	return Semaphore_pend(mac->binsem_RxOvf, BIOS_NO_WAIT);
}

UInt SMac_getRxQueueFree()
{
	return SMac_util_GetRxQueueFreeCount();
}

UInt32 SMac_getIeeeAddress()
{
	// Ripped from EasyLink: IEEE address location burned into ROM
	//Primary IEEE address location - FCFG (Factory Config) MAC_15_4_0 register represents bits 31:0 of the 64-bit IEEE 802.15.4 address
	#define EASYLINK_PRIMARY_IEEE_ADDR_LOCATION   0x500012F0
	UInt32 *ieeeAddr = (UInt32 *)EASYLINK_PRIMARY_IEEE_ADDR_LOCATION;

	return *ieeeAddr;
}

Void SMac_setAdditionalRxAddress(UInt32 addr)
{
	mac->rxAddresses[1] = addr;
	Event_post(mac->mainTaskEvents, SMAC_EVT_SECADDRUPDATED);
}

UInt32 SMac_getAdditionalRxAddress()
{
	return mac->rxAddresses[1];
}

Task_Mode SMac_runState()
{
	return Task_getMode(Task_handle(&_mainTask_Struct));
}


/**
 * @brief RF commands for custom Carrier Sense-then-TX
 */
rfc_CMD_PROP_CS_t RF_cmdPropCarrierSense =
{
	.commandNo = CMD_PROP_CS,      // CMD_PROP_CS for CC13xx - see SWCU117D, the CC13xx/26xx TRM, page 1594 for detailed description
	.status = 0,              // Updated by RF CPU
	.pNextOp = 0,             // User updated to point to next command in chain
	.startTime = 0,
		/* Available triggerType's
		#define TRIG_NOW 0            ///< Triggers immediately
		#define TRIG_NEVER 1          ///< Never trigs
		#define TRIG_ABSTIME 2        ///< Trigs at an absolute time
		#define TRIG_REL_SUBMIT 3     ///< Trigs at a time relative to the command was submitted
		#define TRIG_REL_START 4      ///< Trigs at a time relative to the command started
		#define TRIG_REL_PREVSTART 5  ///< Trigs at a time relative to the previous command in the chain started
		#define TRIG_REL_FIRSTSTART 6 ///< Trigs at a time relative to the first command in the chain started
		#define TRIG_REL_PREVEND 7    ///< Trigs at a time relative to the previous command in the chain ended
		#define TRIG_REL_EVT1 8       ///< Trigs at a time relative to the context defined "Event 1"
		#define TRIG_REL_EVT2 9       ///< Trigs at a time relative to the context defined "Event 2"
		#define TRIG_EXTERNAL 10      ///< Trigs at an external event to the radio timer
		#define TRIG_PAST_BM 0x80     ///< Bitmask for setting pastTrig bit in order to trig immediately if
								      ///< trigger happened in the past
		*/
	.startTrigger.triggerType = TRIG_NOW,
	.startTrigger.bEnaCmd = 0,   // 0: No CMD_TRIGGER support, 1: Allow a CMD_TRIGGER to start this task
	.startTrigger.triggerNo = 0, // Which CMD_TRIGGER can trigger this
	.startTrigger.pastTrig = 1,  // 1: If radio time > trigger time, trigger immediately!
		/* Available condition rules:
		#define COND_ALWAYS 0         ///< Always run next command (except in case of Abort)
		#define COND_NEVER 1          ///< Never run next command
		#define COND_STOP_ON_FALSE 2  ///< Run next command if this command returned True, stop if it returned
									  ///< False
		#define COND_STOP_ON_TRUE 3   ///< Stop if this command returned True, run next command if it returned
									  ///< False
		#define COND_SKIP_ON_FALSE 4  ///< Run next command if this command returned True, skip a number of
									  ///< commands if it returned False
		#define COND_SKIP_ON_TRUE 5   ///< Skip a number of commands if this command returned True, run next
									  ///< command if it returned False
		 */
	.condition.rule = COND_SKIP_ON_FALSE, // Condition for running next command: How to proceed
	.condition.nSkip = 2,     // # of skips + 1 if rule involves skipping
	.csFsConf.bFsOffIdle = 0, // 0: Keep synth running if cmd ends with Channel Idle, 1: Turn off synth if Idle
	.csFsConf.bFsOffBusy = 0, // 0: Keep synth running if cmd ends with Channel Busy, 1: Turn off synth if Busy
	.csConf.bEnaRssi = 1,     // 1: Enable RSSI as criterion
	.csConf.bEnaCorr = 0,     // 1: Enable Correlation as criterion (presence of preamble bits)
	.csConf.operation = 0,    // 0: Busy if "RSSI OR Correlation" indicates Busy, 1: Busy if "RSSI AND Correlation" indicates Busy
	.csConf.busyOp = 0,       // 0: Continue carrier sense on Channel Busy, 1: End carrier sense if Busy
	.csConf.idleOp = 1,       // 0: Continue on Channel Idle, 1: End on Channel Idle (1 is preferable for CS-before-TX)
	.csConf.timeoutRes = 0,   // 0: Timeout with Channel State Invalid treated as Busy, 1: Timeout treated as Idle
	.rssiThr = 0,             // RSSI threshold
	.numRssiIdle = 0,         // # of consecutive RSSI measurements below threshold needed before channel is declared Idle
	.numRssiBusy = 0,         // # of consecutive RSSI measurements above threshold needed before channel is declared Busy
	.corrPeriod = 0,          // # of RAT ticks for a correlation observation period (currently set to symbol width times 8)
	.corrConfig.numCorrInv = 0,  // # of subsequent correlation tops with max <corrPeriod> RAT ticks between needed to go from Idle to Invalid
	.corrConfig.numCorrBusy = 0, // # of subsequent correlation tops with max <corrPeriod> RAT ticks between needed to go from Invalid to Busy
	.csEndTrigger.triggerType = TRIG_REL_START, // Type of End Trigger
	.csEndTrigger.bEnaCmd = 0,   // 0: No CMD_TRIGGER support, 1: Allow a CMD_TRIGGER to end CS
	.csEndTrigger.triggerNo = 0, // Which CMD_TRIGGER can end CS (w/ bEnaCmd=1)
	.csEndTrigger.pastTrig = 1,  // 0: If a past trigger is never triggered, give an error, 1: Trigger in the past triggers immediately
	.csEndTime = 0               // RAT tick time parameter for .csEndTrigger
};

rfc_CMD_COUNT_BRANCH_t RF_cmdCountBranch =
{
	.commandNo                = CMD_COUNT_BRANCH,
	.status                   = 0x0000,
	.pNextOp                  = 0, // Set this to (uint8_t*)&RF_cmdPropTx in the application
	.startTime                = 0x00000000,
	.startTrigger.triggerType = TRIG_NOW, // Triggers immediately
	.startTrigger.bEnaCmd     = 0x0,
	.startTrigger.triggerNo   = 0x0,
	.startTrigger.pastTrig    = 0x1,
	.condition.rule           = COND_STOP_ON_FALSE, // Run next command if this command returned TRUE, stop if it returned FALSE
													// End causes for the CMD_COUNT_BRANCH command:
													// Finished operation with counter = 0 when being started: DONE_OK         TRUE
													// Finished operation with counter > 0 after decrementing: DONE_OK         TRUE
													// Finished operation with counter = 0 after decrementing: DONE_COUNTDOWN  FALSE
	.condition.nSkip          = 0x0,
	.counter                  = 0, // On start, the radio CPU decrements the value, and the end status of the operation differs if the result is zero
								   // This number is set in the application (CS_RETRIES_WHEN_BUSY) and determines how many times the CMD_PROP_CS should run
								   // in the case where the channel i Busy
	.pNextOpIfOk              = 0, // Set this to (uint8_t*)&RF_cmdPropCs in the application};
};

// CMD_NOP
rfc_CMD_NOP_t RF_cmdNop =
{
    .commandNo                = CMD_NOP,
    .status                   = 0x0000,
    .pNextOp                  = 0, // Set this to (uint8_t*)&RF_cmdPropCs in the application
    .startTime                = 0x00000000,
    .startTrigger.triggerType = TRIG_ABSTIME, // Trigs immediately
    .startTrigger.bEnaCmd     = 0x0,
    .startTrigger.triggerNo   = 0x0,
    .startTrigger.pastTrig    = 0x1,
    .condition.rule           = COND_ALWAYS, // Always run next command (except in case of Abort)
    .condition.nSkip          = 0x0,
};


/**
 * @brief Main task for SMac engine
 */
volatile RF_EventMask lastTxEvent;
static Void SMac_MainTaskFxn(UArg arg0, UArg arg1)
{
	Semaphore_post(mac->binsem_TxEmpty);
	Semaphore_post(mac->binsem_rxSchedUpdate);

	// Check if CCFG_FORCE_VDDR_HH is set to 1 if the txpwr requested is > 13
#if (CCFG_FORCE_VDDR_HH != 0x1)
	if (mac->TxPower > 13) {
		BIOS_exit(1); // Invalid TX power; must compile with CCFG_FORCE_VDDR_HH=1 for 14dBm!
	}
#endif
	// Set TX power in CMD_PROP_RADIO_DIV_SETUP before RF_open is called
	UInt txPwrIdx;
	if (mac->TxPower < 0) {
		txPwrIdx = SMAC_TXPOWER_TABLE_SIZE - 1;
	} else {
		txPwrIdx = mac->TxPower;
	}
	RF_cmdPropRadioDivSetup.txPower = outputPower[txPwrIdx].txPower;

	RF_Params rfParams;
	RF_Params_init(&rfParams);
	mac->rfHandle = RF_open(&mac->rfObject, &RF_prop, (RF_RadioSetup *)&RF_cmdPropRadioDivSetup, &rfParams);
	if (mac->rfHandle == NULL) {
		// Should never reach here, but if so, abort!!
		#ifdef SMAC_DEBUG
		System_printf("SMac_MainTaskFxn: RF_open returned NULL, aborting\n"); System_flush();
		#endif
		return;
	}

	// Configure cmdFs frequency and program the synthesizer
	RF_cmdFs.frequency = (UInt16)(mac->Frequency / 1000000);
	RF_cmdFs.fractFreq = (UInt16)(((UInt64)mac->Frequency - (RF_cmdFs.frequency * 1000000)) * 65536 / 1000000);
	RF_runCmd(mac->rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);
	#ifdef SMAC_DEBUG
	System_printf("SMac_MainTaskFxn: Programming frequency synthesizer; freq=%d, fract=%d\n", RF_cmdFs.frequency, RF_cmdFs.fractFreq); System_flush();
	#endif

	// Right off the bat, put the RF core to sleep until we have something useful to do.
	//RF_yield(mac->rfHandle);
	mac->rfEnable = false;
	mac->rxCmdHandle = NULL;

	// Set up pointers and params for CarrierSense-before-TX
	RF_cmdNop.pNextOp = (rfc_radioOp_t *)&RF_cmdPropCarrierSense;
	RF_cmdPropCarrierSense.pNextOp = (rfc_radioOp_t *)&RF_cmdCountBranch;
	RF_cmdCountBranch.pNextOp = (rfc_radioOp_t *)&RF_cmdPropTx;
	RF_cmdCountBranch.pNextOpIfOk = (rfc_radioOp_t *)&RF_cmdPropCarrierSense;

	RF_cmdPropCarrierSense.rssiThr = -70;  // Signals above -70dBm indicate busy channel
	RF_cmdPropCarrierSense.numRssiIdle = 5;
	RF_cmdPropCarrierSense.numRssiBusy = 5;
	RF_cmdPropCarrierSense.csEndTime = (2000 + 150) * 4;  // ~2ms wait before TX
	// Configure settings for RF_cmdPropTx
	RF_cmdPropTx.pPkt = _txBuffer;
	RF_cmdPropTx.startTrigger.triggerType = TRIG_ABSTIME;
	RF_cmdPropTx.startTrigger.pastTrig = 1;
	RF_cmdPropTx.pktConf.bFsOff = 0;
	RF_cmdPropTx.pktConf.bUseCrc = 1;
	RF_cmdPropTx.pktConf.bVarLen = 1;
	RF_cmdPropTx.pNextOp = 0;

	#ifdef SMAC_DEBUG
	System_printf("SMac_MainTaskFxn: Pointer to state struct: %p\n", mac);
	System_printf("SMac_MainTaskFxn: Pointers to TX RF: RF_cmdNop=%p, RF_cmdPropCarrierSense=%p, RF_cmdCountBranch=%p, RF_cmdPropTx=%p\n", &RF_cmdNop, &RF_cmdPropCarrierSense, &RF_cmdCountBranch, &RF_cmdPropTx); System_flush();
	#endif

	// Set up config for RX
	SMac_util_ConfigureRxQueues();
	RF_cmdPropRx.startTrigger.triggerType = TRIG_ABSTIME;
	RF_cmdPropRx.startTrigger.pastTrig = 1;
	RF_cmdPropRx.pktConf.bRepeatNok = 1;  // Keep listening after CRC error or after successful read
	RF_cmdPropRx.pktConf.bRepeatOk = 1;
	RF_cmdPropRx.pktConf.bVarLen = 1;  // Variable-size frames
	RF_cmdPropRx.pktConf.bChkAddress = 1;  // Automatically check address byte
	RF_cmdPropRx.pktConf.endType = 0;      // If sync started, continue RX until complete regardless of end trigger
	RF_cmdPropRx.pktConf.filterOp = 0;
	RF_cmdPropRx.pktConf.bUseCrc = 1;
	RF_cmdPropRx.rxConf.bAutoFlushCrcErr = 1;
	RF_cmdPropRx.rxConf.bAutoFlushIgnored = 1;
	RF_cmdPropRx.rxConf.bIncludeHdr = 0;
	RF_cmdPropRx.rxConf.bIncludeCrc = 0;
	RF_cmdPropRx.rxConf.bAppendRssi = 1;
	RF_cmdPropRx.rxConf.bAppendStatus = 0;
	RF_cmdPropRx.maxPktLen = SMAC_MAXIMUM_FRAMESIZE;
	RF_cmdPropRx.pOutput = 0;
	RF_cmdPropRx.pQueue = &mac->rxQueue;


	// Signal that we're ready to roll
	Semaphore_post(mac->binsem_TxEmpty);

	while (1) {
		#ifdef SMAC_DEBUG
		System_printf("Polling events:\n"); System_flush();
		#endif
		UInt events = Event_pend(mac->mainTaskEvents, 0, SMAC_EVT_ALL, BIOS_WAIT_FOREVER);
		// Process one by one
		if (events & (UInt)SMAC_EVT_END) {
			if (mac->rfEnable) {
				RF_close(mac->rfHandle);
			}
			return;
		}

		if (events & (UInt)SMAC_EVT_SECADDRUPDATED) {
			if (mac->rfEnable && mac->rxCmdHandle) {
				// Halt RX and set SMAC_EVT_UPDATERX so our next events check will re-enable RX.
				RF_flushCmd(mac->rfHandle, mac->rxCmdHandle, 1);
				mac->rfEnable = false;
				mac->rxCmdHandle = NULL;
				events |= SMAC_EVT_UPDATERX;
			}

			if (mac->rxAddresses[1] == 0) {
				#ifdef SMAC_DEBUG
				System_printf("SMac_MainTaskFxn: Cleared secondary address\n"); System_flush();
				#endif
			} else {
				#ifdef SMAC_DEBUG
				System_printf("SMac_MainTaskFxn: Set secondary address: %x\n", mac->rxAddresses[1]); System_flush();
				#endif
			}
		}

		// Handle SMAC_EVT_RX before SMAC_EVT_UPDATERX in order to gracefully drain a full RX queue in the event of RX Queue Overflow
		if (events & (UInt)SMAC_EVT_RX) {
			// Process RX Queue entry byte-by-byte, executing program ID callbacks as discovered
			/* Frame format:
			 * [FrameLen 1B][DestAddr 4B][SrcAddr 4B][Program entry: [ProgramID 2B][PayloadLen 1B][Payload...]]{[ProgramID 2B][PayloadLen1B][Payload...]}..etc...
			 */
			rfc_dataEntryGeneral_t* currentDataEntry = RFQueue_getDataEntry();
			#ifdef SMAC_DEBUG
			UInt totalFrames = 0;
			#endif

			while (currentDataEntry->status == DATA_ENTRY_FINISHED) {
				#ifdef SMAC_DEBUG
				totalFrames++;
				#endif
				UInt32 dstAddr, srcAddr;
				UInt16 prID;
				Int16 fLen;
				UInt8 pLen, *frm = &currentDataEntry->data;
				if (currentDataEntry->config.lenSz == 1) {
					fLen = (Int16)frm[0];
					frm++;
				} else if (currentDataEntry->config.lenSz == 2) {
					fLen = (Int16)frm[0] | ((Int16)frm[1] << 8);
					frm += 2;
				} else {
					#ifdef SMAC_DEBUG
					System_printf("SMac_MainTaskFxn: SMAC_EVT_RX: Odd error: currentDataEntry->config.lenSz is %d\n", currentDataEntry->config.lenSz); System_flush();
					#endif
					RFQueue_nextEntry();  // Skip current entry and set pointer to next item
					currentDataEntry = RFQueue_getDataEntry();
					continue;  // Invalid or corrupted RX buffer entry?
				}
				fLen--;  // There is an RSSI byte at the very end
				if (fLen < 11) {
					#ifdef SMAC_DEBUG
					System_printf("SMac_MainTaskFxn: SMAC_EVT_RX: Packet received with fLen < 11 (fLen=%d)\n", fLen); System_flush();
					#endif
					RFQueue_nextEntry();  // Skip current entry and set pointer to next item
					currentDataEntry = RFQueue_getDataEntry();
					continue;  // Invalid frame
				}
				dstAddr = (UInt32)frm[0] | ((Uint32)frm[1] << 8) | ((Uint32)frm[2] << 16) | ((UInt32)frm[3] << 24);
				// Verify dstAddr is valid for this unit, since other transceivers with the same LSB in their IEEE address might trigger the address match.
				if (!SMac_util_ValidateOurAddress(dstAddr)) {
					#ifdef SMAC_DEBUG
					System_printf("SMac_MainTaskFxn: SMAC_EVT_RX: Packet received not destined to us (destAddr=%x)\n", dstAddr); System_flush();
					#endif
					RFQueue_nextEntry();  // Skip current entry and set pointer to next item
					currentDataEntry = RFQueue_getDataEntry();
					continue;  // Not destined for us; ignore
				}
				srcAddr = (UInt32)frm[4] | ((Uint32)frm[5] << 8) | ((Uint32)frm[6] << 16) | ((UInt32)frm[7] << 24);
				frm += 8;
				fLen -= 8;

				while (fLen > 2) {
					prID = (UInt16)frm[0] | ((UInt16)frm[1] << 8);
					pLen = frm[2];
					if (fLen < (3 + pLen)) {
						// Invalid; end of frame, bail out
						#ifdef SMAC_DEBUG
						System_printf("SMac_MainTaskFxn: SMAC_EVT_RX: Frame at idx=%d claims pLen=%d, only %d bytes left in frame\n", (UInt)(frm-&currentDataEntry->data), pLen, fLen); System_flush();
						#endif
						fLen = 0;
						continue;
					}
					frm += 3;
					SMac_RxCallback rxCb = SMac_util_getCallbackByProgramID(prID);
					if (rxCb != NULL) {
						#ifdef SMAC_DEBUG
						System_printf("SMac_MainTaskFxn: SMAC_EVT_RX: exec rxCb srcAddr=%x, prID=%x, pLen=%d\n", srcAddr, prID, pLen); System_flush();
						#endif
						rxCb(srcAddr, prID, pLen, frm);
					}
					if (mac->allProgramCallback != NULL) {
						#ifdef SMAC_DEBUG
						System_printf("SMac_MainTaskFxn: SMAC_EVT_RX: exec allProgramCallback srcAddr=%x, prID=%x, pLen=%d\n", srcAddr, prID, pLen); System_flush();
						#endif
						mac->allProgramCallback(srcAddr, prID, pLen, frm);
					}
					frm += pLen;
					fLen -= 3 + pLen;
				}
				// Finished processing packet
				RFQueue_nextEntry();  // Release current queue entry and set pointer to next item
				currentDataEntry = RFQueue_getDataEntry();
				#ifdef SMAC_DEBUG
				System_printf("SMac_MainTaskFxn: SMAC_EVT_RX: processed %d frames\n", totalFrames); System_flush();
				#endif
			}
		}

		if (events & (UInt)SMAC_EVT_UPDATERX) {
			if (Semaphore_pend(mac->binsem_rxSchedUpdate, 50000 / Clock_tickPeriod)) {  // Never wait more than 50ms for this...
				SMac_RxSchedule s = mac->rxSched;
				UInt32 rxms = mac->rxSched_Millis;
				Semaphore_post(mac->binsem_rxSchedUpdate);

				if (mac->rfEnable) {
					if (mac->rxCmdHandle) {
						if (s == SMAC_RXTYPE_DISABLED || s == SMAC_RXTYPE_AFTERTX) {
							// Disable RX
							RF_flushCmd(mac->rfHandle, mac->rxCmdHandle, 1);
							mac->rfEnable = false;
							mac->rxCmdHandle = NULL;
							//RF_yield(mac->rfHandle);
							#ifdef SMAC_DEBUG
							System_printf("SMac_MainTaskFxn: Disable RX, radio off\n"); System_flush();
							#endif
						}
					}
				} else {
					if (s == SMAC_RXTYPE_CONTINUAL) {
						// Activate RX mode
						// Configure RF_cmdPropRx with timeout
						SMac_util_PrepareRxCmd(rxms);
						mac->rfEnable = true;
						RF_cmdPropRx.startTime = RF_getCurrentTime();
						mac->rxCmdHandle = RF_postCmd(mac->rfHandle, (RF_Op*)&RF_cmdPropRx, RF_PriorityNormal, SMac_RxCallbackFxn, IRQ_RX_ENTRY_DONE | IRQ_COMMAND_DONE | IRQ_LAST_COMMAND_DONE);
						#ifdef SMAC_DEBUG
						System_printf("SMac_MainTaskFxn: Activate RX\n"); System_flush();
						#endif
					}
				}
			} else {
				#ifdef SMAC_DEBUG
				System_printf("SMac_MainTaskFxn: SMAC_EVT_UPDATERX: Timeout pending binsem_rxSchedUpdate\n"); System_flush();
				#endif
			}
		}

		if (events & (UInt)SMAC_EVT_RXTIMEDOUT) {
			if (Semaphore_pend(mac->binsem_rxSchedUpdate, 50000 / Clock_tickPeriod)) {  // Never wait more than 50ms for this...
				SMac_RxSchedule s = mac->rxSched;
				UInt32 rxms = mac->rxSched_Millis;

				if (s == SMAC_RXTYPE_CONTINUAL) {
					mac->rxSched = SMAC_RXTYPE_DISABLED;
					mac->rxSched_Millis = 0;
				}
				Semaphore_post(mac->binsem_rxSchedUpdate);
				// Use RF_yield to inform RF driver to shut off radio
				//RF_yield(mac->rfHandle);
				mac->rfEnable = false;
				mac->rxCmdHandle = NULL;
				#ifdef SMAC_DEBUG
				System_printf("SMac_MainTaskFxn: RX timeout - radio off\n"); System_flush();
				#endif
			} else {
				#ifdef SMAC_DEBUG
				System_printf("SMac_MainTaskFxn: SMAC_EVT_RXTIMEDOUT: Timeout pending binsem_rxSchedUpdate\n"); System_flush();
				#endif
			}
		}

		if (events & (UInt)SMAC_EVT_TXSUBMITTED) {
			// Process TX submissions into the txQueue
			Bool mbxRet;
			do {
				SMac_PacketInternal *txstore = SMac_util_getFreeTxSlot();
				if (txstore == NULL) {
					#ifdef SMAC_DEBUG
					System_printf("SMac_MainTaskFxn: SMAC_EVT_TXSUBMITTED: Ran out of TX queue slots\n"); System_flush();
					#endif
					break;  // Quit accepting mailbox entries, let them pend until TX is requested
				}
				mbxRet = Mailbox_pend(mac->queue_TxSubmit, txstore, BIOS_NO_WAIT);  // Retrieve mailbox packet entry and store immediately into TxQueue
			} while (mbxRet);  // So long as Mailbox_pend returns true, keep stuffing TX queue.
		}

		if (events & (UInt)SMAC_EVT_TXREQUESTED) {
			// Perform TX repeatedly until txQueue is empty
			// Loop through TxQueue collecting packets destined to the same DestAddr into one frame
			SMac_PacketInternal *pkt;
			Semaphore_Handle txNotifyList[SMAC_TXQUEUE_MAXDEPTH];
			UInt frIdx = 0, notifyCount = 0;
			Int frRem, i;

			if (pkt = SMac_util_getFirstTxEntry()) {
				UInt32 dstAddr = pkt->destAddr, srcAddr = mac->rxAddresses[0];
				_txBuffer[0] = (UInt8)dstAddr;
				_txBuffer[1] = (UInt8)(dstAddr >> 8);
				_txBuffer[2] = (UInt8)(dstAddr >> 16);
				_txBuffer[3] = (UInt8)(dstAddr >> 24);
				_txBuffer[4] = (UInt8)srcAddr;
				_txBuffer[5] = (UInt8)(srcAddr >> 8);
				_txBuffer[6] = (UInt8)(srcAddr >> 16);
				_txBuffer[7] = (UInt8)(srcAddr >> 24);
				frIdx += 8;

				do {
					frRem = SMac_util_StuffPacket(_txBuffer, frIdx, pkt);
					frIdx = SMAC_FRAMESIZE_ALLOCATION - frRem;
					if (frRem != -1) {
						// Add binary semaphore notifier to our list
						txNotifyList[notifyCount++] = pkt->binsem_NotifySent;
						// Clear pkt from TxQueue
						pkt->data = NULL;
					}
					pkt = SMac_util_getNextTxEntryForDestAddr(pkt->destAddr);
				} while (pkt && frRem > 0);

				#ifdef SMAC_DEBUG
				System_printf("SMac_MainTaskFxn: SMAC_EVT_TXREQUESTED: %d packets serviced in this transmit\n", notifyCount); System_flush();
				#endif

				if (frIdx > 8) {
					// Valid TX packet ready to roll!  Initiate TX mode.  Frame length is frIdx.
					// Step 1: Check if RX mode active, if so, halt RX command.
					if (mac->rfEnable) {
						if (mac->rxCmdHandle) {
							#ifdef SMAC_DEBUG
							System_printf("SMac_MainTaskFxn: SMAC_EVT_TXREQUESTED: Killing active RX mode in preparation for TX\n"); System_flush();
							#endif
							RF_flushCmd(mac->rfHandle, mac->rxCmdHandle, 1);
							mac->rxCmdHandle = NULL;
						}
					}
					// Step 2: Submit Carrier-Sense-then-TX command.
					mac->rfEnable = true;
					// Configure RF_cmdCsThenTx with payload, length, reload Carrier Sense
					RF_cmdPropTx.pktLen = frIdx;
					RF_cmdPropCarrierSense.status = 0;
					RF_cmdCountBranch.counter = mac->txCsWait;
					RF_cmdNop.startTime = RF_getCurrentTime();
					RF_cmdCountBranch.startTime = RF_cmdNop.startTime;
					RF_cmdPropTx.startTime = RF_cmdNop.startTime;
					#ifdef SMAC_DEBUG
					RF_cmdPropTx.status = 0xFEFE;
					#endif
					lastTxEvent = 0xFFFF;
					RF_EventMask e = RF_runCmd(mac->rfHandle, (RF_Op*)&RF_cmdNop, RF_PriorityNormal, NULL, 0);
					lastTxEvent = e;
					#ifdef SMAC_DEBUG
					System_printf("SMac_MainTaskFxn: SMAC_EVT_TXREQUESTED: Carrier Sense return status: %04x, CountBranch=%d, CountBranch status=%04x, Nop status=%04x\n", RF_cmdPropCarrierSense.status, RF_cmdCountBranch.counter, RF_cmdCountBranch.status, RF_cmdNop.status); System_flush();
					if (RF_cmdPropTx.status == 0xFEFE) {
						System_printf("SMac_MainTaskFxn: RF_cmdPropTx never ran!\n"); System_flush();
					}
					#endif

					// TODO: Somehow failed transfers should be signalled?  Also what to do if RF_cmdPropCarrierSense.status == PROP_DONE_BUSY?
					// Step 3: Post binary semaphore notifications to all TX packet notifiers
					for (i=0; i < notifyCount; i++) {
						if (txNotifyList[i] != NULL) {
							Semaphore_post(txNotifyList[i]);
						}
					}

					// Step 4: Should we retrigger this event if there are remaining TxQueue entries?
					if (SMac_util_getTxQueueDepth() > 0) {
						#ifdef SMAC_DEBUG
						System_printf("SMac_MainTaskFxn: SMAC_EVT_TXREQUESTED: More TX queue entries, retriggering\n"); System_flush();
						#endif
						Event_post(mac->mainTaskEvents, SMAC_EVT_TXREQUESTED);
					} else {
						// Step 4a: If we are using SMAC_RXTYPE_AFTERTX or SMAC_RXTYPE_CONTINUAL, initiate RX mode now.
						Semaphore_pend(mac->binsem_rxSchedUpdate, BIOS_WAIT_FOREVER);
						if (mac->rxSched == SMAC_RXTYPE_AFTERTX || mac->rxSched == SMAC_RXTYPE_CONTINUAL) {
							UInt32 ms = mac->rxSched_Millis;
							Semaphore_post(mac->binsem_rxSchedUpdate);

							// Configure RF_cmdPropRx with timeout
							#ifdef SMAC_DEBUG
							System_printf("SMac_MainTaskFxn: SMAC_EVT_TXREQUESTED: Enabling RX-after-TX with %d ms timeout\n", ms); System_flush();
							#endif
							SMac_util_PrepareRxCmd(ms);
							mac->rfEnable = true;
							RF_cmdPropRx.startTime = RF_getCurrentTime();
							mac->rxCmdHandle = RF_postCmd(mac->rfHandle, (RF_Op*)&RF_cmdPropRx, RF_PriorityNormal, SMac_RxCallbackFxn, IRQ_RX_ENTRY_DONE | IRQ_COMMAND_DONE | IRQ_LAST_COMMAND_DONE);
						} else {
							Semaphore_post(mac->binsem_rxSchedUpdate);
							// No RX after TX, so, shut down the radio.
							mac->rfEnable = false;
							//RF_yield(mac->rfHandle);
							#ifdef SMAC_DEBUG
							System_printf("SMac_MainTaskFxn: SMAC_EVT_TXREQUESTED: No RX-after-TX, radio off\n"); System_flush();
							#endif
						}
						Semaphore_post(mac->binsem_TxEmpty);  // TX queue empty, signal anyone waiting for it

					}
				} else {
					// Invalid situation - TxQueue must be corrupt, so, clear out the TxQueue.
					#ifdef SMAC_DEBUG
					System_printf("SMac_MainTaskFxn: SMAC_EVT_TXREQUESTED: Corrupt TXQueue: getFirstTxEntry returned NULL\n"); System_flush();
					#endif
					SMac_util_clearTxQueue();
					Semaphore_post(mac->binsem_TxEmpty);  // TX queue empty, signal anyone waiting for it
				}
			}
		}
	}
}

/**
 * @brief Callback function for RF driver used for RX events
 */
static Void SMac_RxCallbackFxn(RF_Handle h, RF_CmdHandle c, RF_EventMask e)
{
	#ifdef SMAC_DEBUG
	UInt32 e1 = (e >> 32), e2 = (UInt32)e;
	System_printf("SMac_RxCallbackFxn: EventMask = %08x%08x, RF_cmdPropRx status = %x\n", e1, e2, RF_cmdPropRx.status); System_flush();
	#endif
	if (e & (RF_EventRxEntryDone)) {
		#ifdef SMAC_DEBUG
		System_printf("SMac_RxCallbackFxn: Issuing SMAC_EVT_RX\n"); System_flush();
		#endif
		Event_post(mac->mainTaskEvents, SMAC_EVT_RX);
	}
	if (e & (RF_EventCmdDone | RF_EventLastCmdDone)) {
		if (RF_cmdPropRx.status == PROP_ERROR_RXBUF) {
			// RX buffer overflow; signal RX (to drain the RX queue buffer) and RXUPDATE
			#ifdef SMAC_DEBUG
			System_printf("SMac_RxCallbackFxn: Detected RXBUFOVF, issuing SMAC_EVT_RX | SMAC_EVT_UPDATERX\n"); System_flush();
			#endif
			mac->rfEnable = false;
			mac->rxCmdHandle = NULL;
			Event_post(mac->mainTaskEvents, SMAC_EVT_RX | SMAC_EVT_UPDATERX);
			Semaphore_post(mac->binsem_RxOvf);
		} else {
			#ifdef SMAC_DEBUG
			System_printf("SMac_RxCallbackFxn: Issuing SMAC_EVT_RXTIMEDOUT\n"); System_flush();
			#endif
			Event_post(mac->mainTaskEvents, SMAC_EVT_RXTIMEDOUT);
		}
	}
}

//! @brief Helper func to search txQueue to get a count of active entries
static inline UInt SMac_util_getTxQueueDepth()
{
	int i;
	UInt total = 0;

	for (i=0; i < SMAC_TXQUEUE_MAXDEPTH; i++) {
		if (mac->txQueue[i].data != NULL) {
			total++;
		}
	}
	#ifdef SMAC_DEBUG
	System_printf("SMac_util_getTxQueueDepth: Found %d non-NULL entries\n", total); System_flush();
	#endif
	return total;
}

//! @brief Helper utility for purging the TX queue
static Void SMac_util_clearTxQueue()
{
	int i;

	for (i=0; i < SMAC_TXQUEUE_MAXDEPTH; i++) {
		mac->txQueue[i].data = NULL;
	}
}

//! @brief Helper utility: Find a free TX slot or return NULL if none available.
static inline SMac_PacketInternal * SMac_util_getFreeTxSlot()
{
	int i;

	for (i=0; i < SMAC_TXQUEUE_MAXDEPTH; i++) {
		if (mac->txQueue[i].data == NULL) {
			return &mac->txQueue[i];
		}
	}
	return NULL;
}

//! @brief Helper utility: Locate the first valid TX entry in the TxQueue.
static inline SMac_PacketInternal * SMac_util_getFirstTxEntry()
{
	int i;

	for (i=0; i < SMAC_TXQUEUE_MAXDEPTH; i++) {
		if (mac->txQueue[i].data != NULL) {
			return &mac->txQueue[i];
		}
	}
	return NULL;
}

//! @brief Helper function: Find the next TX entry in the TxQueue destined for the same address
static inline SMac_PacketInternal * SMac_util_getNextTxEntryForDestAddr(UInt32 dstAddr)
{
	int i;

	for (i=0; i < SMAC_TXQUEUE_MAXDEPTH; i++) {
		if (mac->txQueue[i].destAddr == dstAddr && mac->txQueue[i].data != NULL) {
			return &mac->txQueue[i];
		}
	}
	return NULL;
}

//! @brief Helper function: Add specified packet to buffer, returning remaining available frame capacity or -1 if won't fit.
static inline Int SMac_util_StuffPacket(Void *frameBuf, UInt startPos, SMac_PacketInternal *pkt)
{
	UInt8 *cStart = ((UInt8 *)frameBuf) + startPos;
	UInt8 pLen = pkt->length;

	if ( (SMAC_MAXIMUM_FRAMESIZE-startPos) < (3+pLen) ) {
		return -1;
	}

	cStart[0] = pkt->programID & 0xFF;
	cStart[1] = pkt->programID >> 8;
	cStart[2] = pLen;
	memcpy(cStart+3, pkt->data, pLen);

	startPos += 3 + pLen;
	return SMAC_MAXIMUM_FRAMESIZE - startPos;
}

static inline SMac_RxCallback SMac_util_getCallbackByProgramID(UInt16 prID)
{
	int i;

	for (i=0; i < SMAC_MAXIMUM_PROGRAMID_REGISTRATIONS; i++) {
		if (mac->programCallbacks[i].programID == prID) {
			return mac->programCallbacks[i].callback;
		}
	}
	return (SMac_RxCallback)NULL;
}

static inline Bool SMac_util_ValidateOurAddress(UInt32 dstAddr)
{
	if (dstAddr == mac->rxAddresses[0]) {
		return true;
	}
	if (mac->rxAddresses[1] != 0) {
		if (dstAddr == mac->rxAddresses[1]) {
			return true;
		}
	}
	return false;
}

static Void SMac_util_PrepareRxCmd(UInt32 timeoutMillis)
{
	// Step 1: Configure address matching
	RF_cmdPropRx.address0 = (UInt8)mac->rxAddresses[0];
	#ifdef SMAC_DEBUG
	System_printf("SMac_util_PrepareRxCmd: address0=%02x for IEEEAddr=%08x\n", RF_cmdPropRx.address0, mac->rxAddresses[0]); System_flush();
	#endif
	if (mac->rxAddresses[1] == 0) {
		RF_cmdPropRx.address1 = (UInt8)mac->rxAddresses[0];
	} else {
		RF_cmdPropRx.address1 = (UInt8)mac->rxAddresses[1];
		#ifdef SMAC_DEBUG
		System_printf("SMac_util_PrepareRxCmd: address1=%02x for AltAddr=%08x\n", RF_cmdPropRx.address1, mac->rxAddresses[1]); System_flush();
		#endif
	}

	// Step 2: Configure endTime and trigger
	if (timeoutMillis != 0) {
		RF_cmdPropRx.endTrigger.triggerType = TRIG_REL_START;
		RF_cmdPropRx.endTime = (4000000/1000) * timeoutMillis;  // RAT = 4MHz timer
	} else {
		RF_cmdPropRx.endTrigger.triggerType = TRIG_NEVER;
		RF_cmdPropRx.endTime = 0;
	}
}

static Void SMac_util_ConfigureRxQueues()
{
	Int i;

	if (mac->RxQueueBuffer == NULL) {
		// SMac_init() probably wasn't run!
		#ifdef SMAC_DEBUG
		System_printf("SMac_MainTaskFxn: mac->RxQueueBuffer is NULL, most likely SMac_init() wasn't run before SMac_open?\n"); System_flush();
		#endif
		return;
	}
	if ( RFQueue_defineQueue(&mac->rxQueue, mac->RxQueueBuffer, RXQUEUEBUFSIZE, SMAC_RXQUEUE_MAXDEPTH, SMAC_FRAMESIZE_ALLOCATION) ) {
		// error!
		#ifdef SMAC_DEBUG
		System_printf("SMac_MainTaskFxn: RFQueue_defineQueue failed!\n"); System_flush();
		#endif
		return;
	}

	rfc_dataEntryGeneral_t *cp = (rfc_dataEntryGeneral_t *)mac->rxQueue.pCurrEntry;
	for (i=0; i < SMAC_RXQUEUE_MAXDEPTH; i++) {
		cp->config.lenSz = 1;  // Support providing a 1-byte frame length indicator
		cp = (rfc_dataEntryGeneral_t *)cp->pNextEntry;
	}
}

static inline UInt SMac_util_GetRxQueueFreeCount()
{
	rfc_dataEntryGeneral_t *cp = (rfc_dataEntryGeneral_t *)mac->rxQueue.pCurrEntry;
	Int i;
	UInt count = 0;

	for (i=0; i < SMAC_RXQUEUE_MAXDEPTH; i++) {
		if (cp->status == DATA_ENTRY_PENDING) {
			count++;
			cp = (rfc_dataEntryGeneral_t *)cp->pNextEntry;
		} else {
			return count;
		}
	}
	return count;
}
