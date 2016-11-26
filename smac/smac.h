/**
 * @brief SMac API definitions
 *
 */
#ifndef SMAC_SMAC_H_
#define SMAC_SMAC_H_

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/drivers/rf/RF.h>
#include <driverlib/rf_prop_mailbox.h>
#include <smartrf_settings/smartrf_settings.h>
#include "RFQueue.h"

// Defining this symbol enables a whole slew of System_printf's in the stack code, some of which may affect performance.
//#define SMAC_DEBUG 1

/**
 * @brief SMac config definitions - many can be overridden in project predefined symbol properties
 */
#ifndef SMAC_TXQUEUE_MAXDEPTH
#define SMAC_TXQUEUE_MAXDEPTH 8
#endif

#ifndef SMAC_TXSUBMISSION_MAXDEPTH
#define SMAC_TXSUBMISSION_MAXDEPTH 8
#endif

#ifndef SMAC_RXQUEUE_MAXDEPTH
#define SMAC_RXQUEUE_MAXDEPTH 8
#endif

#ifndef SMAC_MAXIMUM_FRAMESIZE
#define SMAC_MAXIMUM_FRAMESIZE 240
#endif

#ifndef SMAC_MAXIMUM_PROGRAMID_REGISTRATIONS
#define SMAC_MAXIMUM_PROGRAMID_REGISTRATIONS 8
#endif

#ifndef SMAC_MAINTASK_STACKSIZE
#define SMAC_MAINTASK_STACKSIZE 1024
#endif

#define SMAC_FRAMESIZE_ALLOCATION (SMAC_MAXIMUM_FRAMESIZE+2) //! @brief Needs to be >= SMAC_MAXIMUM_FRAMESIZE, includes pktlen and RSSI
static UInt32 SMac_Basestation = 0xDEADBEEF;
typedef void(*SMac_RxCallback)(UInt32 srcAddr, UInt16 programID, UInt8 payloadLen, Void *payload);  // Packet callback function type

/**
 *  @brief Internal TX queue format
 *  @details When packets are submitted to the SMac RTOS task, the data pointer will point back into
 *           a buffer owned by the client task, and this buffer should remain untouched until binsem_NotifySent
 *           has been posted.
 */
typedef struct {
	Semaphore_Handle binsem_NotifySent;
	UInt32 destAddr;
	UInt16 programID;
	UInt8 length;
	Void *data;
} SMac_PacketInternal;

/**
 * @brief Manner of RX modes, i.e. whether to turn on RX after TX and with what timeout...
 */
typedef enum {
	SMAC_RXTYPE_DISABLED = 0,
	SMAC_RXTYPE_CONTINUAL = 1,
	SMAC_RXTYPE_AFTERTX = 2
} SMac_RxSchedule;

typedef enum {
	SMAC_EVT_RX = 1 << 0,
	SMAC_EVT_UPDATERX = 1 << 1,
	SMAC_EVT_TXSUBMITTED = 1 << 2,
	SMAC_EVT_RXTIMEDOUT = 1 << 3,
	SMAC_EVT_TXREQUESTED = 1 << 4,
	SMAC_EVT_SECADDRUPDATED = 1 << 5,
	SMAC_EVT_CFGUPDATED = 1 << 6,
	SMAC_EVT_END = 1 << 7
} SMac_MainTaskEvent;
#define SMAC_EVT_ALL ( (UInt)SMAC_EVT_RX | (UInt)SMAC_EVT_UPDATERX | (UInt)SMAC_EVT_TXSUBMITTED \
		                  | (UInt)SMAC_EVT_RXTIMEDOUT | (UInt)SMAC_EVT_TXREQUESTED | (UInt)SMAC_EVT_SECADDRUPDATED \
						  | (UInt)SMAC_EVT_CFGUPDATED | (UInt)SMAC_EVT_END )

typedef struct {
	UInt16 programID;
	SMac_RxCallback callback;
} SMac_RxRegistration;

typedef UInt32 SMac_RxAddress;  // 4-byte addresses with LSB most unique; OTA format sends Little-Endian (LSB first).

typedef struct {
	UInt32 Frequency;
	Int8 TxPower;
	Semaphore_Handle binsem_TxEmpty;
	Semaphore_Handle binsem_RxOvf;
	Event_Handle mainTaskEvents;
	Bool rfEnable;  //! @brief This will switch on & off particularly with TX-only nodes, and denotes whether RF_close/RF_open needs to run.
	Mailbox_Handle queue_TxSubmit;
	SMac_PacketInternal txQueue[SMAC_TXQUEUE_MAXDEPTH];
	dataQueue_t rxQueue;
	UInt8 * RxQueueBuffer;  //! @brief Must be assigned to a 32-bit aligned buffer
	Semaphore_Handle binsem_rxSchedUpdate;  // Semaphore locking access to rxSched, rxSched_postTxMillis
	volatile SMac_RxSchedule rxSched;
	volatile UInt32 rxSched_Millis;  // How long to activate RX mode for
	Semaphore_Handle binsem_regUpdate;  // Semaphore locking access to callback registries
	volatile SMac_RxRegistration programCallbacks[SMAC_MAXIMUM_PROGRAMID_REGISTRATIONS];
	volatile SMac_RxCallback allProgramCallback;
	UInt16 txCsWait;  // COUNT_BRANCH counter in 2ms increments

	RF_Object rfObject;
	RF_Handle rfHandle;
	RF_CmdHandle rxCmdHandle;
	SMac_RxAddress rxAddresses[2];  // Up to 2 available - the IEEE address and one other
} SMac_Struct;


/**
 * @brief SMac API
 */
Bool SMac_registerRx(UInt16, SMac_RxCallback); //! @brief Define callback function for a particular programID.  Runs from SMac Task context.
Bool SMac_deregisterRx(UInt16);
Bool SMac_registerAllRx(SMac_RxCallback);  //! @brief The default "run this for EVERY packet" callback.
Bool SMac_deregisterAllRx(Void);
Void SMac_init(Void);
Bool SMac_open(UInt32, Int8);  //! @brief Start the SMac tasks with RF speed, TXpower specified
Bool SMac_close(Void);  //! @brief Stop the SMac tasks, shut down RF completely
Task_Mode SMac_runState();  //! @brief Return the TI-RTOS Task_State of the main task

Bool SMac_setRxAfterTx(UInt32); //! @brief Request that RX mode automatically runs for <arg> milliseconds after draining TX queue, or 0 disables this feature
Bool SMac_enableRx(UInt32);  //! @brief Begin RX mode for up to <arg> milliseconds, or indefinite if 0.  This overrides setRxAfterTx.
Bool SMac_disableRx(Void);   //! @brief End RX mode immediately.  It can short-end setRxAfterTx receive mode but won't clear its setting.
Bool SMac_submitTx(Semaphore_Handle, UInt32, UInt16, UInt8, Void *); //! @brief Submit a new packet for eventual transmission
Void SMac_requestTx();  //! @brief Signal SMac stack to begin transmitting all contents of TxQueue
Bool SMac_pendTxQueue(UInt32);  //! @brief Allow a task to pend up to <arg> timeout until TX queues are empty
Bool SMac_isRxOverflow();  //! @brief Check a semaphore indicating if RX overflow has occurred recently.  Semaphore is cleared when this runs.
UInt SMac_getRxQueueFree();  //! @brief Check how full the RX queue is at this moment.

UInt32 SMac_getIeeeAddress();  //! @brief Obtain the IEEE 802.15.4 address used to derive our SrcAddr
Void SMac_setAdditionalRxAddress(UInt32 addr);  //! @brief Set a secondary RX address, or unset it if 0
UInt32 SMac_getAdditionalRxAddress();  //! @brief Return the value of our "secondary" RX address or 0 if it's not set.

Void SMac_setFrequency(UInt32);  //! @brief Change radio center frequency; restarts RX if active
Void SMac_setTxPower(Int8);      //! @brief Change radio TX power; restarts RX if active


/**
 * @brief Common program IDs
 */
#define SMAC_PROG_NONE 0x0000
#define SMAC_PROG_DEBUG 0xFFFE

#endif /* SMAC_SMAC_H_ */
