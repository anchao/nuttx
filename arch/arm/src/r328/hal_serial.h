#ifndef HAL_SERIAL_H
#define HAL_SERIAL_H

#ifdef __cplusplus
extern "C"
{
#endif

// version combine.
#define SUNXI_HAL_VERSION_MAJOR_MINOR(major, minor)     (((major) << 8) | (minor))

typedef struct sunxi_hal_version
{
    // API version NO.
    uint16_t api;

    // Driver version NO.
    uint16_t drv;
} sunxi_hal_version_t;

// General return code of hal driver.
#define SUNXI_HAL_OK                     0UL
// Unspecified error.
#define SUNXI_HAL_ERROR                 -1UL
// Hal is busy.
#define SUNXI_HAL_ERROR_BUSY            -2UL
// Timout occured.
#define SUNXI_HAL_ERROR_TIMEOUT         -3UL
// Operaion not supported.
#define SUNXI_HAL_ERROR_UNSUPOT         -4UL
// Parameter error.
#define SUNXI_HAL_ERROR_PARAERR         -5UL
// Start of driver specific errors.
#define SUNXI_HAL_ERROR_DRVSPECIFIC     -6UL

/* This enum defines baud rate of the UART frame. */
typedef enum
{
    UART_BAUDRATE_300 = 0,
    UART_BAUDRATE_600,
    UART_BAUDRATE_1200,
    UART_BAUDRATE_2400,
    UART_BAUDRATE_4800,
    UART_BAUDRATE_9600,
    UART_BAUDRATE_19200,
    UART_BAUDRATE_38400,
    UART_BAUDRATE_57600,
    UART_BAUDRATE_115200,
    UART_BAUDRATE_230400,
    UART_BAUDRATE_576000,
    UART_BAUDRATE_921600,
    UART_BAUDRATE_1000000,
    UART_BAUDRATE_1500000,
    UART_BAUDRATE_3000000,
    UART_BAUDRATE_4000000,
    UART_BAUDRATE_MAX,
} uart_baudrate_t;

/* This enum defines word length of the UART frame. */
typedef enum
{
    UART_WORD_LENGTH_5 = 0,
    UART_WORD_LENGTH_6,
    UART_WORD_LENGTH_7,
    UART_WORD_LENGTH_8,
} uart_word_length_t;

/* This enum defines stop bit of the UART frame. */
typedef enum
{
    UART_STOP_BIT_1 = 0,
    UART_STOP_BIT_2,
} uart_stop_bit_t;

/*
 * UART data width
 */
typedef enum {
    DATA_WIDTH_5BIT,
    DATA_WIDTH_6BIT,
    DATA_WIDTH_7BIT,
    DATA_WIDTH_8BIT,
    DATA_WIDTH_9BIT
} hal_uart_data_width_t;

/*
 * UART mode
 */
typedef enum {
    MODE_TX,
    MODE_RX,
    MODE_TX_RX
} hal_uart_mode_t;

/* This enum defines parity of the UART frame. */
typedef enum
{
    UART_PARITY_NONE = 0,
    UART_PARITY_ODD,
    UART_PARITY_EVEN
} uart_parity_t;

/*
 * UART flow control
 */
typedef enum {
    FLOW_CONTROL_DISABLED,
    FLOW_CONTROL_CTS,
    FLOW_CONTROL_RTS,
    FLOW_CONTROL_CTS_RTS
} hal_uart_flow_control_t;

/* This struct defines UART configure parameters. */
typedef struct
{
    uart_baudrate_t baudrate;
    uart_word_length_t word_length;
    uart_stop_bit_t stop_bit;
    uart_parity_t parity;
} _uart_config_t;



// brief General power states
typedef enum sunxi_hal_power_state
{
    ///< Power off: no operation possible
    SUSNXI_HAL_POWER_OFF,
    ///< Low Power mode: retain state, detect and signal wake-up events
    SUSNXI_HAL_POWER_LOW,
    ///< Power on: full operation at maximum performance
    SUSNXI_HAL_POWER_FULL
} sunxi_hal_power_state_e;

/*
 * This enum defines return status of the UART HAL public API.
 * User should check return value after calling these APIs.
 */
typedef enum
{
    HAL_UART_STATUS_ERROR_PARAMETER = -4,      /**< Invalid user input parameter. */
    HAL_UART_STATUS_ERROR_BUSY = -3,           /**< UART port is currently in use. */
    HAL_UART_STATUS_ERROR_UNINITIALIZED = -2,  /**< UART port has not been initialized. */
    HAL_UART_STATUS_ERROR = -1,                /**< UART driver detected a common error. */
    HAL_UART_STATUS_OK = 0                     /**< UART function executed successfully. */
} hal_uart_status_t;

typedef enum
{
    UART_0 = 0,
    UART_1,
    UART_2,
    UART_3,
    UART_MAX,
} uart_port_t;

//=================================reg===================================================//
/*
 * brief USART Status
 */
typedef struct sunxi_hal_usart_status
{
    uint32_t tx_busy          : 1;        ///< Transmitter busy flag
    uint32_t rx_busy          : 1;        ///< Receiver busy flag
    uint32_t tx_underflow     : 1;        ///< Transmit data underflow detected (cleared on start of next send operation)
    uint32_t rx_overflow      : 1;        ///< Receive data overflow detected (cleared on start of next receive operation)
    uint32_t rx_break         : 1;        ///< Break detected on receive (cleared on start of next receive operation)
    uint32_t rx_framing_error : 1;        ///< Framing error detected on receive (cleared on start of next receive operation)
    uint32_t rx_parity_error  : 1;        ///< Parity error detected on receive (cleared on start of next receive operation)
    uint32_t reserved         : 25;
} sunxi_hal_usart_status_t;

/*
 *brief USART Modem Control
 */
typedef enum sunxi_hal_usart_modem_control
{
    SUNXI_HAL_USART_RTS_CLEAR,            ///< Deactivate RTS
    SUNXI_HAL_USART_RTS_SET,              ///< Activate RTS
    SUNXI_HAL_USART_DTR_CLEAR,            ///< Deactivate DTR
    SUNXI_HAL_USART_DTR_SET               ///< Activate DTR
} sunxi_hal_usart_modem_control_e;

typedef enum sunxi_uart_control
{
	SUNXI_SET_BAUD,
	SUNXI_SET_FLOWCTL,
	SUNXI_SET_FORMAT,
} sunxi_uart_control_t;

/*
 *brief USART Modem Status
 */
typedef struct sunxi_hal_usart_modem_status
{
    uint32_t cts      : 1;                ///< CTS state: 1=Active, 0=Inactive
    uint32_t dsr      : 1;                ///< DSR state: 1=Active, 0=Inactive
    uint32_t dcd      : 1;                ///< DCD state: 1=Active, 0=Inactive
    uint32_t ri       : 1;                ///< RI  state: 1=Active, 0=Inactive
    uint32_t reserved : 28;
} sunxi_hal_usart_modem_status_t;

/* This enum defines the UART event when an interrupt occurs. */
typedef enum {
	UART_EVENT_TRANSACTION_ERROR = -1,
	UART_EVENT_RX_BUFFER_ERROR = -2,
	UART_EVENT_TX_COMPLETE = 1,
	UART_EVENT_RX_COMPLETE = 2,
} uart_callback_event_t;

/** @brief This typedef defines user's callback function prototype.
 *             This callback function will be called in UART interrupt handler when UART interrupt is raised.
 *             User should call uart_register_callback() to register callbacks to UART driver explicitly.
 *             Note, that the callback function is not appropriate for time-consuming operations. \n
 *             parameter "event" : for more information, please refer to description of #uart_callback_event_t.
 *             parameter "user_data" : a user defined data used in the callback function.
 */
typedef void (*uart_callback_t)(uart_callback_event_t event, void *user_data);

typedef struct
{
    uint8_t *buf;
    uint32_t len;
    uint32_t head;
    uint32_t tail;
    int32_t cnt;
} uart_ring_buf_t;

/* This struct defines UART private data */
typedef struct
{
    /* basic info */
    uart_port_t uart_port;
    uint32_t irqn;

    /* uart register value */
    unsigned char ier;
    unsigned char lcr;
    unsigned char mcr;
    unsigned char fcr;
    unsigned char dll;
    unsigned char dlh;

    /* tx & rx buf */
    const char *tx_buf;
    uint32_t tx_buf_size;
    /* rx ring buf */
    uart_ring_buf_t ring_buf;

    /* user callback */
    uart_callback_t func;

    void *arg;
} uart_priv_t;
/**
  \fn          sunxi_hal_version_t SUNXI_HAL_USART_GetVersion (void)
  \brief       Get driver version.
  \return      \ref sunxi_hal_version_t

  \fn          SUNXI_HAL_USART_CAPABILITIES SUNXI_HAL_USART_GetCapabilities (void)
  \brief       Get driver capabilities
  \return      \ref SUNXI_HAL_USART_CAPABILITIES

  \fn          int32_t SUNXI_HAL_USART_Initialize (SUNXI_HAL_USART_SignalEvent_t cb_event)
  \brief       Initialize USART Interface.
  \param[in]   cb_event  Pointer to \ref SUNXI_HAL_USART_SignalEvent
  \return      \ref execution_status

  \fn          int32_t SUNXI_HAL_USART_Uninitialize (void)
  \brief       De-initialize USART Interface.
  \return      \ref execution_status

  \fn          int32_t SUNXI_HAL_USART_PowerControl (SUNXI_HAL_POWER_STATE state)
  \brief       Control USART Interface Power.
  \param[in]   state  Power state
  \return      \ref execution_status

  \fn          int32_t SUNXI_HAL_USART_Send (const void *data, uint32_t num)
  \brief       Start sending data to USART transmitter.
  \param[in]   data  Pointer to buffer with data to send to USART transmitter
  \param[in]   num   Number of data items to send
  \return      \ref execution_status

  \fn          int32_t SUNXI_HAL_USART_Receive (void *data, uint32_t num)
  \brief       Start receiving data from USART receiver.
  \param[out]  data  Pointer to buffer for data to receive from USART receiver
  \param[in]   num   Number of data items to receive
  \return      \ref execution_status

  \fn          int32_t SUNXI_HAL_USART_Transfer (const void *data_out,
                                                 void *data_in,
                                           uint32_t    num)
  \brief       Start sending/receiving data to/from USART transmitter/receiver.
  \param[in]   data_out  Pointer to buffer with data to send to USART transmitter
  \param[out]  data_in   Pointer to buffer for data to receive from USART receiver
  \param[in]   num       Number of data items to transfer
  \return      \ref execution_status

  \fn          uint32_t SUNXI_HAL_USART_GetTxCount (void)
  \brief       Get transmitted data count.
  \return      number of data items transmitted

  \fn          uint32_t SUNXI_HAL_USART_GetRxCount (void)
  \brief       Get received data count.
  \return      number of data items received

  \fn          int32_t SUNXI_HAL_USART_Control (uint32_t control, uint32_t arg)
  \brief       Control USART Interface.
  \param[in]   control  Operation
  \param[in]   arg      Argument of operation (optional)
  \return      common \ref execution_status and driver specific \ref usart_execution_status

  \fn          SUNXI_HAL_USART_STATUS SUNXI_HAL_USART_GetStatus (void)
  \brief       Get USART status.
  \return      USART status \ref SUNXI_HAL_USART_STATUS

  \fn          int32_t SUNXI_HAL_USART_SetModemControl (SUNXI_HAL_USART_MODEM_CONTROL control)
  \brief       Set USART Modem Control line state.
  \param[in]   control  \ref SUNXI_HAL_USART_MODEM_CONTROL
  \return      \ref execution_status

  \fn          SUNXI_HAL_USART_MODEM_STATUS SUNXI_HAL_USART_GetModemStatus (void)
  \brief       Get USART Modem Status lines state.
  \return      modem status \ref SUNXI_HAL_USART_MODEM_STATUS

  \fn          void SUNXI_HAL_USART_SignalEvent (uint32_t event)
  \brief       Signal USART Events.
  \param[in]   event  \ref USART_events notification mask
  \return      none
*/

typedef void (*sunxi_hal_usart_signal_event_t)(uint32_t event);   ///< Pointer to \ref SUNXI_HAL_USART_SignalEvent : Signal USART Event.


/**
\brief USART Device Driver Capabilities.
*/
typedef struct sunxi_hal_usart_capabilities
{
    uint32_t asynchronous       : 1;      ///< supports UART (Asynchronous) mode
    uint32_t synchronous_master : 1;      ///< supports Synchronous Master mode
    uint32_t synchronous_slave  : 1;      ///< supports Synchronous Slave mode
    uint32_t single_wire        : 1;      ///< supports UART Single-wire mode
    uint32_t irda               : 1;      ///< supports UART IrDA mode
    uint32_t smart_card         : 1;      ///< supports UART Smart Card mode
    uint32_t smart_card_clock   : 1;      ///< Smart Card Clock generator available
    uint32_t flow_control_rts   : 1;      ///< RTS Flow Control available
    uint32_t flow_control_cts   : 1;      ///< CTS Flow Control available
    uint32_t event_tx_complete  : 1;      ///< Transmit completed event: \ref SUNXI_HAL_USART_EVENT_TX_COMPLETE
    uint32_t event_rx_timeout   : 1;      ///< Signal receive character timeout event: \ref SUNXI_HAL_USART_EVENT_RX_TIMEOUT
    uint32_t rts                : 1;      ///< RTS Line: 0=not available, 1=available
    uint32_t cts                : 1;      ///< CTS Line: 0=not available, 1=available
    uint32_t dtr                : 1;      ///< DTR Line: 0=not available, 1=available
    uint32_t dsr                : 1;      ///< DSR Line: 0=not available, 1=available
    uint32_t dcd                : 1;      ///< DCD Line: 0=not available, 1=available
    uint32_t ri                 : 1;      ///< RI Line: 0=not available, 1=available
    uint32_t event_cts          : 1;      ///< Signal CTS change event: \ref SUNXI_HAL_USART_EVENT_CTS
    uint32_t event_dsr          : 1;      ///< Signal DSR change event: \ref SUNXI_HAL_USART_EVENT_DSR
    uint32_t event_dcd          : 1;      ///< Signal DCD change event: \ref SUNXI_HAL_USART_EVENT_DCD
    uint32_t event_ri           : 1;      ///< Signal RI change event: \ref SUNXI_HAL_USART_EVENT_RI
    uint32_t reserved           : 11;     ///< Reserved (must be zero)
} sunxi_hal_usart_capabilities_t;


/*
 *brief Access structure of the USART Driver.
 */
typedef struct sunxi_hal_driver_usart
{

    ///< Pointer to \ref SUNXI_HAL_USART_GetVersion : Get driver version.
    sunxi_hal_version_t (*get_version)(int32_t dev);

    ///< Pointer to \ref SUNXI_HAL_USART_GetCapabilities : Get driver capabilities.
    sunxi_hal_usart_capabilities_t (*get_capabilities)(int32_t dev);

    ///< Pointer to \ref SUNXI_HAL_USART_Initialize : Initialize USART Interface.
    int32_t (*initialize)(int32_t uart_port);

    ///< Pointer to \ref SUNXI_HAL_USART_Uninitialize : De-initialize USART Interface.
    int32_t (*uninitialize)(int32_t dev);

    ///< Pointer to \ref SUNXI_HAL_USART_PowerControl : Control USART Interface Power.
    int32_t (*power_control)(int32_t dev, sunxi_hal_power_state_e state);

    ///< Pointer to \ref SUNXI_HAL_USART_Send : Start sending data to USART transmitter.
    int32_t (*send)(int32_t dev, const char *data, uint32_t num);

    ///< Pointer to \ref SUNXI_HAL_USART_Receive : Start receiving data from USART receiver.
    int32_t (*receive)(int32_t dev, char *data, uint32_t num);

    ///< Pointer to \ref SUNXI_HAL_USART_Transfer : Start sending/receiving data to/from USART.
    int32_t (*transfer)(int32_t dev, const void *data_out, void *data_in, uint32_t    num);

    ///< Pointer to \ref SUNXI_HAL_USART_GetTxCount : Get transmitted data count.
    uint32_t (*get_tx_count)(int32_t dev);

    ///< Pointer to \ref SUNXI_HAL_USART_GetRxCount : Get received data count.
    uint32_t (*get_rx_count)(int32_t dev);

    ///< Pointer to \ref SUNXI_HAL_USART_Control : Control USART Interface.
    int32_t (*control)(int32_t dev, uint32_t control, uint32_t arg);

    ///< Pointer to \ref SUNXI_HAL_USART_GetStatus : Get USART status.
    sunxi_hal_usart_status_t (*get_status)(int32_t dev);
    ///< Pointer to \ref SUNXI_HAL_USART_SetModemControl : Set USART Modem Control line state.
    int32_t (*set_modem_control)(int32_t dev, sunxi_hal_usart_modem_control_e control);

    ///< Pointer to \ref SUNXI_HAL_USART_GetModemStatus : Get USART Modem Status lines state.
    sunxi_hal_usart_modem_status_t (*get_modem_status)(int32_t dev);
} const sunxi_hal_driver_usart_t;

void hal_uart_register_callback(uart_port_t uart_port,
				uart_callback_t user_callback,
				void *user_data);

#ifdef __cplusplus
}
#endif

#endif
