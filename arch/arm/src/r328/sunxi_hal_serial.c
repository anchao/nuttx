#include "sunxi_hal_serial.h"

#include "hal_serial.h"
#include <nuttx/config.h>
#include <stdio.h>
#include <interrupt.h>
#include "hal_gpio.h"
#include "aw_types.h"
#include "hal_clk.h"
#include <stdint.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include "up_internal.h"
#include "up_arch.h"
#include "chip.h"
static void uart_enable_irq(uart_port_t uart_port, uint32_t irq_type);
static void uart_disable_irq(uart_port_t uart_port, uint32_t irq_type);


typedef struct {
	hal_clk_id_t clk_id;
}hal_set_clk_t;

static hal_set_clk_t clock_id[] = {
		{HAL_CLK_PERIPH_UART0},
		{HAL_CLK_PERIPH_UART1},
		{HAL_CLK_PERIPH_UART2},
		{HAL_CLK_PERIPH_UART3},
	};

#if (0)
#define UART_LOG_DEBUG
#endif
#define UART_INIT(fmt, ...) printf("uart: "fmt, ##__VA_ARGS__)
#define UART_ERR(fmt, ...)  printf("uart: "fmt, ##__VA_ARGS__)

#ifdef UART_LOG_DEBUG
#define UART_INFO(fmt, ...) printf("[%s %d]"fmt, __func__, __LINE__, ##__VA_ARGS__)
#else
#define UART_INFO(fmt, ...)
#endif

#define HAL_ARG_UNUSED(NAME)   (void)(NAME)

#define hal_readb(reg)          (*(volatile uint8_t  *)(reg))
#define hal_readw(reg)          (*(volatile uint16_t *)(reg))
#define hal_readl(reg)          (*(volatile uint32_t *)(reg))
#define hal_writeb(value,reg)   (*(volatile uint8_t  *)(reg) = (value))
#define hal_writew(value,reg)   (*(volatile uint16_t *)(reg) = (value))
#define hal_writel(value,reg)   (*(volatile uint32_t *)(reg) = (value))

static uint32_t sunxi_uart_port[] =
{
    APB_USART0_BASE, APB_USART1_BASE, APB_USART2_BASE, APB_USART3_BASE
};
static const uint32_t g_uart_irqn[] = {R328_IRQ_UART0, R328_IRQ_UART1,
                                       R328_IRQ_UART2, R328_IRQ_UART3
                                      };
static sunxi_hal_version_t hal_uart_driver =
{
    SUNXI_HAL_USART_API_VERSION,
    SUNXI_HAL_USART_DRV_VERSION
};
static uart_priv_t g_uart_priv[UART_MAX];

static uint8_t g_uart_buffer[UART_MAX][256];

static const uint32_t g_uart_baudrate_map[] =
{
    300,
    600,
    1200,
    2400,
    4800,
    9600,
    19200,
    38400,
    57600,
    115200,
    230400,
    576000,
    921600,
    1000000,
    1500000,
    3000000,
    4000000,
};

//driver capabilities, support uart function only.
static const sunxi_hal_usart_capabilities_t driver_capabilities =
{
    1, /* supports UART (Asynchronous) mode */
    0, /* supports Synchronous Master mode */
    0, /* supports Synchronous Slave mode */
    0, /* supports UART Single-wire mode */
    0, /* supports UART IrDA mode */
    0, /* supports UART Smart Card mode */
    0, /* Smart Card Clock generator available */
    0, /* RTS Flow Control available */
    0, /* CTS Flow Control available */
    0, /* Transmit completed event: \ref ARM_USARTx_EVENT_TX_COMPLETE */
    0, /* Signal receive character timeout event: \ref ARM_USARTx_EVENT_RX_TIMEOUT */
    0, /* RTS Line: 0=not available, 1=available */
    0, /* CTS Line: 0=not available, 1=available */
    0, /* DTR Line: 0=not available, 1=available */
    0, /* DSR Line: 0=not available, 1=available */
    0, /* DCD Line: 0=not available, 1=available */
    0, /* RI Line: 0=not available, 1=available */
    0, /* Signal CTS change event: \ref ARM_USARTx_EVENT_CTS */
    0, /* Signal DSR change event: \ref ARM_USARTx_EVENT_DSR */
    0, /* Signal DCD change event: \ref ARM_USARTx_EVENT_DCD */
    0, /* Signal RI change event: \ref ARM_USARTx_EVENT_RI */
    0  /* Reserved */
};

static bool uart_port_is_valid(uart_port_t uart_port)
{
    return (uart_port < UART_MAX);
}

static bool uart_baudrate_is_valid(uart_baudrate_t baudrate)
{
    return (baudrate < UART_BAUDRATE_MAX);
}

static bool uart_config_is_valid(const _uart_config_t *config)
{
    return ((config->baudrate < UART_BAUDRATE_MAX) &&
            (config->word_length <= UART_WORD_LENGTH_8) &&
            (config->stop_bit <= UART_STOP_BIT_2) &&
            (config->parity <= UART_PARITY_EVEN));
}

static sunxi_hal_version_t get_version(int32_t dev)
{
    HAL_ARG_UNUSED(dev);
    return hal_uart_driver;
}

static sunxi_hal_usart_capabilities_t get_capabilities(int32_t dev)
{
    HAL_ARG_UNUSED(dev);
    return driver_capabilities;
}

static void uart_set_format(uart_port_t uart_port, uart_word_length_t word_length,
                            uart_stop_bit_t stop_bit, uart_parity_t parity)
{
    const unsigned long uart_base = sunxi_uart_port[uart_port];
    uart_priv_t *uart_priv = &g_uart_priv[uart_port];
    uint32_t value;

    value = hal_readb(uart_base + UART_LCR);

    /* set word length */
    value &= ~(UART_LCR_DLEN_MASK);
    switch (word_length)
    {
        case UART_WORD_LENGTH_5:
            value |= UART_LCR_WLEN5;
            break;
        case UART_WORD_LENGTH_6:
            value |= UART_LCR_WLEN6;
            break;
        case UART_WORD_LENGTH_7:
            value |= UART_LCR_WLEN7;
            break;
        case UART_WORD_LENGTH_8:
        default:
            value |= UART_LCR_WLEN8;
            break;
    }

    /* set stop bit */
    switch (stop_bit)
    {
        case UART_STOP_BIT_1:
        default:
            value &= ~(UART_LCR_STOP);
            break;
        case UART_STOP_BIT_2:
            value |= UART_LCR_STOP;
            break;
    }

    /* set parity bit */
    value &= ~(UART_LCR_PARITY_MASK);
    switch (parity)
    {
        case UART_PARITY_NONE:
            value &= ~(UART_LCR_PARITY);
            break;
        case UART_PARITY_ODD:
            value |= UART_LCR_PARITY;
            break;
        case UART_PARITY_EVEN:
            value |= UART_LCR_PARITY;
            value |= UART_LCR_EPAR;
            break;
    }

    uart_priv->lcr = value;
    hal_writeb(uart_priv->lcr, uart_base + UART_LCR);
}

static void uart_set_baudrate(uart_port_t uart_port, uart_baudrate_t baudrate)
{
    const unsigned long uart_base = sunxi_uart_port[uart_port];
    uart_priv_t *uart_priv = &g_uart_priv[uart_port];
    uint32_t actual_baudrate = g_uart_baudrate_map[baudrate];
    uint32_t quot, uart_clk;

    uart_clk = 24000000; /* FIXME: fixed to 24MHz */

    quot = (uart_clk + 8 * actual_baudrate) / (16 * actual_baudrate);

    UART_INFO("baudrate: %d, quot = %d\r\n", actual_baudrate, quot);

    uart_priv->dlh = quot >> 8;
    uart_priv->dll = quot & 0xff;

    /* hold tx so that uart will update lcr and baud in the gap of tx */
    hal_writeb(UART_HALT_HTX | UART_HALT_FORCECFG, uart_base + UART_HALT);
    hal_writeb(uart_priv->lcr | UART_LCR_DLAB, uart_base + UART_LCR);
    hal_writeb(uart_priv->dlh, uart_base + UART_DLH);
    hal_writeb(uart_priv->dll, uart_base + UART_DLL);
    hal_writeb(UART_HALT_HTX | UART_HALT_FORCECFG | UART_HALT_LCRUP, uart_base + UART_HALT);
    /* FIXME: implement timeout */
    while (hal_readb(uart_base + UART_HALT) & UART_HALT_LCRUP)
        ;

    /* In fact there are two DLABs(DLAB and DLAB_BAK) in the hardware implementation.
     * The DLAB_BAK is sellected only when SW_UART_HALT_FORCECFG is set to 1,
     * and this bit can be access no matter uart is busy or not.
     * So we select the DLAB_BAK always by leaving SW_UART_HALT_FORCECFG to be 1. */
    hal_writeb(uart_priv->lcr, uart_base + UART_LCR);
    hal_writeb(UART_HALT_FORCECFG, uart_base + UART_HALT);
}

void uart_set_fifo(uart_port_t uart_port, uint32_t value)
{
    const unsigned long uart_base = sunxi_uart_port[uart_port];
    uart_priv_t *uart_priv = &g_uart_priv[uart_port];

    uart_priv->fcr = value;
    hal_writeb(uart_priv->fcr, uart_base + UART_FCR);
}

static void uart_force_idle(uart_port_t uart_port)
{
    const unsigned long uart_base = sunxi_uart_port[uart_port];
    uart_priv_t *uart_priv = &g_uart_priv[uart_port];

    if (uart_priv->fcr & UART_FCR_FIFO_EN)
    {
        hal_writeb(UART_FCR_FIFO_EN, uart_base + UART_FCR);
        hal_writeb(UART_FCR_TXFIFO_RST
                   | UART_FCR_RXFIFO_RST
                   | UART_FCR_FIFO_EN, uart_base + UART_FCR);
        hal_writeb(0, uart_base + UART_FCR);
    }

    hal_writeb(uart_priv->fcr, uart_base + UART_FCR);
    (void)hal_readb(uart_base + UART_FCR);
}

static void uart_handle_busy(uart_port_t uart_port)
{
    const unsigned long uart_base = sunxi_uart_port[uart_port];
    uart_priv_t *uart_priv = &g_uart_priv[uart_port];

    (void)hal_readb(uart_base + UART_USR);

    /*
     * Before reseting lcr, we should ensure than uart is not in busy
     * state. Otherwise, a new busy interrupt will be introduced.
     * It is wise to set uart into loopback mode, since it can cut down the
     * serial in, then we should reset fifo(in my test, busy state
     * (UART_USR_BUSY) can't be cleard until the fifo is empty).
     */
    hal_writeb(uart_priv->mcr | UART_MCR_LOOP, uart_base + UART_MCR);
    uart_force_idle(uart_port);
    hal_writeb(uart_priv->lcr, uart_base + UART_LCR);
    hal_writeb(uart_priv->mcr, uart_base + UART_MCR);
}

static int32_t uart_ring_buf_init(uart_port_t uart_port, uint8_t *buf, uint32_t len)
{
	uart_priv_t *uart_priv = &g_uart_priv[uart_port];
	uart_ring_buf_t *rb = &uart_priv->ring_buf;

	if (len < 2 || !buf) {
		printf("ring buffer init err\n");
		return -1;
	}

	rb->buf = buf;
	rb->len = len;
	rb->cnt = 0;
	rb->head = rb->tail = 0;

	return 0;
}

static int32_t uart_ring_buf_put(uart_port_t uart_port, uint8_t data)
{
	uart_priv_t *uart_priv = &g_uart_priv[uart_port];
	uart_ring_buf_t *rb = &uart_priv->ring_buf;

	if (rb->cnt >= rb->len) {
		printf("ring buffer is full\n");
		return -1;
	}

	rb->buf[rb->tail] = data;
	rb->tail++;
	rb->cnt++;

	rb->tail = rb->tail % rb->len;

	return 0;
}

int32_t uart_ring_buf_get(uart_port_t uart_port, uint8_t *dst, uint32_t len)
{
	uart_priv_t *uart_priv = &g_uart_priv[uart_port];
	uart_ring_buf_t *rb = &uart_priv->ring_buf;
	int32_t i;
	uint8_t *tmp = dst;

	if (rb->cnt <= 0) {
		printf("ring buffer is empty\n");
		return -1;
	}

	/* disable rx irq to protect critical zone */
	//uart_disable_irq(uart_port, UART_IER_RDI);
	irqstate_t flags;
	flags   = enter_critical_section();

	if (len > rb->cnt)
		len = rb->cnt;

	for (i = 0; i < len; i++) {
		rb->cnt--;
		*tmp = rb->buf[rb->head++];
		tmp++;
		rb->head = rb->head % rb->len;
	}

	/* leave critical zone */
	//uart_enable_irq(uart_port, UART_IER_RDI);
	leave_critical_section(flags);

	return len;
}

static uint32_t uart_handle_rx(uart_port_t uart_port, uint32_t lsr)
{
	const unsigned long uart_base = sunxi_uart_port[uart_port];
	uint8_t ch = 0;
	uint16_t max_count = 256;
	uint32_t ret;

	do {
		if (lsr & UART_LSR_DR) {
			irqstate_t flags;
			flags   = enter_critical_section();

			ch = hal_readb(uart_base + UART_RBR);
			ret = uart_ring_buf_put(uart_port, ch);
			leave_critical_section(flags);
		}

		lsr = hal_readb(uart_base + UART_LSR);
	} while ((lsr & (UART_LSR_DR | UART_LSR_BI))
			&& (max_count > 0) && !ret);

	return lsr;
}

static void uart_handle_tx(uart_port_t uart_port)
{
	const unsigned long uart_base = sunxi_uart_port[uart_port];
	uart_priv_t *uart_priv = &g_uart_priv[uart_port];
	const char *buf = uart_priv->tx_buf;
	uint32_t buf_size = uart_priv->tx_buf_size;
	uint32_t usr;
	uint16_t max_count = 128;

	if (!buf || buf_size <= 0)
		return;

	do {
		usr = hal_readb(uart_base + UART_USR);
		if (usr & UART_USR_TFNF) {
			hal_writeb(*buf, uart_base + UART_THR);
			++buf;
			--buf_size;
			--max_count;
		}
	} while (buf_size > 0 && max_count > 0);

	if (buf_size == 0) {
		/* disable tx irq*/
		//uart_disable_irq(uart_port, UART_IER_THRI);

		/* tx finished, reset tx buffer and callback to user */
		uart_priv->tx_buf = NULL;
		uart_priv->tx_buf_size = 0;
		//callback(UART_EVENT_TX_COMPLETE, uart_priv->arg);
	} else {
		uart_priv->tx_buf = buf;
		uart_priv->tx_buf_size = buf_size;
	}
}

static int uart_irq_handler(int irq, void *dev_id, void *arg)
{
    uart_priv_t *uart_priv = arg;
    uart_port_t uart_port = uart_priv->uart_port;
    const unsigned long uart_base = sunxi_uart_port[uart_port];
    uint32_t iir, lsr;

    iir = hal_readb(uart_base + UART_IIR) & UART_IIR_IID_MASK;
    lsr = hal_readb(uart_base + UART_LSR);

    UART_INFO("IRQ uart%d lsr is %08x \n", uart_port, lsr);
    if (iir == UART_IIR_IID_BUSBSY) {
        uart_handle_busy(uart_port);
    }
    else
    {
        if (lsr & (UART_LSR_DR | UART_LSR_BI))
        {
            lsr = uart_handle_rx(uart_port, lsr);
        }
        else if (iir & UART_IIR_IID_CHARTO)
            /* has charto irq but no dr lsr? just read and ignore */
        {
            hal_readb(uart_base + UART_RBR);
        }

        //if (lsr & UART_LSR_THRE) {
        //  uart_handle_tx(uart_port);
	//}
    }
    return 0;
}

static void uart_enable_irq(uart_port_t uart_port, uint32_t irq_type)
{
    const unsigned long uart_base = sunxi_uart_port[uart_port];
    uint32_t value;

    value = hal_readb(uart_base + UART_IER);
    value |= irq_type;
    hal_writeb(value, uart_base + UART_IER);
}


static void uart_disable_irq(uart_port_t uart_port, uint32_t irq_type)
{
	const unsigned long uart_base = sunxi_uart_port[uart_port];
	uint32_t value;

	value = hal_readb(uart_base + UART_IER);
	value &= ~(irq_type);
	hal_writeb(value, uart_base + UART_IER);
}

static void uart_enable_busy_cfg(uart_port_t uart_port)
{
    const unsigned long uart_base = sunxi_uart_port[uart_port];
    uint32_t value;

    value = hal_readb(uart_base + UART_HALT);
    value |= UART_HALT_FORCECFG;
    hal_writeb(value, uart_base + UART_HALT);
}

static void uart_pinctrl_init(uart_port_t uart_port)
{
    switch (uart_port)
    {
        case UART_0:
            hal_gpio_pinmux_set_function(HAL_GPIO_224, 3);//TX
            hal_gpio_pinmux_set_function(HAL_GPIO_225, 3);//RX
            break;
        case UART_1:
            hal_gpio_pinmux_set_function(HAL_GPIO_198, 2);//TX
            hal_gpio_pinmux_set_function(HAL_GPIO_199, 2);//RX
            break;
        case UART_2:
            hal_gpio_pinmux_set_function(HAL_GPIO_32, 2);//TX
            hal_gpio_pinmux_set_function(HAL_GPIO_33, 2);//RX
            break;
        case UART_3:
            hal_gpio_pinmux_set_function(HAL_GPIO_228, 2);//TX
            hal_gpio_pinmux_set_function(HAL_GPIO_229, 2);//RX
            break;
        default:
            break;
    }
}

static _uart_config_t uart_defconfig =
{
    .baudrate    = UART_BAUDRATE_115200,
    .word_length = UART_WORD_LENGTH_8,
    .stop_bit    = UART_STOP_BIT_1,
    .parity      = UART_PARITY_NONE,
};//defult uart config

static int32_t initialize(int32_t uart_port)
{
    uart_priv_t *uart_priv = &g_uart_priv[uart_port];
    uint32_t irqn = g_uart_irqn[uart_port];
    uint32_t value = 0;
    _uart_config_t *uart_config = &uart_defconfig;
    char uart_name[8];

    UART_INIT("Initializing uart%ld \n", uart_port);
    if ((!uart_port_is_valid(uart_port)) ||
        (!uart_config_is_valid(uart_config)))
    {
        return HAL_UART_STATUS_ERROR_PARAMETER;
    }

    /* enable clk */
    hal_set_clk_t sys_clk = clock_id[uart_port];
    hal_clock_enable(sys_clk.clk_id);

    /* request gpio */
    uart_pinctrl_init(uart_port);

    /* config uart attributes */
    uart_set_format(uart_port, uart_config->word_length,
                    uart_config->stop_bit, uart_config->parity);
    uart_set_baudrate(uart_port, uart_config->baudrate);

    value |= UART_FCR_RXTRG_1_2 | UART_FCR_TXTRG_1_2 | UART_FCR_FIFO_EN;
    uart_set_fifo(uart_port, value);

    sprintf(uart_name, "uart%ld", uart_port);
    if (uart_priv->uart_port == uart_port && uart_priv->irqn == irqn)
    {
        UART_ERR("irq for uart%ld already enabled\n", uart_port);
    }
    else
    {
        uart_priv->uart_port = uart_port;
        uart_priv->irqn = irqn;

	irq_attach(irqn, uart_irq_handler, uart_priv);
	up_enable_irq(irqn);
    }
    /* set uart IER */
    uart_enable_irq(uart_port, UART_IER_RDI | UART_IER_RLSI);

    /* force config */
    uart_enable_busy_cfg(uart_port);

	uint8_t *uart_buf = &g_uart_buffer[uart_port][0];
	uart_ring_buf_init(uart_port, uart_buf, 256);

    return SUNXI_HAL_OK;
}

static int32_t uninitialize(int32_t dev)
{
    //TODO: PINMUX and CLK
    return SUNXI_HAL_OK;
}

static int32_t power_control(int32_t dev, sunxi_hal_power_state_e state)
{
    return SUNXI_HAL_OK;
}

static int _uart_putc(int devid, char c)
{
    volatile uint32_t *sed_buf;
    volatile uint32_t *sta;

    sed_buf = (uint32_t *)(sunxi_uart_port[devid] + UART_THR);
    sta = (uint32_t *)(sunxi_uart_port[devid] + UART_USR);

    /* FIFO status, contain valid data */
    while (!(*sta & 0x02));
    *sed_buf = c;

    return 1;
}

static int32_t _uart_send(int32_t dev, const char *data, uint32_t num)
{
    int size;

    assert(data != NULL);

    size = num;
    while (num && (*data != '\0'))
    {
        /*
        ¦   ¦* to be polite with serial console add a line feed
        ¦   ¦* to the carriage return character
        ¦   ¦*/
        if (*data == '\n')
        {
            _uart_putc(dev, '\r');
        }

        _uart_putc(dev, *data);

        ++ data;
        -- num;
    }

    return size - num;
}

static int _uart_getc(int devid)
{
    int ch = -1;
    volatile uint32_t *rec_buf;
    volatile uint32_t *sta;
    volatile uint32_t *fifo;

    rec_buf = (uint32_t *)(sunxi_uart_port[devid] + UART_RHB);
    sta = (uint32_t *)(sunxi_uart_port[devid] + UART_USR);
    fifo = (uint32_t *)(sunxi_uart_port[devid] + UART_RFL);

    while (!(*fifo & 0x1ff));

    /* Receive Data Available */
    if (*sta & 0x08)
    {
        ch = *rec_buf & 0xff;
    }

    return ch;
}

static int32_t receive(int32_t dev, char *data, uint32_t num)
{
	uint8_t *tmp = (uint8_t *)data;
	if(data != NULL) {
		uart_ring_buf_get(dev, tmp, num);
	}
	return num;

#if 0
    int ch;
    int size;

    assert(data != NULL);
    size = num;

    while (num)
    {
        ch = _uart_getc(dev);
        if (ch == -1)
        {
            break;
        }
        *data = ch;
        data ++;
        num --;

        if (ch == '\n')
        {
            break;
        }
    }

    return size - num;
#endif

}

static int32_t transfer(int32_t dev, const void *data_out, void *data_in, uint32_t num)
{
    return SUNXI_HAL_OK;
}

static uint32_t get_tx_count(int32_t dev)
{
    //TODO: need verify
    return 0;
}

static uint32_t get_rx_count(int32_t dev)
{
    //TODO: need verify
    return 0;
}

static int32_t control(int32_t dev, uint32_t control, uint32_t arg)
{
	sunxi_uart_control_t cmd = control;
	switch(cmd) {
	case SUNXI_SET_BAUD:
		uart_set_baudrate(dev, arg);
		break;
	case SUNXI_SET_FORMAT:
	default:
		break;
	}

    return SUNXI_HAL_OK;
}

static sunxi_hal_usart_status_t get_status(int32_t dev)
{
    sunxi_hal_usart_status_t status = {1, 1, 0, 0, 0, 0, 0, 0};
    return status;
}

static int32_t set_modem_control(int32_t dev, sunxi_hal_usart_modem_control_e control)
{
    return SUNXI_HAL_OK;
}

static sunxi_hal_usart_modem_status_t get_modem_status(int32_t dev)
{
    sunxi_hal_usart_modem_status_t status = {0, 0, 0, 0, 0};
    return status;
}

const sunxi_hal_driver_usart_t sunxi_hal_usart_driver =
{
    .get_version  = get_version,
    .get_capabilities = get_capabilities,
    .initialize = initialize,
    .uninitialize = uninitialize,
    .power_control = power_control,
    .send = _uart_send,
    .receive = receive,
    .transfer = transfer,
    .get_tx_count = get_tx_count,
    .get_rx_count = get_rx_count,
    .control = control,
    .get_status = get_status,
    .set_modem_control = set_modem_control,
    .get_modem_status = get_modem_status,
};

int serial_driver_init(void)
{
    sinfo("serial hal driver init");
    return 0;
}
