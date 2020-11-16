/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2016 Linaro Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_uart

/**
 * @brief Driver for UART port on STM32 family processor.
 * @note  LPUART and U(S)ART have the same base and
 *        majority of operations are performed the same way.
 *        Please validate for newly added series.
 */

#include <kernel.h>
#include <arch/cpu.h>
#include <sys/__assert.h>
#include <soc.h>
#include <init.h>
#include <drivers/uart.h>
#include <drivers/pinmux.h>
#include <pinmux/stm32/pinmux_stm32.h>
#include <drivers/clock_control.h>

#include <linker/sections.h>
#include <drivers/clock_control/stm32_clock_control.h>
#include "uart_stm32.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(uart_stm32);

#define HAS_LPUART_1 (DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(lpuart1), \
					 st_stm32_lpuart, okay))

/* convenience defines */
#define DEV_CFG(dev)							\
	((const struct uart_stm32_config * const)(dev)->config)
#define DEV_DATA(dev)							\
	((struct uart_stm32_data * const)(dev)->data)
#define UART_STRUCT(dev)					\
	((USART_TypeDef *)(DEV_CFG(dev))->uconf.base)

#define TIMEOUT 1000

#ifdef CONFIG_UART_ASYNC_API
#define STM32_UART_SECONDARY_BUFFER_TIMEOUT 1000

#ifndef CONFIG_UART_INTERRUPT_DRIVEN
#error Stm32 UART async driver needs UART_INTERRUPT_DRIVEN switched on
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#define LOCKED 0
#define UNLOCKED 1

static void uart_stm32_irq_tx_enable(const struct device *dev);
static void uart_stm32_irq_rx_enable(const struct device *dev);

static int uart_stm32_resource_lock(atomic_t *lock){
    if (*lock == LOCKED)
        return -EBUSY;
    bool success = atomic_cas ( lock,
                                UNLOCKED,
                                LOCKED);
    if (!success)
        return -EBUSY;

    return 0;
}

static inline int uart_stm32_resource_unlock(atomic_t *lock){
    *lock = UNLOCKED;
    return 0;
}

static int uart_stm32_callback_set(const struct device *dev,
                                          uart_callback_t callback,
                                          void *user_data)
{
    struct uart_stm32_data *data = DEV_DATA(dev);

    data->async_callback = callback;
    data->user_data = user_data;
    return 0;
}

static int uart_stm32_tx(const struct device *dev, const uint8_t *buf,
            size_t len,
            int32_t timeout)
{
    struct uart_stm32_data *data = DEV_DATA(dev);

    if (len == 0 || dev == NULL)
        return -ENOTSUP;

    if (uart_stm32_resource_lock(&data->tx_buffer_lock) != 0)
        return -EBUSY;

    if (timeout != SYS_FOREVER_MS)
        k_timer_start(data->tx_timer,
                      K_MSEC(timeout),
                      K_NO_WAIT);

    uart_stm32_irq_tx_enable(dev);
    data->tx_counter = 1;
    data->tx_abort_pending = false;
    data->tx_buffer = buf;
    data->tx_buffer_length = len;
    USART_TypeDef *UartInstance = UART_STRUCT(dev);
    --data->tx_buffer_length;
    LL_USART_TransmitData8(UartInstance, data->tx_buffer[0]);
    return SUCCESS;
}

static void _uart_stm32_tx_finish(const struct device *dev,
                              enum uart_event_type event)
{
    struct uart_stm32_data *data = DEV_DATA(dev);
    data->tx_buffer_length = 0;
    data->tx_counter = 0;
    data->tx_buffer = NULL;
    data->tx_abort_pending = false;
    k_timer_stop(data->tx_timer);
    uart_stm32_resource_unlock(&data->tx_buffer_lock);
    struct uart_event evt = {
        .type = event,
        .data.tx.buf = data->tx_buffer,
        .data.tx.len = data->tx_buffer_length
    };
    if (data->async_callback != NULL)
    {
        data->async_callback(dev, &evt, data->user_data);
    }
}

static int uart_stm32_tx_abort(const struct device *dev)
{
    struct uart_stm32_data *data = DEV_DATA(dev);

    if (data->tx_buffer_lock == LOCKED)
    {
        return -EFAULT;
    }
    data->tx_abort_pending = true;
    return 0;
}

static inline void uart_stm32_tx_timeout(struct k_timer *timer,
                                  const struct device *dev)
{
    UNUSED(timer);
    struct uart_stm32_data *data = DEV_DATA(dev);
    if (data->tx_buffer_lock == LOCKED)
        data->tx_abort_pending = true;
}

static inline void uart_stm32_tx_isr(const struct device *dev,
                                     struct uart_stm32_data *data,
                                     USART_TypeDef *UartInstance)
{
    LL_USART_ClearFlag_TC(UartInstance);
    if (data->tx_abort_pending)
        _uart_stm32_tx_finish(dev, UART_TX_ABORTED);
    else if (data->tx_counter <= data->tx_buffer_length)
    {
        LL_USART_TransmitData8(UartInstance, data->tx_buffer[data->tx_counter++]);
    } else {
        if (data->tx_buffer != NULL)
            _uart_stm32_tx_finish(dev, UART_TX_DONE);
    }
}

static void _uart_stm32_async_callback(const struct device *dev,
                                     struct uart_event *evt)
{
    struct uart_stm32_data *data = DEV_DATA(dev);
    if(data->async_callback != NULL)
        data->async_callback(dev, evt, data->user_data);
}

static void _uart_stm32_rx_ready_callback(const struct device *dev)
{
    struct uart_stm32_data *data = DEV_DATA(dev);
    if(data->async_callback != NULL)
    {
        struct uart_event evt = {
            .type = UART_RX_RDY,
        };
        evt.data.rx.buf = data->rx_buffer;
        evt.data.rx.len = data->rx_counter - data->rx_offset;
        evt.data.rx.offset = data->rx_offset;

        data->rx_offset = data->rx_counter;
        data->async_callback(dev, &evt, data->user_data);
    }
}

static void _uart_stm32_simple_callback(const struct device *dev,
                                                enum uart_event_type event)
{
    struct uart_stm32_data *data = DEV_DATA(dev);
    if(data->async_callback != NULL)
    {
        struct uart_event evt = {
            .type = event
        };
        data->async_callback(dev, &evt, data->user_data);
    }
}

static void _uart_stm32_stopped_callback(const struct device *dev,
                                         enum uart_rx_stop_reason reason)
{
    struct uart_stm32_data *data = DEV_DATA(dev);
    if(data->async_callback != NULL)
    {
        struct uart_event evt = {
            .type = UART_RX_STOPPED,
            .data.rx_stop.reason = reason
        };
        data->async_callback(dev, &evt, data->user_data);
    }
}

static void _uart_stm32_rx_buffer_released_callback(const struct device *dev)
{
    struct uart_stm32_data *data = DEV_DATA(dev);
    if(data->async_callback != NULL)
    {
        struct uart_event evt = {
            .type = UART_RX_BUF_RELEASED,
            .data.rx_buf.buf = data->rx_buffer
        };
        data->async_callback(dev, &evt, data->user_data);
    }
}


static void _uart_stm32_reset_rx_buffer(struct uart_stm32_data *data)
{
    data->rx_buffer = NULL;
    data->rx_offset = 0;
    data->rx_buffer_length = 0;
    data->rx_timeout = 0;
    data->rx_counter = 0;
}


static int uart_stm32_rx_enable(const struct device *dev, uint8_t *buf,
                   size_t len,
                   int32_t timeout)
{
    struct uart_stm32_data *data = DEV_DATA(dev);

    if (len == 0 || dev == NULL)
        return -ENOTSUP;

    if (uart_stm32_resource_lock(&data->rx_buffer_lock) != 0)
        return -EBUSY;

    uart_stm32_irq_rx_enable(dev);
    data->rx_buffer = buf;
    data->rx_buffer_length = len;
    data->rx_counter = 0;
    data->rx_offset = 0;
    data->rx_timeout = timeout;
    data->rx_secondary_buffer = NULL;
    data->rx_secondary_buffer_length = 0;

    USART_TypeDef *UartInstance = UART_STRUCT(dev);
    LL_USART_ClearFlag_ORE(UartInstance);
    LL_USART_ClearFlag_NE(UartInstance);
    LL_USART_ClearFlag_FE(UartInstance);
    LL_USART_ClearFlag_PE(UartInstance);
    return 0;
}

static int uart_stm32_rx_buf_rsp(const struct device *dev, uint8_t *buf,
                size_t len)
{
    struct uart_stm32_data *data = DEV_DATA(dev);

    if (data->rx_secondary_buffer_length != 0)
        return -EBUSY;

    if (data->rx_secondary_buffer_lock == LOCKED)
        return -EACCES;

    if (uart_stm32_resource_lock(&data->rx_secondary_buffer_lock) != 0)
        return -EBUSY;

    data->rx_secondary_buffer = buf;
    data->rx_secondary_buffer_length = len;

    uart_stm32_resource_unlock(&data->rx_secondary_buffer_lock);
    return 0;
}

static int uart_stm32_rx_disable(const struct device *dev){
    struct uart_stm32_data *data = DEV_DATA(dev);

    if (data->rx_buffer_lock == UNLOCKED)
        return -EFAULT;

    k_timer_stop(data->rx_timer);
    uart_stm32_resource_unlock(&data->rx_buffer_lock);

    struct uart_event evt = {
        .type = UART_RX_DISABLED
    };
    _uart_stm32_async_callback(dev, &evt);

    if (data->rx_counter > data->rx_offset)
        _uart_stm32_rx_ready_callback(dev);

    _uart_stm32_reset_rx_buffer(data);
    return 0;
}

static void _uart_stm32_stop_with_error(const struct device *dev,
                                        int err)
{
    struct uart_stm32_data *data = DEV_DATA(dev);
    enum uart_rx_stop_reason errors[] = {
        UART_ERROR_OVERRUN,
        UART_ERROR_FRAMING,
        UART_ERROR_PARITY,
        UART_ERROR_NOISE};

    k_timer_stop(data->rx_timer);
    uart_stm32_resource_unlock(&data->rx_buffer_lock);
    _uart_stm32_rx_ready_callback(dev);
    for (int i = 0; i < 4; ++i){
        if (err & errors[i])
            _uart_stm32_stopped_callback(dev, errors[i]);
    }
    _uart_stm32_rx_buffer_released_callback(dev);
    _uart_stm32_simple_callback(dev, UART_RX_DISABLED);
    _uart_stm32_reset_rx_buffer(data);
}

static inline void uart_stm32_rx_timeout(struct k_timer *timer,
                                  const struct device *dev)
{
    UNUSED(timer);
    _uart_stm32_rx_ready_callback(dev);
}

static inline void uart_stm32_rx_isr(const struct device *dev,
                                           struct uart_stm32_data *data,
                                           USART_TypeDef *UartInstance)
{
    uint8_t dr = LL_USART_ReceiveData8(UartInstance);
    if (data->rx_buffer != NULL)
    {
        if (data->rx_counter < data->rx_buffer_length)
        {
            if (data->rx_counter == 0)
            {
                struct uart_event evt = {
                    .type = UART_RX_BUF_REQUEST
                };
                _uart_stm32_async_callback(dev, &evt);
            }

            data->rx_buffer[data->rx_counter++] = dr;
            if (data->rx_timeout != SYS_FOREVER_MS)
                k_timer_start(data->rx_timer,
                              K_MSEC(data->rx_timeout),
                              K_NO_WAIT);

            /* Is Rx buffer filled? */
            if (data->rx_counter == data->rx_buffer_length)
            {
                if (data->rx_secondary_buffer == NULL)
                {
                    k_timer_stop(data->rx_timer);
                    uart_stm32_resource_unlock(&data->rx_buffer_lock);
                    _uart_stm32_rx_ready_callback(dev);
                    _uart_stm32_rx_buffer_released_callback(dev);
                    _uart_stm32_simple_callback(dev, UART_RX_DISABLED);
                    _uart_stm32_reset_rx_buffer(data);
                } else {
                    _uart_stm32_rx_ready_callback(dev);
                    _uart_stm32_rx_buffer_released_callback(dev);
                    if (uart_stm32_resource_lock(&data->rx_secondary_buffer_lock) != 0)
                    {
                        _uart_stm32_simple_callback(dev, UART_RX_DISABLED);
                    } else {
                        data->rx_buffer = data->rx_secondary_buffer;
                        data->rx_buffer_length = data->rx_secondary_buffer_length;
                        data->rx_secondary_buffer = NULL;
                        data->rx_secondary_buffer_length = 0;
                        uart_stm32_resource_unlock(&data->rx_secondary_buffer_lock);
                        data->rx_counter = 0;
                        data->rx_offset = 0;
                    }
                }
            }
        }
    }
}

#endif /* CONFIG_UART_ASYNC_API */

static inline void uart_stm32_set_baudrate(const struct device *dev,
					   uint32_t baud_rate)
{
	const struct uart_stm32_config *config = DEV_CFG(dev);
	struct uart_stm32_data *data = DEV_DATA(dev);
	USART_TypeDef *UartInstance = UART_STRUCT(dev);

	uint32_t clock_rate;

	/* Get clock rate */
	if (clock_control_get_rate(data->clock,
			       (clock_control_subsys_t *)&config->pclken,
			       &clock_rate) < 0) {
		LOG_ERR("Failed call clock_control_get_rate");
		return;
	}


#if HAS_LPUART_1
	if (IS_LPUART_INSTANCE(UartInstance)) {
		LL_LPUART_SetBaudRate(UartInstance,
				      clock_rate,
#ifdef USART_PRESC_PRESCALER
				      LL_USART_PRESCALER_DIV1,
#endif
				      baud_rate);
	} else {
#endif /* HAS_LPUART_1 */

		LL_USART_SetBaudRate(UartInstance,
				     clock_rate,
#ifdef USART_PRESC_PRESCALER
				     LL_USART_PRESCALER_DIV1,
#endif
#ifdef USART_CR1_OVER8
				     LL_USART_OVERSAMPLING_16,
#endif
				     baud_rate);

#if HAS_LPUART_1
	}
#endif /* HAS_LPUART_1 */
}

static inline void uart_stm32_set_parity(const struct device *dev,
					 uint32_t parity)
{
	USART_TypeDef *UartInstance = UART_STRUCT(dev);

	LL_USART_SetParity(UartInstance, parity);
}

static inline uint32_t uart_stm32_get_parity(const struct device *dev)
{
	USART_TypeDef *UartInstance = UART_STRUCT(dev);

	return LL_USART_GetParity(UartInstance);
}

static inline void uart_stm32_set_stopbits(const struct device *dev,
					   uint32_t stopbits)
{
	USART_TypeDef *UartInstance = UART_STRUCT(dev);

	LL_USART_SetStopBitsLength(UartInstance, stopbits);
}

static inline uint32_t uart_stm32_get_stopbits(const struct device *dev)
{
	USART_TypeDef *UartInstance = UART_STRUCT(dev);

	return LL_USART_GetStopBitsLength(UartInstance);
}

static inline void uart_stm32_set_databits(const struct device *dev,
					   uint32_t databits)
{
	USART_TypeDef *UartInstance = UART_STRUCT(dev);

	LL_USART_SetDataWidth(UartInstance, databits);
}

static inline uint32_t uart_stm32_get_databits(const struct device *dev)
{
	USART_TypeDef *UartInstance = UART_STRUCT(dev);

	return LL_USART_GetDataWidth(UartInstance);
}

static inline void uart_stm32_set_hwctrl(const struct device *dev,
					 uint32_t hwctrl)
{
	USART_TypeDef *UartInstance = UART_STRUCT(dev);

	LL_USART_SetHWFlowCtrl(UartInstance, hwctrl);
}

static inline uint32_t uart_stm32_get_hwctrl(const struct device *dev)
{
	USART_TypeDef *UartInstance = UART_STRUCT(dev);

	return LL_USART_GetHWFlowCtrl(UartInstance);
}

static inline uint32_t uart_stm32_cfg2ll_parity(enum uart_config_parity parity)
{
	switch (parity) {
	case UART_CFG_PARITY_ODD:
		return LL_USART_PARITY_ODD;
	case UART_CFG_PARITY_EVEN:
		return LL_USART_PARITY_EVEN;
	case UART_CFG_PARITY_NONE:
	default:
		return LL_USART_PARITY_NONE;
	}
}

static inline enum uart_config_parity uart_stm32_ll2cfg_parity(uint32_t parity)
{
	switch (parity) {
	case LL_USART_PARITY_ODD:
		return UART_CFG_PARITY_ODD;
	case LL_USART_PARITY_EVEN:
		return UART_CFG_PARITY_EVEN;
	case LL_USART_PARITY_NONE:
	default:
		return UART_CFG_PARITY_NONE;
	}
}

static inline uint32_t uart_stm32_cfg2ll_stopbits(enum uart_config_stop_bits sb)
{
	switch (sb) {
/* Some MCU's don't support 0.5 stop bits */
#ifdef LL_USART_STOPBITS_0_5
	case UART_CFG_STOP_BITS_0_5:
		return LL_USART_STOPBITS_0_5;
#endif	/* LL_USART_STOPBITS_0_5 */
	case UART_CFG_STOP_BITS_1:
		return LL_USART_STOPBITS_1;
/* Some MCU's don't support 1.5 stop bits */
#ifdef LL_USART_STOPBITS_1_5
	case UART_CFG_STOP_BITS_1_5:
		return LL_USART_STOPBITS_1_5;
#endif	/* LL_USART_STOPBITS_1_5 */
	case UART_CFG_STOP_BITS_2:
	default:
		return LL_USART_STOPBITS_2;
	}
}

static inline enum uart_config_stop_bits uart_stm32_ll2cfg_stopbits(uint32_t sb)
{
	switch (sb) {
/* Some MCU's don't support 0.5 stop bits */
#ifdef LL_USART_STOPBITS_0_5
	case LL_USART_STOPBITS_0_5:
		return UART_CFG_STOP_BITS_0_5;
#endif	/* LL_USART_STOPBITS_0_5 */
	case LL_USART_STOPBITS_1:
		return UART_CFG_STOP_BITS_1;
/* Some MCU's don't support 1.5 stop bits */
#ifdef LL_USART_STOPBITS_1_5
	case LL_USART_STOPBITS_1_5:
		return UART_CFG_STOP_BITS_1_5;
#endif	/* LL_USART_STOPBITS_1_5 */
	case LL_USART_STOPBITS_2:
	default:
		return UART_CFG_STOP_BITS_2;
	}
}

static inline uint32_t uart_stm32_cfg2ll_databits(enum uart_config_data_bits db)
{
	switch (db) {
/* Some MCU's don't support 7B or 9B datawidth */
#ifdef LL_USART_DATAWIDTH_7B
	case UART_CFG_DATA_BITS_7:
		return LL_USART_DATAWIDTH_7B;
#endif	/* LL_USART_DATAWIDTH_7B */
#ifdef LL_USART_DATAWIDTH_9B
	case UART_CFG_DATA_BITS_9:
		return LL_USART_DATAWIDTH_9B;
#endif	/* LL_USART_DATAWIDTH_9B */
	case UART_CFG_DATA_BITS_8:
	default:
		return LL_USART_DATAWIDTH_8B;
	}
}

static inline enum uart_config_data_bits uart_stm32_ll2cfg_databits(uint32_t db)
{
	switch (db) {
/* Some MCU's don't support 7B or 9B datawidth */
#ifdef LL_USART_DATAWIDTH_7B
	case LL_USART_DATAWIDTH_7B:
		return UART_CFG_DATA_BITS_7;
#endif	/* LL_USART_DATAWIDTH_7B */
#ifdef LL_USART_DATAWIDTH_9B
	case LL_USART_DATAWIDTH_9B:
		return UART_CFG_DATA_BITS_9;
#endif	/* LL_USART_DATAWIDTH_9B */
	case LL_USART_DATAWIDTH_8B:
	default:
		return UART_CFG_DATA_BITS_8;
	}
}

/**
 * @brief  Get LL hardware flow control define from
 *         Zephyr hardware flow control option.
 * @note   Supports only UART_CFG_FLOW_CTRL_RTS_CTS.
 * @param  fc: Zephyr hardware flow control option.
 * @retval LL_USART_HWCONTROL_RTS_CTS, or LL_USART_HWCONTROL_NONE.
 */
static inline uint32_t uart_stm32_cfg2ll_hwctrl(enum uart_config_flow_control fc)
{
	if (fc == UART_CFG_FLOW_CTRL_RTS_CTS) {
		return LL_USART_HWCONTROL_RTS_CTS;
	}

	return LL_USART_HWCONTROL_NONE;
}

/**
 * @brief  Get Zephyr hardware flow control option from
 *         LL hardware flow control define.
 * @note   Supports only LL_USART_HWCONTROL_RTS_CTS.
 * @param  fc: LL hardware flow control definition.
 * @retval UART_CFG_FLOW_CTRL_RTS_CTS, or UART_CFG_FLOW_CTRL_NONE.
 */
static inline enum uart_config_flow_control uart_stm32_ll2cfg_hwctrl(uint32_t fc)
{
	if (fc == LL_USART_HWCONTROL_RTS_CTS) {
		return UART_CFG_FLOW_CTRL_RTS_CTS;
	}

	return UART_CFG_FLOW_CTRL_NONE;
}

static int uart_stm32_configure(const struct device *dev,
				const struct uart_config *cfg)
{
	struct uart_stm32_data *data = DEV_DATA(dev);
	USART_TypeDef *UartInstance = UART_STRUCT(dev);
	const uint32_t parity = uart_stm32_cfg2ll_parity(cfg->parity);
	const uint32_t stopbits = uart_stm32_cfg2ll_stopbits(cfg->stop_bits);
	const uint32_t databits = uart_stm32_cfg2ll_databits(cfg->data_bits);
	const uint32_t flowctrl = uart_stm32_cfg2ll_hwctrl(cfg->flow_ctrl);

	/* Hardware doesn't support mark or space parity */
	if ((UART_CFG_PARITY_MARK == cfg->parity) ||
	    (UART_CFG_PARITY_SPACE == cfg->parity)) {
		return -ENOTSUP;
	}

#if defined(LL_USART_STOPBITS_0_5) && HAS_LPUART_1
	if (IS_LPUART_INSTANCE(UartInstance) &&
	    UART_CFG_STOP_BITS_0_5 == cfg->stop_bits) {
		return -ENOTSUP;
	}
#else
	if (UART_CFG_STOP_BITS_0_5 == cfg->stop_bits) {
		return -ENOTSUP;
	}
#endif

#if defined(LL_USART_STOPBITS_1_5) && HAS_LPUART_1
	if (IS_LPUART_INSTANCE(UartInstance) &&
	    UART_CFG_STOP_BITS_1_5 == cfg->stop_bits) {
		return -ENOTSUP;
	}
#else
	if (UART_CFG_STOP_BITS_1_5 == cfg->stop_bits) {
		return -ENOTSUP;
	}
#endif

	/* Driver doesn't support 5 or 6 databits and potentially 7 or 9 */
	if ((UART_CFG_DATA_BITS_5 == cfg->data_bits) ||
	    (UART_CFG_DATA_BITS_6 == cfg->data_bits)
#ifndef LL_USART_DATAWIDTH_7B
	    || (UART_CFG_DATA_BITS_7 == cfg->data_bits)
#endif /* LL_USART_DATAWIDTH_7B */
#ifndef LL_USART_DATAWIDTH_9B
	    || (UART_CFG_DATA_BITS_9 == cfg->data_bits)
#endif /* LL_USART_DATAWIDTH_9B */
		) {
		return -ENOTSUP;
	}

	/* Driver supports only RTS CTS flow control */
	if (UART_CFG_FLOW_CTRL_NONE != cfg->flow_ctrl) {
		if (!IS_UART_HWFLOW_INSTANCE(UartInstance) ||
		    UART_CFG_FLOW_CTRL_RTS_CTS != cfg->flow_ctrl) {
			return -ENOTSUP;
		}
	}

	LL_USART_Disable(UartInstance);

	if (parity != uart_stm32_get_parity(dev)) {
		uart_stm32_set_parity(dev, parity);
	}

	if (stopbits != uart_stm32_get_stopbits(dev)) {
		uart_stm32_set_stopbits(dev, stopbits);
	}

	if (databits != uart_stm32_get_databits(dev)) {
		uart_stm32_set_databits(dev, databits);
	}

	if (flowctrl != uart_stm32_get_hwctrl(dev)) {
		uart_stm32_set_hwctrl(dev, flowctrl);
	}

	if (cfg->baudrate != data->baud_rate) {
		uart_stm32_set_baudrate(dev, cfg->baudrate);
		data->baud_rate = cfg->baudrate;
	}

	LL_USART_Enable(UartInstance);
	return 0;
};

static int uart_stm32_config_get(const struct device *dev,
				 struct uart_config *cfg)
{
	struct uart_stm32_data *data = DEV_DATA(dev);

	cfg->baudrate = data->baud_rate;
	cfg->parity = uart_stm32_ll2cfg_parity(uart_stm32_get_parity(dev));
	cfg->stop_bits = uart_stm32_ll2cfg_stopbits(
		uart_stm32_get_stopbits(dev));
	cfg->data_bits = uart_stm32_ll2cfg_databits(
		uart_stm32_get_databits(dev));
	cfg->flow_ctrl = uart_stm32_ll2cfg_hwctrl(
		uart_stm32_get_hwctrl(dev));
	return 0;
}

static int uart_stm32_poll_in(const struct device *dev, unsigned char *c)
{

#ifdef CONFIG_UART_ASYNC_API
    struct uart_stm32_data *data = DEV_DATA(dev);
    if (uart_stm32_resource_lock(&data->rx_buffer_lock) != 0)
        return -EBUSY;
#endif /* CONFIG_UART_ASYNC_API */

	USART_TypeDef *UartInstance = UART_STRUCT(dev);

	/* Clear overrun error flag */
	if (LL_USART_IsActiveFlag_ORE(UartInstance)) {
		LL_USART_ClearFlag_ORE(UartInstance);
	}

	if (!LL_USART_IsActiveFlag_RXNE(UartInstance)) {
		return -1;
	}

	*c = (unsigned char)LL_USART_ReceiveData8(UartInstance);

#ifdef CONFIG_UART_ASYNC_API
    uart_stm32_resource_unlock(&data->rx_buffer_lock);
#endif /* CONFIG_UART_ASYNC_API */

	return 0;
}

static void uart_stm32_poll_out(const struct device *dev,
					unsigned char c)
{
    #ifdef CONFIG_UART_ASYNC_API
    struct uart_stm32_data *data = DEV_DATA(dev);
    if (uart_stm32_resource_lock(&data->rx_buffer_lock) != 0)
        return;
    #endif /* CONFIG_UART_ASYNC_API */

	USART_TypeDef *UartInstance = UART_STRUCT(dev);

	/* Wait for TXE flag to be raised */
	while (!LL_USART_IsActiveFlag_TXE(UartInstance)) {
	}

	LL_USART_ClearFlag_TC(UartInstance);

	LL_USART_TransmitData8(UartInstance, (uint8_t)c);

    #ifdef CONFIG_UART_ASYNC_API
    uart_stm32_resource_unlock(&data->rx_buffer_lock);
    #endif /* CONFIG_UART_ASYNC_API */
}

static int uart_stm32_err_check(const struct device *dev)
{
	USART_TypeDef *UartInstance = UART_STRUCT(dev);

    uint32_t err = 0U;

	/* Check for errors, but don't clear them here.
	 * Some SoC clear all error flags when at least
	 * one is cleared. (e.g. F4X, F1X, and F2X)
	 */

    if (LL_USART_IsActiveFlag_NE(UartInstance)) {
        err |= UART_ERROR_NOISE;
    }

	if (LL_USART_IsActiveFlag_ORE(UartInstance)) {
		err |= UART_ERROR_OVERRUN;
	}

	if (LL_USART_IsActiveFlag_PE(UartInstance)) {
		err |= UART_ERROR_PARITY;
	}

	if (LL_USART_IsActiveFlag_FE(UartInstance)) {
		err |= UART_ERROR_FRAMING;
	}

	if (err & UART_ERROR_OVERRUN) {
		LL_USART_ClearFlag_ORE(UartInstance);
	}

	if (err & UART_ERROR_PARITY) {
		LL_USART_ClearFlag_PE(UartInstance);
	}

	if (err & UART_ERROR_FRAMING) {
		LL_USART_ClearFlag_FE(UartInstance);
	}

    if (err & UART_ERROR_NOISE) {
        LL_USART_ClearFlag_NE(UartInstance);
    }

	return err;
}

static inline void __uart_stm32_get_clock(const struct device *dev)
{
	struct uart_stm32_data *data = DEV_DATA(dev);
	const struct device *clk =
		device_get_binding(STM32_CLOCK_CONTROL_NAME);

	__ASSERT_NO_MSG(clk);

	data->clock = clk;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static int uart_stm32_fifo_fill(const struct device *dev,
				  const uint8_t *tx_data,
				  int size)
{
	USART_TypeDef *UartInstance = UART_STRUCT(dev);
	uint8_t num_tx = 0U;

	while ((size - num_tx > 0) &&
	       LL_USART_IsActiveFlag_TXE(UartInstance)) {
		/* TXE flag will be cleared with byte write to DR|RDR register */

		/* Send a character (8bit , parity none) */
		LL_USART_TransmitData8(UartInstance, tx_data[num_tx++]);
	}

	return num_tx;
}

static int uart_stm32_fifo_read(const struct device *dev, uint8_t *rx_data,
				  const int size)
{
	USART_TypeDef *UartInstance = UART_STRUCT(dev);
	uint8_t num_rx = 0U;

	while ((size - num_rx > 0) &&
	       LL_USART_IsActiveFlag_RXNE(UartInstance)) {
		/* RXNE flag will be cleared upon read from DR|RDR register */

		/* Receive a character (8bit , parity none) */
		rx_data[num_rx++] = LL_USART_ReceiveData8(UartInstance);

		/* Clear overrun error flag */
		if (LL_USART_IsActiveFlag_ORE(UartInstance)) {
			LL_USART_ClearFlag_ORE(UartInstance);
		}
	}

	return num_rx;
}

static void uart_stm32_irq_tx_enable(const struct device *dev)
{
	USART_TypeDef *UartInstance = UART_STRUCT(dev);

	LL_USART_EnableIT_TC(UartInstance);
}

static void uart_stm32_irq_tx_disable(const struct device *dev)
{
	USART_TypeDef *UartInstance = UART_STRUCT(dev);

	LL_USART_DisableIT_TC(UartInstance);
}

static int uart_stm32_irq_tx_ready(const struct device *dev)
{
	USART_TypeDef *UartInstance = UART_STRUCT(dev);

	return LL_USART_IsActiveFlag_TXE(UartInstance);
}

static int uart_stm32_irq_tx_complete(const struct device *dev)
{
	USART_TypeDef *UartInstance = UART_STRUCT(dev);

	return LL_USART_IsActiveFlag_TC(UartInstance);
}

static void uart_stm32_irq_rx_enable(const struct device *dev)
{
	USART_TypeDef *UartInstance = UART_STRUCT(dev);

	LL_USART_EnableIT_RXNE(UartInstance);
}

static void uart_stm32_irq_rx_disable(const struct device *dev)
{
	USART_TypeDef *UartInstance = UART_STRUCT(dev);

	LL_USART_DisableIT_RXNE(UartInstance);
}

static int uart_stm32_irq_rx_ready(const struct device *dev)
{
	USART_TypeDef *UartInstance = UART_STRUCT(dev);

	return LL_USART_IsActiveFlag_RXNE(UartInstance);
}

static void uart_stm32_irq_err_enable(const struct device *dev)
{
	USART_TypeDef *UartInstance = UART_STRUCT(dev);

    /* Enable FE, ORE interruptions */
	LL_USART_EnableIT_ERROR(UartInstance);
#if !defined(CONFIG_SOC_SERIES_STM32F0X) || defined(USART_LIN_SUPPORT)
	/* Enable Line break detection */
	if (IS_UART_LIN_INSTANCE(UartInstance)) {
		LL_USART_EnableIT_LBD(UartInstance);
	}
#endif
	/* Enable parity error interruption */
	LL_USART_EnableIT_PE(UartInstance);
}

static void uart_stm32_irq_err_disable(const struct device *dev)
{
	USART_TypeDef *UartInstance = UART_STRUCT(dev);

	/* Disable FE, ORE interruptions */
	LL_USART_DisableIT_ERROR(UartInstance);
#if !defined(CONFIG_SOC_SERIES_STM32F0X) || defined(USART_LIN_SUPPORT)
	/* Disable Line break detection */
	if (IS_UART_LIN_INSTANCE(UartInstance)) {
		LL_USART_DisableIT_LBD(UartInstance);
	}
#endif
	/* Disable parity error interruption */
	LL_USART_DisableIT_PE(UartInstance);
}

static int uart_stm32_irq_is_pending(const struct device *dev)
{
	USART_TypeDef *UartInstance = UART_STRUCT(dev);

	return ((LL_USART_IsActiveFlag_RXNE(UartInstance) &&
		 LL_USART_IsEnabledIT_RXNE(UartInstance)) ||
		(LL_USART_IsActiveFlag_TC(UartInstance) &&
		 LL_USART_IsEnabledIT_TC(UartInstance)));
}

static int uart_stm32_irq_update(const struct device *dev)
{
    UNUSED(dev);
	return 1;
}

static void uart_stm32_irq_callback_set(const struct device *dev,
					uart_irq_callback_user_data_t cb,
					void *cb_data)
{
	struct uart_stm32_data *data = DEV_DATA(dev);

	data->user_cb = cb;
	data->user_data = cb_data;
}

static void uart_stm32_isr(const struct device *dev)
{
	struct uart_stm32_data *data = DEV_DATA(dev);

    if (data->user_cb)
    {
		data->user_cb(dev, data->user_data);
	}

#ifdef CONFIG_UART_ASYNC_API
    USART_TypeDef *UartInstance = UART_STRUCT(dev);

    if (LL_USART_IsActiveFlag_TXE(UartInstance)){
        uart_stm32_tx_isr(dev, data, UartInstance);
    }

    if (LL_USART_IsActiveFlag_RXNE(UartInstance)){
        uart_stm32_rx_isr(dev, data, UartInstance);
    }

    if ((UartInstance->ISR &
            ( USART_ISR_ORE |
              USART_ISR_PE |
              USART_ISR_FE |
              USART_ISR_NE )) == 0 )
        return;

    int err = uart_stm32_err_check(dev);
    if (err != 0){
        if (err & UART_ERROR_PARITY)
            _uart_stm32_stop_with_error(dev, UART_ERROR_PARITY);
        if (err & UART_ERROR_FRAMING)
            _uart_stm32_stop_with_error(dev, UART_ERROR_FRAMING);
        if (err & UART_ERROR_OVERRUN)
            _uart_stm32_stop_with_error(dev, UART_ERROR_FRAMING);
        if (err & UART_ERROR_NOISE)
            _uart_stm32_stop_with_error(dev, UART_ERROR_FRAMING);
    }
#endif
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api uart_stm32_driver_api = {
#ifdef CONFIG_UART_ASYNC_API

    .callback_set = uart_stm32_callback_set,
    .tx = uart_stm32_tx,
    .tx_abort = uart_stm32_tx_abort,
    .rx_enable = uart_stm32_rx_enable,
    .rx_buf_rsp = uart_stm32_rx_buf_rsp,
    .rx_disable = uart_stm32_rx_disable,

#endif
	.poll_in = uart_stm32_poll_in,
	.poll_out = uart_stm32_poll_out,
	.err_check = uart_stm32_err_check,
	.configure = uart_stm32_configure,
	.config_get = uart_stm32_config_get,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_stm32_fifo_fill,
	.fifo_read = uart_stm32_fifo_read,
	.irq_tx_enable = uart_stm32_irq_tx_enable,
	.irq_tx_disable = uart_stm32_irq_tx_disable,
	.irq_tx_ready = uart_stm32_irq_tx_ready,
	.irq_tx_complete = uart_stm32_irq_tx_complete,
	.irq_rx_enable = uart_stm32_irq_rx_enable,
	.irq_rx_disable = uart_stm32_irq_rx_disable,
	.irq_rx_ready = uart_stm32_irq_rx_ready,
	.irq_err_enable = uart_stm32_irq_err_enable,
	.irq_err_disable = uart_stm32_irq_err_disable,
	.irq_is_pending = uart_stm32_irq_is_pending,
	.irq_update = uart_stm32_irq_update,
	.irq_callback_set = uart_stm32_irq_callback_set,
#endif	/* CONFIG_UART_INTERRUPT_DRIVEN */
};

/**
 * @brief Initialize UART channel
 *
 * This routine is called to reset the chip in a quiescent state.
 * It is assumed that this function is called only once per UART.
 *
 * @param dev UART device struct
 *
 * @return 0
 */
static int uart_stm32_init(const struct device *dev)
{
	const struct uart_stm32_config *config = DEV_CFG(dev);
	struct uart_stm32_data *data = DEV_DATA(dev);
	USART_TypeDef *UartInstance = UART_STRUCT(dev);
	uint32_t ll_parity;
	uint32_t ll_datawidth;
	int err;

	__uart_stm32_get_clock(dev);
	/* enable clock */
	if (clock_control_on(data->clock,
			(clock_control_subsys_t *)&config->pclken) != 0) {
		return -EIO;
	}

	/* Configure dt provided device signals when available */
	err = stm32_dt_pinctrl_configure(config->pinctrl_list,
					 config->pinctrl_list_size,
					 (uint32_t)UART_STRUCT(dev));
	if (err < 0) {
		return err;
	}

	LL_USART_Disable(UartInstance);

	/* TX/RX direction */
	LL_USART_SetTransferDirection(UartInstance,
				      LL_USART_DIRECTION_TX_RX);

	/* Determine the datawidth and parity. If we use other parity than
	 * 'none' we must use datawidth = 9 (to get 8 databit + 1 parity bit).
	 */
	if (config->parity == 2) {
		/* 8 databit, 1 parity bit, parity even */
		ll_parity = LL_USART_PARITY_EVEN;
		ll_datawidth = LL_USART_DATAWIDTH_9B;
	} else if (config->parity == 1) {
		/* 8 databit, 1 parity bit, parity odd */
		ll_parity = LL_USART_PARITY_ODD;
		ll_datawidth = LL_USART_DATAWIDTH_9B;
	} else {  /* Default to 8N0, but show warning if invalid value */
		if (config->parity != 0) {
			LOG_WRN("Invalid parity setting '%d'."
				"Defaulting to 'none'.", config->parity);
		}
		/* 8 databit, parity none */
		ll_parity = LL_USART_PARITY_NONE;
		ll_datawidth = LL_USART_DATAWIDTH_8B;
	}

	/* Set datawidth and parity, 1 start bit, 1 stop bit  */
	LL_USART_ConfigCharacter(UartInstance,
				 ll_datawidth,
				 ll_parity,
				 LL_USART_STOPBITS_1);

	if (config->hw_flow_control) {
		uart_stm32_set_hwctrl(dev, LL_USART_HWCONTROL_RTS_CTS);
	}

	/* Set the default baudrate */
	uart_stm32_set_baudrate(dev, data->baud_rate);

	LL_USART_Enable(UartInstance);

#ifdef USART_ISR_TEACK
	/* Wait until TEACK flag is set */
	while (!(LL_USART_IsActiveFlag_TEACK(UartInstance))) {
	}
#endif /* !USART_ISR_TEACK */

#ifdef USART_ISR_REACK
	/* Wait until REACK flag is set */
	while (!(LL_USART_IsActiveFlag_REACK(UartInstance))) {
	}
#endif /* !USART_ISR_REACK */

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	config->uconf.irq_config_func(dev);
#endif

#ifdef CONFIG_UART_ASYNC_API
    data->tx_buffer_lock = UNLOCKED;
    data->rx_buffer_lock = UNLOCKED;
    data->rx_secondary_buffer_lock = UNLOCKED;
    data->async_callback = NULL;
    _uart_stm32_reset_rx_buffer(data);
#endif
	return 0;
}


#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define STM32_UART_IRQ_HANDLER_DECL(index)                                      \
	static void uart_stm32_irq_config_func_##index(const struct device *dev)
#define STM32_UART_IRQ_HANDLER_FUNC(index)                                      \
	.irq_config_func = uart_stm32_irq_config_func_##index,
#define STM32_UART_IRQ_HANDLER(index)                                           \
static void uart_stm32_irq_config_func_##index(const struct device *dev)        \
{                                                                               \
    UNUSED(dev);                                                                \
    IRQ_CONNECT(DT_INST_IRQN(index),                                            \
        DT_INST_IRQ(index, priority),                                           \
        uart_stm32_isr, DEVICE_GET(uart_stm32_##index),                         \
        0);                                                                     \
    irq_enable(DT_INST_IRQN(index));                                            \
}
#else
#define STM32_UART_IRQ_HANDLER_DECL(index)
#define STM32_UART_IRQ_HANDLER_FUNC(index)
#define STM32_UART_IRQ_HANDLER(index)
#endif

#ifdef CONFIG_UART_ASYNC_API
#define STM32_UART_ASYNC_TIMERS_DECL(index)                                     \
static void uart_stm32_timer_rx_callback_##index(struct k_timer *timer);        \
static void uart_stm32_timer_tx_callback_##index(struct k_timer *timer);        \
K_TIMER_DEFINE(uart_stm32_timer_rx_##index,                                     \
                   uart_stm32_timer_rx_callback_##index, NULL);                 \
K_TIMER_DEFINE(uart_stm32_timer_tx_##index,                                     \
                   uart_stm32_timer_tx_callback_##index, NULL);
#define STM32_UART_ASYNC_TIMERS_IMPL(index)                                     \
static void uart_stm32_timer_rx_callback_##index(struct k_timer *timer) {       \
    uart_stm32_rx_timeout(timer, DEVICE_GET(uart_stm32_##index));               \
};                                                                              \
static void uart_stm32_timer_tx_callback_##index(struct k_timer *timer) {       \
    uart_stm32_tx_timeout(timer, DEVICE_GET(uart_stm32_##index));               \
};
#define STM32_UART_ASYNC_DATA_INIT(index)                                       \
    .rx_timer = &uart_stm32_timer_rx_##index,                                   \
    .tx_timer = &uart_stm32_timer_rx_##index
#else
#define STM32_UART_ASYNC_TIMERS_DECL(index)
#define STM32_UART_ASYNC_DATA_INIT(index)
#define STM32_UART_ASYNC_TIMERS_IMPL(index)
#endif


#define STM32_UART_INIT(index)                                                  \
STM32_UART_IRQ_HANDLER_DECL(index);                                             \
STM32_UART_ASYNC_TIMERS_DECL(index)                                             \
                                                                                \
static const struct soc_gpio_pinctrl uart_pins_##index[] =                      \
                ST_STM32_DT_INST_PINCTRL(index, 0);                             \
                                                                                \
static const struct uart_stm32_config uart_stm32_cfg_##index = {                \
    .uconf = {                                                                  \
        .base = (uint8_t *)DT_INST_REG_ADDR(index),                             \
        STM32_UART_IRQ_HANDLER_FUNC(index)                                      \
    },                                                                          \
    .pclken = { .bus = DT_INST_CLOCKS_CELL(index, bus),                         \
            .enr = DT_INST_CLOCKS_CELL(index, bits)                             \
    },                                                                          \
    .hw_flow_control = DT_INST_PROP(index, hw_flow_control),                    \
    .parity = DT_INST_PROP_OR(index, parity, UART_CFG_PARITY_NONE),             \
    .pinctrl_list = uart_pins_##index,                                          \
    .pinctrl_list_size = ARRAY_SIZE(uart_pins_##index),                         \
};                                                                              \
                                                                                \
static struct uart_stm32_data uart_stm32_data_##index = {                       \
    .baud_rate = DT_INST_PROP(index, current_speed),                            \
    STM32_UART_ASYNC_DATA_INIT(index)                                           \
};                                                                              \
                                                                                \
DEVICE_AND_API_INIT(uart_stm32_##index, DT_INST_LABEL(index),                   \
            &uart_stm32_init,                                                   \
            &uart_stm32_data_##index, &uart_stm32_cfg_##index,                  \
            PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                   \
            &uart_stm32_driver_api);                                            \
                                                                                \
STM32_UART_ASYNC_TIMERS_IMPL(index)                                             \
STM32_UART_IRQ_HANDLER(index)

DT_INST_FOREACH_STATUS_OKAY(STM32_UART_INIT)
