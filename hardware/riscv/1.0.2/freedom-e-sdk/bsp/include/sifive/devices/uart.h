// See LICENSE for license details.

#ifndef _SIFIVE_UART_H
#define _SIFIVE_UART_H

// /* Register offsets */
#define UART_REG_TXFIFO         0x00
#define UART_REG_RXFIFO         0x04
#define UART_REG_TXCTRL         0x08
#define UART_REG_RXCTRL         0x0c
#define UART_REG_IP             0x14
#define UART_REG_DIV            0x18

/* TXCTRL register */
#define UART_TXEN               0x1
#define UART_TXWM(x)            (((x) & 0xffff) << 16)

/* RXCTRL register */
#define UART_RXEN               0x1
#define UART_RXWM(x)            (((x) & 0xffff) << 16)

/* IP register */
#define UART_IP_TXWM            0x1
#define UART_IP_RXWM            0x2

/* Register offsets */
#define UART_REG_BRDL           0x00 /* Baud rate divisor (LSB)        */  
#define UART_REG_IER            0x04 /* Interrupt enable reg.          */
#define UART_REG_FCR            0x08 /* FIFO control reg.              */
#define UART_REG_LCR            0x0C /* Line control reg.              */
#define UART_REG_LSR            0x14 /* Line control reg.              */
#define UART_WR_CH              0x00 /* Write FIFO                     */

#define UART_BAUD_RATE       (115200)
#define UART_LCR_CS8         (0x03)  /* 8 bits data size */
#define UART_LCR_1_STB       (0x00)  /* 1 stop bit */
#define UART_LCR_PDIS        (0x00)  /* parity disable */

#define UART_LSR_THRE_BIT    (0x20)
#define UART_LSR_DR          (0x00)
#define UART_LSR_TFE         (0x06)
#define UART_FCR_FIFO_BIT    (0x01)  /* enable XMIT and RCVR FIFO */
#define UART_FCR_RCVRCLR_BIT (0x02)  /* clear RCVR FIFO */
#define UART_FCR_XMITCLR_BIT (0x04)  /* clear XMIT FIFO */
#define UART_FCR_MODE0_BIT   (0x00)  /* set receiver in mode 0 */
#define UART_FCR_MODE1_BIT   (0x08)  /* set receiver in mode 1 */
#define UART_FCR_FIFO_8_BIT  (0x80)  /* 8 bytes in RCVR FIFO */
#define UART_DLAB_BIT        (0x80)  /* DLAB bit in LCR */


#endif /* _SIFIVE_UART_H */
