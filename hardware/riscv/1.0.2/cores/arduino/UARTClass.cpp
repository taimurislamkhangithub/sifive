/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "UARTClass.h"
#include "variant.h"


UARTClass Serial;
UARTClass Serial1;


void UARTClass::init(uint32_t base)
{
  UBRD = (uint32_t *)(base + UART_REG_BRDL);
  UIER = (uint32_t *)(base + UART_REG_IER);
  UIIR = (uint32_t *)(base + UART_REG_IIR);
  UFCR = (uint32_t *)(base + UART_REG_FCR);
  ULCR = (uint32_t *)(base + UART_REG_LCR);
  ULSR = (uint32_t *)(base + UART_REG_LSR);
  UWRC = (uint32_t *)(base + UART_WR_CH);
}

int UARTClass::sio_probe_rx()
{
  
    /* Check for characters in UART Receive FIFO */
    if ((UART_LSR_DR & (*ULSR)) == 0)
    {
        return (0);
    }
    return (1);

}

int UARTClass::sio_getchar(int blocking)
{
  int c, busy;

  while (blocking && !sio_probe_rx());
  c = *UWRC;
  return c;
}

int UARTClass::sio_putchar(char c, int blocking)
{

  if (blocking){
  /* Check for space in UART FIFO */
    while((*ULSR & UART_LSR_THRE_BIT) == 0);
  }
  // write char
  *UWRC = c;
  return 0;
}

/*
 * Set RS-232 baudrate.  Works well with FT-232R from 9600 to 1000000 bauds.
 */
void UARTClass::sio_setbaud(int bauds)
{
  *UBRD = (F_CPU / bauds) / 16;
}

// Public Methods //////////////////////////////////////////////////////////////

void UARTClass::begin(unsigned long bauds)
{

  /* SET LSR to be 1's so Whisper will be happy that ch is ready */
  *ULSR = 0xff;
  /* Set DLAB bit in LCR */
  *ULCR |= UART_DLAB_BIT;
  /* Set divisor regs  devisor = 27: clock_freq/baud_rate*16 -->> clock = 50MHz, baud=115200*/
  *UBRD = (F_CPU / bauds) / 16;
  /* 8 data bits, 1 stop bit, no parity, clear DLAB */
  *ULCR = (UART_LCR_CS8 | UART_LCR_1_STB | UART_LCR_PDIS);
  *UFCR = (UART_FCR_FIFO_BIT | UART_FCR_MODE0_BIT | UART_FCR_FIFO_8_BIT | UART_FCR_RCVRCLR_BIT | UART_FCR_XMITCLR_BIT);
  /* disable interrupts  */
  *UIER = (0x00);

}

void UARTClass::end(void)
{
}

// int UARTClass::available(void)
// {
//   return sio_probe_rx();
// }

int UARTClass::available(void)
{
      if (sio_probe_rx())
    {
      int c = sio_getchar(1);
      if (c != -1) {
          sio_receive(c);
      }
    }

    int availableInBuffer = (sio_rxbuf_head - sio_rxbuf_tail + RX_BUF_SIZE) % RX_BUF_SIZE;

    // If there are characters in the buffer, return the count
    if (availableInBuffer > 0) {
        return availableInBuffer;
    }
    else
    {
      return 0;
    }

}

int UARTClass::availableForWrite(void)
{
  int busy;
  busy = (!(*ULSR & UART_LSR_THRE_BIT));
  return (!busy);
}

int UARTClass::peek(void)
{
  sio_probe_rx();
  if (sio_rxbuf_tail == sio_rxbuf_head)
    return (-1);
  else
    return (sio_rxbuf[sio_rxbuf_tail]);
}

// int UARTClass::read(void)
// {
//   return (sio_getchar(1));
// }

int UARTClass::read(void)
{
    sio_probe_rx();

    // Check if there's a character in the buffer
    if (sio_rxbuf_tail != sio_rxbuf_head) {
        int c = sio_rxbuf[sio_rxbuf_tail];
        sio_rxbuf_tail = (sio_rxbuf_tail + 1) % RX_BUF_SIZE;
        return c;
    }
    else{
      return 0;
    }
}

void UARTClass::flush(void)
{
  /* Check for space in UART FIFO */
    while((*ULSR & UART_LSR_THRE_BIT) == 0);
}

size_t
UARTClass::write(uint8_t uc_data)
{
  sio_putchar(uc_data, 1);
  return (1);
}

// Add received characters to the buffer
void UARTClass::sio_receive(char c)
{
    // Check if the buffer is not full
    if ((sio_rxbuf_head + 1) % RX_BUF_SIZE != sio_rxbuf_tail) {
        sio_rxbuf[sio_rxbuf_head] = c;
        sio_rxbuf_head = (sio_rxbuf_head + 1) % RX_BUF_SIZE;
    }
}