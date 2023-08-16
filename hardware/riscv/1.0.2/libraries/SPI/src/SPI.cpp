/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@arduino.cc>
 * Copyright (c) 2014 by Paul Stoffregen <paul@pjrc.com> (Transaction API)
 * Copyright (c) 2014 by Matthijs Kooijman <matthijs@stdin.nl> (SPISettings AVR)
 * Copyright (c) 2014 by Andrew J. Kroll <xxxajk@gmail.com> (atomicity fixes)
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#include "SPI.h"



SPIClass SPI;
//extern UARTClass Serial;

void SPIClass::begin()
{

    Serial.print("BEGIN");
    SPCR = 0x10;
    SPER= 0x00;
    SPSR = 0x05;
    SPCR = 0x00;
  // uint8_t sreg = SREG;
  // noInterrupts(); // Protect from a scheduler and prevent transactionBegin
  // if (!initialized) {
  //   // Set SS to high so a connected chip will be "deselected" by default
  //   uint8_t port = digitalPinToPort(SS);
  //   uint8_t bit = digitalPinToBitMask(SS);
  //   volatile uint8_t *reg = portModeRegister(port);

  //   // if the SS pin is not already configured as an output
  //   // then set it high (to enable the internal pull-up resistor)
  //   if(!(*reg & bit)){
  //     digitalWrite(SS, HIGH);
  //   }

  //   // When the SS pin is set as OUTPUT, it can be used as
  //   // a general purpose output port (it doesn't influence
  //   // SPI operations).
  //   pinMode(SS, OUTPUT);

  //   // Warning: if the SS pin ever becomes a LOW INPUT then SPI
  //   // automatically switches to Slave, so the data direction of
  //   // the SS pin MUST be kept as OUTPUT.
  //   SPCR |= _BV(MSTR);
  //   SPCR |= _BV(SPE);

  //   // Set direction register for SCK and MOSI pin.
  //   // MISO pin automatically overrides to INPUT.
  //   // By doing this AFTER enabling SPI, we avoid accidentally
  //   // clocking in a single bit since the lines go directly
  //   // from "input" to SPI control.
  //   // http://code.google.com/p/arduino/issues/detail?id=888
  //   pinMode(SCK, OUTPUT);
  //   pinMode(MOSI, OUTPUT);
  // }
  // initialized++; // reference count
  // SREG = sreg;
}

void SPIClass::end() {
  // uint8_t sreg = SREG;
  // noInterrupts(); // Protect from a scheduler and prevent transactionBegin
  // // Decrease the reference counter
  // if (initialized)
  //   initialized--;
  // // If there are no more references disable SPI
  // if (!initialized) {
     SPCR &= ~SPE;
  //   interruptMode = 0;
  //   #ifdef SPI_TRANSACTION_MISMATCH_LED
  //   inTransactionFlag = 0;
  //   #endif
  // }
  // SREG = sreg;
}



void SPIClass::usingInterrupt(uint8_t interruptNumber)
{
  // uint8_t mask = 0;
  // uint8_t sreg = SREG;
  // noInterrupts(); // Protect from a scheduler and prevent transactionBegin
  // switch (interruptNumber) {
  // #ifdef SPI_INT0_MASK
  // case 0: mask = SPI_INT0_MASK; break;
  // #endif
  // #ifdef SPI_INT1_MASK
  // case 1: mask = SPI_INT1_MASK; break;
  // #endif
  // #ifdef SPI_INT2_MASK
  // case 2: mask = SPI_INT2_MASK; break;
  // #endif
  // #ifdef SPI_INT3_MASK
  // case 3: mask = SPI_INT3_MASK; break;
  // #endif
  // #ifdef SPI_INT4_MASK
  // case 4: mask = SPI_INT4_MASK; break;
  // #endif
  // #ifdef SPI_INT5_MASK
  // case 5: mask = SPI_INT5_MASK; break;
  // #endif
  // #ifdef SPI_INT6_MASK
  // case 6: mask = SPI_INT6_MASK; break;
  // #endif
  // #ifdef SPI_INT7_MASK
  // case 7: mask = SPI_INT7_MASK; break;
  // #endif
  // default:
  //   interruptMode = 2;
  //   break;
  // }
  // interruptMask |= mask;
  // if (!interruptMode)
  //   interruptMode = 1;
  // SREG = sreg;
}

void SPIClass::notUsingInterrupt(uint8_t interruptNumber)
{
  // // Once in mode 2 we can't go back to 0 without a proper reference count
  // if (interruptMode == 2)
  //   return;
  // uint8_t mask = 0;
  // uint8_t sreg = SREG;
  // noInterrupts(); // Protect from a scheduler and prevent transactionBegin
  // switch (interruptNumber) {
  // #ifdef SPI_INT0_MASK
  // case 0: mask = SPI_INT0_MASK; break;
  // #endif
  // #ifdef SPI_INT1_MASK
  // case 1: mask = SPI_INT1_MASK; break;
  // #endif
  // #ifdef SPI_INT2_MASK
  // case 2: mask = SPI_INT2_MASK; break;
  // #endif
  // #ifdef SPI_INT3_MASK
  // case 3: mask = SPI_INT3_MASK; break;
  // #endif
  // #ifdef SPI_INT4_MASK
  // case 4: mask = SPI_INT4_MASK; break;
  // #endif
  // #ifdef SPI_INT5_MASK
  // case 5: mask = SPI_INT5_MASK; break;
  // #endif
  // #ifdef SPI_INT6_MASK
  // case 6: mask = SPI_INT6_MASK; break;
  // #endif
  // #ifdef SPI_INT7_MASK
  // case 7: mask = SPI_INT7_MASK; break;
  // #endif
  // default:
  //   break;
  //   // this case can't be reached
  // }
  // interruptMask &= ~mask;
  // if (!interruptMask)
  //   interruptMode = 0;
  // SREG = sreg;
}


// /*
//  * Copyright (c) 2010 by Cristian Maglie <c.maglie@arduino.cc>
//  * Copyright (c) 2014 by Paul Stoffregen <paul@pjrc.com> (Transaction API)
//  * SPI Master library for arduino.
//  *
//  * This file is free software; you can redistribute it and/or modify
//  * it under the terms of either the GNU General Public License version 2
//  * or the GNU Lesser General Public License version 2.1, both as
//  * published by the Free Software Foundation.
//  */
// // heyyyyyyyyyyy mariaa
// #include "SPI.h"
// #define TWO 2
// #define THREE 3
// ////////////////////////////////////////////////////////////////////////////////
// /* Define some constants for read-write access. */


// #define SPI_TRANSMIT_FAIL 0x31
// #define SPI_TRANSMIT_SUCCESS 0x30
// #define READ_Reg(dir) (*(volatile unsigned *)dir)
// #define WRITE_Reg(dir,val) ((*(volatile unsigned *)dir) = (val))

// #define OC_SPI_SPCR(addr) (addr + (8*0x00) )
// #define OC_SPI_SPSR(addr) (addr + (8*0x01) )                                                                        	                                                                         	
// #define OC_SPI_SPDR(addr) (addr + (8*0x02) )                                                                          	
// #define OC_SPI_SPER(addr) (addr + (8*0x03) )                                                                          
// #define OC_SPI_SPCS(addr) (addr + (8*0x04) )   

// /* ----- Control register                                                                                                                       */

// /* SPI clock Rate                                                                                               */
// #define OC_SPI_SPR (1 << 0)

// // #define OC_SPI_RES1 (1 << 1)
// /* Clock Phase: Determines the phase of sampling and sending data.                                              */
// #define OC_SPI_CPHA (1 << 2)
// /* Clock Polarity: Determines idle state of SPI clock (SCK)                                                     */
// #define OC_SPI_CPOL (1 << 3)
// /* When MSTR = 1, the SPI core is a controller device                                                           */
// #define OC_SPI_MSTR (1 << 4)
// /* When SPE = 1, the SPI core is enabled.                                                                       */
// // #define OC_SPI_RES2 (1 << 5)
// #define OC_SPI_SPE (1 << 6)
// /* When SPIE = 1, when the SPI Interrupt Flag in the status register is set, the host is interrupted            */
// #define OC_SPI_SPIE (1 << 7)

// /* ----- Status register bits                                                                                                                   */

// /* Read FIFO Empty:       #define SPI_MODE0 0x00
// #define SPI_MODE1 0x01
// #define SPI_MODE2 0x02
// #define SPI_MODE3 0x03  If RFEMPTY = 1, the read FIFO is empty.                                         */
// #define OC_SPI_RFEMPTY (1 << 0)
// /* Read FIFO Full:          If RFFULL = 1, the read FIFO is full.                                           */
// #define OC_SPI_RFFULL (1 << 1)
// /* Write FIFO Empty:        IF WFEMPTY = 1, the write FIFO is empty                                         */
// #define OC_SPI_WFEMPTY (1 << 2)
// /* Write FIFO Full:         IF WFFULL = 1, the write FIFO is full.                                          */
// #define OC_SPI_WFFULL (1 << 3)
// /* Write Collision flag:    When WCOL = 1, the SPDATA register was written to while the Write FIFO was full */
// #define OC_SPI_WCOL (1 << 6)
// /* SPI Interrupt Flag:      SPIF = 1 upon completion of a transfer block         */
// #define OC_SPI_SPIF (1 << 7)

// /* ----- SPI Extended Register                                        */

// /* Extended SPI Clock Rate Select: Add two bits to the SPR (SPI Clock Rate Select).                         */
// #define OC_SPI_ESPR (3 << 0)
// /* Interrupt Count: Determine the transfer block size. The SPIF bit is set after ICNT transfers             */
// #define OC_SPI_ICNT (3 << 6)

// #define OC_BITSET(reg,bitmask)      ((*(volatile unsigned *)reg) |= (bitmask))
// #define OC_BITCLEAR(reg,bitmask)    ((*(volatile unsigned *)reg) &= (~(bitmask)))
// #define OC_ISSET(reg,bitmask)       ((READ_Reg(reg))&(bitmask))
// #define OC_ISCLEAR(reg,bitmask)     (!(OC_ISSET(reg,bitmask)))

// #define  read_csr(csr) ({ unsigned long __tmp; \
//   asm volatile ("csrr %0, " #csr : "=r"(__tmp)); \
//   __tmp; })


// int32_t HAL_GetTick()
// {
//   return millis();
// }
// //////////////////////////////////////////////////////////////////////////////////////////////


// SPIClass::SPIClass(uint32_t _id) :
//   id(_id)
// {
// 	// Empty
// }

// // void SPIClass::begin() {
  
// //   GPIO_REG(GPIO_IOF_SEL) &= ~SPI_IOF_MASK;
// //   GPIO_REG(GPIO_IOF_EN)  |= SPI_IOF_MASK;

// //   //setClockDivider(F_CPU/1000000);
// //   setDataMode(SPI_MODE0);
// //   setBitOrder(MSBFIRST);
  
// // }

// // specifies chip select pin to attach to hardware SPI interface
// void SPIClass::begin(uint8_t _pin) {
  	
//   //       // enable CS pin for selected channel/pin
//   //       uint32_t iof_mask = digitalPinToBitMask(_pin);
//   //       GPIO_REG(GPIO_IOF_SEL)  &= ~iof_mask;
//   //       GPIO_REG(GPIO_IOF_EN)   |=  iof_mask;

// 	// // Default speed set to ~1Mhz
// 	// //setClockDivider(_pin, F_CPU/1000000);
// 	// setDataMode(_pin, SPI_MODE0);
// 	// setBitOrder(_pin, MSBFIRST);

// 	// this->begin();

//       // 8'b0000 1001 indicates seial peripheral interrupt and master mode of spi is selected
//     WRITE_Reg(OC_SPI_SPCR(address), 0x10);
//     // Set extension register to 0x00
//     // SPIF is set after every completed transfer -- Default Settting
//     WRITE_Reg(OC_SPI_SPER(address), 0x00);
//     // Set status register to 0x05
//     // Interrpt flag is enabled -- Default settings
//     WRITE_Reg(OC_SPI_SPSR(address), 0x05);
//     // Set chip select register to 0x00
//     WRITE_Reg(OC_SPI_SPCS(address), 0x00);
//     // Return 1 to indicate success

// }

// void SPIClass:: begin()
// { // Set control register to 0x10
//     // 8'b0000 1001 indicates seial peripheral interrupt and master mode of spi is selected
//     WRITE_Reg(OC_SPI_SPCR(address), 0x10);
//     // Set extension register to 0x00
//     // SPIF is set after every completed transfer -- Default Settting
//     WRITE_Reg(OC_SPI_SPER(address), 0x00);
//     // Set status register to 0x05
//     // Interrpt flag is enabled -- Default settings
//     WRITE_Reg(OC_SPI_SPSR(address), 0x05);
//     // Set chip select register to 0x00
//     WRITE_Reg(OC_SPI_SPCS(address), 0x00);
//     // Return 1 to indicate success
// }

// void SPIClass::usingInterrupt(uint8_t interruptNumber)
// {
// }

// // start an SPI transaction using specified SPIsettings
// void SPIClass::beginTransaction(SPISettings settings)
// {
//   // before starting a transaction, set SPI peripheral to desired mode

//   SPI_REG(SPI_REG_FMT) = SPI_FMT_PROTO(SPI_PROTO_S) |
//     SPI_FMT_ENDIAN((settings.border == LSBFIRST) ? SPI_ENDIAN_LSB : SPI_ENDIAN_MSB) |
//     SPI_FMT_DIR(SPI_DIR_RX) |
//     SPI_FMT_LEN(8);
  
//   SPI_REG(SPI_REG_SCKDIV)  = settings.sckdiv;

//   SPI_REG(SPI_REG_SCKMODE) = settings.sckmode;

//   // We Don't control CS, so this setting doesn't matter.
//   //SPI_REG(SPI_REG_CSDEF)   = 0xFFFF;

// }


// // start an SPI transaction using specified CS pin and SPIsettings
// void SPIClass::beginTransaction(uint8_t pin, SPISettings settings)
// {
  
//   // before starting a transaction, set SPI peripheral to desired mode
//   SPI_REG(SPI_REG_CSID)   = SS_PIN_TO_CS_ID(pin); 
//   SPI_REG(SPI_REG_CSMODE) = SPI_CSMODE_HOLD;

//   // There is no way here to change the CS polarity.
//   SPI_REG(SPI_REG_CSDEF)   = 0xFFFF;

//   this->beginTransaction(settings);

// }

// void SPIClass::endTransaction(void) {
//   SPI_REG(SPI_REG_CSMODE) = SPI_CSMODE_AUTO;
// }

// void SPIClass::end(uint8_t _pin) {
//   GPIO_REG(GPIO_IOF_EN)  &= ~digitalPinToBitMask(_pin);    
// }

// void SPIClass::end() {
//   GPIO_REG(GPIO_IOF_EN)  &= ~SPI_IOF_MASK;
// }

// void SPIClass::setBitOrder(BitOrder _bitOrder) {
//   SPI_REG(SPI_REG_FMT) = SPI_FMT_PROTO(SPI_PROTO_S) |
//     SPI_FMT_ENDIAN((_bitOrder == LSBFIRST) ? SPI_ENDIAN_LSB : SPI_ENDIAN_MSB) |
//     SPI_FMT_DIR(SPI_DIR_RX) |
//     SPI_FMT_LEN(8);
// }

// void SPIClass::setBitOrder(uint8_t _pin, BitOrder _bitOrder) {
// 	uint32_t ch = SS_PIN_TO_CS_ID(_pin);
// 	bitOrder[ch] = _bitOrder;
// 	// This gets used later?
// }

// // void SPIClass::setDataMode(uint8_t _mode) {
// //   SPI_REG(SPI_REG_SCKMODE) = _mode;
// // }

// void SPIClass::setDataMode(uint8_t _pin, uint8_t _mode) {
// 	uint32_t ch = SS_PIN_TO_CS_ID(_pin);
// 	mode[ch] = _mode;
// 	// This gets used later?
// }


// void SPIClass::setDataMode(uint8_t _mode)
// { 
//     /* Set SPI control register with desired configuration (no interrupts, core enabled,
//     reserved, controller, cpol=0, cha=0, clock divisor 11 for 4096)*/
//     switch (_mode)
//     {
//     case 0:
//         OC_BITSET(OC_SPI_SPCR(address),OC_SPI_SPR);
//         OC_BITCLEAR(OC_SPI_SPCR(address), OC_SPI_CPHA);
//         OC_BITCLEAR(OC_SPI_SPCR(address), OC_SPI_CPOL);
//         OC_BITSET(OC_SPI_SPCR(address),OC_SPI_MSTR);
//         OC_BITSET(OC_SPI_SPCR(address),OC_SPI_SPE);
//         OC_BITCLEAR(OC_SPI_SPCR(address),OC_SPI_SPIE);

//         break;
//     case 1:
//         OC_BITSET(OC_SPI_SPCR(address),OC_SPI_SPR);
//         OC_BITCLEAR(OC_SPI_SPCR(address), OC_SPI_CPHA);
//         OC_BITSET(OC_SPI_SPCR(address), OC_SPI_CPOL);
//         OC_BITSET(OC_SPI_SPCR(address),OC_SPI_MSTR);
//         OC_BITSET(OC_SPI_SPCR(address),OC_SPI_SPE);
//         OC_BITCLEAR(OC_SPI_SPCR(address),OC_SPI_SPIE);
//         break;
//     case 2:
//         OC_BITSET(OC_SPI_SPCR(address),OC_SPI_SPR);
//         OC_BITSET(OC_SPI_SPCR(address), OC_SPI_CPHA);
//         OC_BITCLEAR(OC_SPI_SPCR(address), OC_SPI_CPOL);
//         OC_BITSET(OC_SPI_SPCR(address),OC_SPI_MSTR);
//         OC_BITSET(OC_SPI_SPCR(address),OC_SPI_SPE);
//         OC_BITCLEAR(OC_SPI_SPCR(address),OC_SPI_SPIE);
//         break;
//     case 3:
//         OC_BITSET(OC_SPI_SPCR(address),OC_SPI_SPR);
//         OC_BITSET(OC_SPI_SPCR(address), OC_SPI_CPHA);
//         OC_BITSET(OC_SPI_SPCR(address), OC_SPI_CPOL);
//         OC_BITSET(OC_SPI_SPCR(address),OC_SPI_MSTR);
//         OC_BITSET(OC_SPI_SPCR(address),OC_SPI_SPE);
//         OC_BITCLEAR(OC_SPI_SPCR(address),OC_SPI_SPIE);
//         break;
//     default:
//         OC_BITSET(OC_SPI_SPCR(address),OC_SPI_SPR);
//         OC_BITCLEAR(OC_SPI_SPCR(address), OC_SPI_CPHA);
//         OC_BITCLEAR(OC_SPI_SPCR(address), OC_SPI_CPOL);
//         OC_BITSET(OC_SPI_SPCR(address),OC_SPI_MSTR);
//         OC_BITSET(OC_SPI_SPCR(address),OC_SPI_SPE);
//         OC_BITCLEAR(OC_SPI_SPCR(address),OC_SPI_SPIE);
//         break;
//     }
//     // // Set SPI event register with desired configuration (interrupt count 00 (7:6), clock divisor 10 (1:0) for 4096)
//     WRITE_Reg(OC_SPI_SPER(address), 0x01);
//     // Return success status
// }


// void SPIClass::setClockDivider(uint8_t _divider) {
//   SPI_REG(SPI_REG_SCKDIV) = _divider;
// }

// void SPIClass::setClockDivider(uint8_t _pin, uint8_t _divider) {
//   uint32_t ch = SS_PIN_TO_CS_ID(_pin);
//   divider[ch] = _divider;
// }


// // byte SPIClass::transfer(uint8_t _data, SPITransferMode _mode) {

// //   // SPI_Write(spi, _channel, _data);
// //   while (SPI_REG(SPI_REG_TXFIFO) & SPI_TXFIFO_FULL) ;
// //   SPI_REG(SPI_REG_TXFIFO) = _data;
  
// //   // return SPI_Read(spi);
// //   volatile int32_t x;
// //   while ((x = SPI_REG(SPI_REG_RXFIFO)) & SPI_RXFIFO_EMPTY);
// //   return x & 0xFF;
  
// //   if (_mode == SPI_LAST) {
// //     SPI_REG(SPI_REG_CSMODE) = SPI_CSMODE_AUTO;
// //   }
  
// // }

// byte SPIClass::transfer(byte _pin, uint8_t _data, SPITransferMode _mode) {

//   // No need to do anything with the pin, because that was already
//   // set up earlier.
//   return this->transfer(_data, _mode);
 
// }

// uint16_t SPIClass::transfer16(byte _pin, uint16_t _data, SPITransferMode _mode) {
// 	union { uint16_t val; struct { uint8_t lsb; uint8_t msb; }; } t;
// 	uint32_t ch = SS_PIN_TO_CS_ID(_pin);

// 	t.val = _data;

// 	if (bitOrder[ch] == LSBFIRST) {
// 		t.lsb = transfer(_pin, t.lsb, SPI_CONTINUE);
// 		t.msb = transfer(_pin, t.msb, _mode);
// 	} else {
// 		t.msb = transfer(_pin, t.msb, SPI_CONTINUE);
// 		t.lsb = transfer(_pin, t.lsb, _mode);
// 	}

// 	return t.val;
// }

// void SPIClass::transfer(byte _pin, void *_buf, size_t _count, SPITransferMode _mode) {
  
//   if (_count == 0)
//     return;
  
//   uint8_t *buffer = (uint8_t *)_buf;
//   if (_count == 1) {
//     *buffer = transfer(_pin, *buffer, _mode);
//     return;
//   }

//   // Send the first byte
//   while (SPI_REG(SPI_REG_TXFIFO) < 0) ;
//   SPI_REG(SPI_REG_TXFIFO) = *buffer;

//   volatile int32_t x;
//   uint8_t r,d;
//   while (_count > 1) {
//     // Prepare next byte
//     d = *(buffer+1);
//     // Read transferred byte and send next one straight away
//     while ((x = (SPI_REG(SPI_REG_RXFIFO)) & SPI_RXFIFO_EMPTY))
//       ;
//     r = x & 0xFF;
//     while (SPI_REG(SPI_REG_TXFIFO) & SPI_TXFIFO_FULL);
//     SPI_REG(SPI_REG_TXFIFO) = d;

// 		// Save read byte
// 		*buffer = r;
// 		buffer++;
// 		_count--;
// 	}

// 	// Receive the last transferred byte
//   while ((x = (SPI_REG(SPI_REG_RXFIFO)) & SPI_RXFIFO_EMPTY))
//     ;
//   r = x & 0xFF;
//   *buffer = r;
// }

// //char SPIClass::transfer(int pin, char pData, uint16_t Size, uint32_t Timeout)
// byte SPIClass::transfer(uint8_t _data, SPITransferMode _mode)
// {   
//     uint32_t Timeout = 200;
//     byte b = 1;
//     uint32_t current_millis = HAL_GetTick();
//     // Set interrupt
//     OC_BITSET(OC_SPI_SPSR(address), OC_SPI_SPIF);
//     // Write data to SPI data register
//     WRITE_Reg(OC_SPI_SPDR(address), _data);

//     // Wait for interrupt flag to be set
//     while (OC_ISCLEAR(OC_SPI_SPSR(address), OC_SPI_SPIF))
//     {
//         if ((HAL_GetTick() - current_millis) > Timeout)
//         {
//             OC_SPI_SPDR(address);
//             return SPI_TRANSMIT_FAIL;
//         }
//     }
//     /*Alternatively, use the following line to wait for interrupt flag:
//     while(!(OC_SPI_SPSR(b_addr) & OC_SPI_SPIF));
//     Wait for receive buffer not empty flag to be set*/
//     while (OC_ISSET(OC_SPI_SPSR(address), OC_SPI_RFEMPTY))
//     {
//         if ((HAL_GetTick() - current_millis) > Timeout)
//         {
//             OC_SPI_SPDR(address);
//             return SPI_TRANSMIT_FAIL;
//         }
//     }
//     /* Alternatively, use the following line to wait for receive buffer not empty flag:
//      while(OC_SPI_SPSR(b_addr) & OC_SPI_SPIF);
//      Read received data from SPI data register */

//   b = READ_Reg(OC_SPI_SPDR(address));
//     // Return received data
//     return b;
// }



//   byte transfer(byte _data) {
//      SPDR = _data;
//     asm volatile("nop");
//     // Check For Recieve FIFO Empty Flag
//     while((SPSR & RFEM));
//     return SPDR;
//   }


  

// // void SPIClass::beginTransaction(int pin, char settings)
// // { 
// //     /* Set SPI control register with desired configuration (no interrupts, core enabled,
// //     reserved, controller, cpol=0, cha=0, clock divisor 11 for 4096)*/
// //     switch (settings)
// //     {
// //     case 0:
// //         OC_BITSET(OC_SPI_SPCR(pin),OC_SPI_SPR);
// //         OC_BITCLEAR(OC_SPI_SPCR(pin), OC_SPI_CPHA);
// //         OC_BITCLEAR(OC_SPI_SPCR(pin), OC_SPI_CPOL);
// //         OC_BITSET(OC_SPI_SPCR(pin),OC_SPI_MSTR);
// //         OC_BITSET(OC_SPI_SPCR(pin),OC_SPI_SPE);
// //         OC_BITCLEAR(OC_SPI_SPCR(pin),OC_SPI_SPIE);

// //         break;
// //     case 1:
// //         OC_BITSET(OC_SPI_SPCR(pin),OC_SPI_SPR);
// //         OC_BITCLEAR(OC_SPI_SPCR(pin), OC_SPI_CPHA);
// //         OC_BITSET(OC_SPI_SPCR(pin), OC_SPI_CPOL);
// //         OC_BITSET(OC_SPI_SPCR(pin),OC_SPI_MSTR);
// //         OC_BITSET(OC_SPI_SPCR(pin),OC_SPI_SPE);
// //         OC_BITCLEAR(OC_SPI_SPCR(pin),OC_SPI_SPIE);
// //         break;
// //     case 2:
// //         OC_BITSET(OC_SPI_SPCR(pin),OC_SPI_SPR);
// //         OC_BITSET(OC_SPI_SPCR(pin), OC_SPI_CPHA);
// //         OC_BITCLEAR(OC_SPI_SPCR(pin), OC_SPI_CPOL);
// //         OC_BITSET(OC_SPI_SPCR(pin),OC_SPI_MSTR);
// //         OC_BITSET(OC_SPI_SPCR(pin),OC_SPI_SPE);
// //         OC_BITCLEAR(OC_SPI_SPCR(pin),OC_SPI_SPIE);
// //         break;
// //     case 3:
// //         OC_BITSET(OC_SPI_SPCR(pin),OC_SPI_SPR);
// //         OC_BITSET(OC_SPI_SPCR(pin), OC_SPI_CPHA);
// //         OC_BITSET(OC_SPI_SPCR(pin), OC_SPI_CPOL);
// //         OC_BITSET(OC_SPI_SPCR(pin),OC_SPI_MSTR);
// //         OC_BITSET(OC_SPI_SPCR(pin),OC_SPI_SPE);
// //         OC_BITCLEAR(OC_SPI_SPCR(pin),OC_SPI_SPIE);
// //         break;
// //     default:
// //         OC_BITSET(OC_SPI_SPCR(pin),OC_SPI_SPR);
// //         OC_BITCLEAR(OC_SPI_SPCR(pin), OC_SPI_CPHA);
// //         OC_BITCLEAR(OC_SPI_SPCR(pin), OC_SPI_CPOL);
// //         OC_BITSET(OC_SPI_SPCR(pin),OC_SPI_MSTR);
// //         OC_BITSET(OC_SPI_SPCR(pin),OC_SPI_SPE);
// //         OC_BITCLEAR(OC_SPI_SPCR(pin),OC_SPI_SPIE);
// //         break;
// //     }
// //     // // Set SPI event register with desired configuration (interrupt count 00 (7:6), clock divisor 10 (1:0) for 4096)
// //     WRITE_Reg(OC_SPI_SPER(pin), 0x01);
// //     // Return success status
// // }
// void SPIClass::attachInterrupt(void) {
// 	// Should be enableInterrupt()
// }

// void SPIClass::detachInterrupt(void) {
// 	// Should be disableInterrupt()
// }


// void SPIClass::chipSelect(int pin, int enable)
// {
//     if (enable)
//     {
//         WRITE_Reg(OC_SPI_SPCS(pin), 0xFF); // CS DOWN
//     }
//     else
//     {
//         WRITE_Reg(OC_SPI_SPCS(pin), 0x00); // CS UP
//     }
//     for (int i = 0; i < 100; i++)
//         ; // Give some time for Chip to respond
// //   return enable;
// }

// #if SPI_INTERFACES_COUNT > 0
// SPIClass SPI(1);
// #endif

