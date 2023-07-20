#ifndef REMAP_IO_h
#define REMAP_IO_h

//#include "macros.h"
// #include "bsp_printf.h"
// #include "macros.h"
// #include "defines.h"
///#include <defines.h>

#define REMAP    0x80004200
#define DIR_REG 0x80004364
#define PINSEL 0x80004340
#define REMAP_OUT_REG (REMAP)
#define REMAP_IN_REG (REMAP+0x24)

#define READ_Reg(dir) (*(volatile unsigned *)dir)
#define WRITE_Reg(dir,val) ((*(volatile unsigned *)dir) = (val))

#define GPIO_BASE_ADDR1 0x80001400    ///GPIO read  input
#define GPIO_INPUT (*((volatile unsigned *)(GPIO_BASE_ADDR1)))    //// GPIO write
#define GPIO_OUTPUT (*((volatile unsigned *)(GPIO_BASE_ADDR1+0x04)))    //// GPIO write  read
#define GPIO_DIR (*((volatile unsigned *)(GPIO_BASE_ADDR1+0x08)))    //// GPIO write  Dir  

// #define OUTPUT 0x0000000f
// #define INPUT 0x00000000


typedef enum
{ //OUTPUTS
    GPIO_old, 
    SPI_CS,
    SPI_MOSI,
    SPI_SCL,
    PWM1,
    PWM2,
    PWM_N_PTC4,
    PWM_P_PTC4,
    PWM_N_PTC3,
    PWM_P_PTC3,
    PWM_N_PTC2,
    PWM_P_PTC2,
    PWM_N_PTC1,
    PWM_P_PTC1,
    O_UART2_TX,
    GPIO,
  //INPUTS
    SPI_MISO = 1,    
    GATE_CLK_PAD_I3 = 2,
    GATE_CLK_PAD_I2 = 3,
    GATE_CLK_PAD_I = 4,
    CAPT_PAD_I4 = 5,
    CAPT_PAD_I3 = 6,   
    CAPT_PAD_I2 = 7,
    CAPT_PAD_I = 8,
    EXT_INT3 = 9,
    EXT_INT2 = 10,
    EXT_INT = 11,
    I_UART2_RX = 12
} PIN_MODE;

void Remap(char pin_number, unsigned int pin_dir, PIN_MODE x)
{
if (pin_number <33 && pin_number >0 && (pin_dir == INPUT || pin_dir == OUTPUT))
   { 
    unsigned int mask = 0;
    unsigned int dir_reg = 0;
    unsigned int address = 0;
    unsigned int shift_bits = 0;
    pin_number = pin_number == 0 ? 1 : pin_number; // pin0 does not exist so check it
    mask = 1 << (pin_number - 1);                    //create a mask for active pin

    if (pin_dir == OUTPUT)
    {
        dir_reg = READ_Reg(DIR_REG) | mask;                   //make bit 1
                                                              // Figure out how many bits to shift and which address
        int mul4 = pin_number % 4;                            // see if multiple of 4
        mul4 = mul4 == 0 ? 3 : mul4 - 1;                      // if multiple of 4 then assign 3 shifts
        shift_bits = mul4 * 8;                                // shift bytes
        address = REMAP_OUT_REG + ((pin_number - 1) / 4) * 4; // calculate address of outreg
        unsigned int current_REMAP_OUT_REG = READ_Reg(address);
        current_REMAP_OUT_REG &= ~(0xff << shift_bits);         //zero out current state bits
        current_REMAP_OUT_REG |= x << shift_bits;  //edit current output register
        WRITE_Reg(address, current_REMAP_OUT_REG); // get closest multiple of 4 and shift bits wrt to remainder
    }

    else if (pin_dir == INPUT)
    {
         dir_reg = READ_Reg(DIR_REG) & ~mask; // make bit 0
        //x=SPI_MISO;  //forced SPI MISO
        address = REMAP_IN_REG + ((x - 1) / 4) * 4;

        // Figure out how many bits to shift and which address
        int mul4 = x % 4;                                      // see if multiple of 4
        mul4 = mul4 == 0 ? 3 : mul4 - 1;                       // if multiple of 4 then assign 3 shifts
        shift_bits = mul4 * 8;                                 // shift bytes
        unsigned int current_REMAP_IN_REG = READ_Reg(address); // save current state
        current_REMAP_IN_REG &= ~(0xff << shift_bits);         //zero out current state bits
        current_REMAP_IN_REG |= (pin_number-1) << shift_bits;      //
        WRITE_Reg(address, current_REMAP_IN_REG);
    }
    WRITE_Reg(DIR_REG, dir_reg);
   }
 
}
#endif