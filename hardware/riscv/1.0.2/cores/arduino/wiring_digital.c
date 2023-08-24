// See LICENSE file for license details.

#include "Arduino.h"

__BEGIN_DECLS

void pinMode( uint32_t dwPin, uint32_t dwMode)
{
        Remap(dwPin, OUTPUT, GPIO);

} 

void digitalWrite(uint32_t pin, uint32_t val)
{
    //Remap(pin, 0xf, 15);
  if (pin ==0)
    {
        pin =1;
    }
    //DIR_REG|=1<<(pin-1);
    
    if(val)
    {
    GPIO_OUTPUT |= 1<<(pin-1);  // BitSet
    }

    else
    {
    GPIO_OUTPUT &= ~(1<<(pin-1));
    }

}

int digitalRead(uint32_t pin)
{
    //Remap(pin, 0, 15);
    int state;
    state = GPIO_INPUT & 1<<(pin-1);
    if (state)
    {
        return 1;
    }
    else{
        return 0;
    }
}

void Remap(char pin_number, unsigned int pin_dir, uint8_t x)
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

__END_DECLS
