// See LICENSE file for license details.

#include "Arduino.h"
#include "Remap.h"

__BEGIN_DECLS

// void
// pinMode(uint32_t pin, uint32_t mode)
// {
  
//   if (pin >= variant_pin_map_size)
//     return;
  
//   GPIO_REG(GPIO_OUTPUT_XOR)  &= ~digitalPinToBitMask(pin);
//   GPIO_REG(GPIO_IOF_EN)      &= ~digitalPinToBitMask(pin);
// //pwm_enabled_pin[pin] = 0;
  
//   switch (mode) {
//   case INPUT_PULLUP:
//     GPIO_REG(GPIO_INPUT_EN)  |=  digitalPinToBitMask(pin);
//     GPIO_REG(GPIO_OUTPUT_EN) &= ~digitalPinToBitMask(pin);
//     GPIO_REG(GPIO_PULLUP_EN) |=  digitalPinToBitMask(pin);
//     break;
//   case INPUT:
//     GPIO_REG(GPIO_INPUT_EN)  |=  digitalPinToBitMask(pin);
//     GPIO_REG(GPIO_OUTPUT_EN) &= ~digitalPinToBitMask(pin);
//     GPIO_REG(GPIO_PULLUP_EN) &= ~digitalPinToBitMask(pin);
//     break;
//   case OUTPUT:
//     GPIO_REG(GPIO_INPUT_EN)  &= ~digitalPinToBitMask(pin);
//     GPIO_REG(GPIO_OUTPUT_EN) |=  digitalPinToBitMask(pin);
//     GPIO_REG(GPIO_PULLUP_EN) &= ~digitalPinToBitMask(pin);
//     break;
//   }
// }


// void
// digitalWrite(uint32_t pin, uint32_t val)
// {
//   if (pin >= variant_pin_map_size)
//     return;
  
//   if (val)
//     GPIO_REG(GPIO_OUTPUT_VAL) |=  digitalPinToBitMask(pin);
//   else
//     GPIO_REG(GPIO_OUTPUT_VAL) &= ~digitalPinToBitMask(pin);

// }

// int
// digitalRead(uint32_t pin)
// {
// if (pin >= variant_pin_map_size)
//   return 0;

//  return ((GPIO_REG(GPIO_INPUT_VAL) & digitalPinToBitMask(pin)) != 0);
// }

void pinMode( uint32_t dwPin, uint32_t dwMode)
{
        Remap(dwPin, OUTPUT, 15);

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

__END_DECLS
