#if !defined(USER_FUNCTIONS_H)
#define USER_FUNCTIONS_H

#include <stm32f410rx.h>
#include <stm32f4xx.h>
#include <user_types.h>

bool read_pin(pin_struct_TypeDef *pin, pin_state_TypeDef state_bit);
pin_struct_TypeDef pin_setup(GPIO_TypeDef *GPIOx, pin_TypeDef pinx, pin_mode_TypeDef mode);
void set_input_pull_mode(pin_struct_TypeDef *pin, pupd_mode_TypeDef input_mode);
void digital_write(pin_struct_TypeDef *pin, pin_state_TypeDef mode);

#endif // USER_FUNCTIONS_H
