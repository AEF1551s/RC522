#if !defined(USER_TYPEDEFS_H)
#define USER_TYPEDEFS_H

#include <stm32f410rx.h>
#include <stm32f4xx.h>
#include <stdbool.h>

typedef enum
{
    HIGH = 0x1U,
    LOW = !HIGH
} pin_state_TypeDef;

typedef enum
{
    INPUT = 0x0U,
    OUTPUT = 0x1U,
    ALTERNATE = 0x2U,
    ANALOG = 0x3U

} pin_mode_TypeDef;

typedef enum
{
    NONE = 0X0U,
    PULLUP = 0x1U,
    PULLDOWN = 0x2U,
    RESERVED = 0x3U

} pupd_mode_TypeDef;

typedef enum
{
    PIN0 = 0U,
    PIN1 = 1U,
    PIN2 = 2U,
    PIN3 = 3U,
    PIN4 = 4U,
    PIN5 = 5U,
    PIN6 = 6U,
    PIN7 = 7U,
    PIN8 = 8U,
    PIN9 = 9U,
    PIN10 = 10U,
    PIN11 = 11U,
    PIN12 = 12U,
    PIN13 = 13U,
    PIN14 = 14U,
    PIN15 = 15U
} pin_TypeDef;

typedef struct
{
    GPIO_TypeDef *GPIOx;
    pin_TypeDef pinx;
    pin_mode_TypeDef mode;
    // TODO:Flag to check if the pin has already been set
} pin_struct_TypeDef;

#endif // USER_TYPEDEFS_H
