#include <ti/csl/soc.h>
#include <ti/csl/csl_types.h>
#include <ti/drv/gpio/GPIO.h>
#include <ti/drv/gpio/soc/GPIO_soc.h>
#include <ti/drv/gpio/test/led_blink/src/GPIO_board.h>
#include <ti/board/board_cfg.h>


/* GPIO Driver board specific pin configuration structure */
GPIO_PinConfig gpioPinConfigs[] =
{
    /* Input pin with interrupt enabled */
    GPIO_DEVICE_CONFIG(GPIO_LED0_PORT_NUM, GPIO_LED0_PIN_NUM) |
    GPIO_CFG_IN_INT_RISING | GPIO_CFG_OUTPUT,

    /* Output pin */
    GPIO_DEVICE_CONFIG(GPIO_LED1_PORT_NUM, GPIO_LED1_PIN_NUM) |
    GPIO_CFG_OUTPUT
};

/* GPIO Driver call back functions */
GPIO_CallbackFxn gpioCallbackFunctions[] =
{
    NULL,
    NULL
};

/* GPIO Driver configuration structure */
GPIO_v0_Config GPIO_v0_config =
{
    gpioPinConfigs,
    gpioCallbackFunctions,
    sizeof(gpioPinConfigs) / sizeof(GPIO_PinConfig),
    sizeof(gpioCallbackFunctions) / sizeof(GPIO_CallbackFxn),
    12,
};
