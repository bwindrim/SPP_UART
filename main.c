#include <stdio.h>
#include "hardware/uart.h"
#include "btstack_run_loop.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "btstack.h"
#include "picow_bt_example_common.h"

// UART defines
// By default the stdout uses uart0, so we will use the uart1
#define UART_ID uart1
#define BAUD_RATE 57600

// Use GPIOs 4 and 5 for UART1
// GPIOs can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define UART_TX_GPIO 4
#define UART_RX_GPIO 5
#define UART_CTS_GPIO 6
#define UART_RTS_GPIO 7

extern int spp_uart_init(void);

int main(int argc, const char * argv[])
{
    stdio_init_all();

    printf("Initialising...\n");

    // Set up our UART
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_GPIO, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_GPIO, GPIO_FUNC_UART);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_fifo_enabled(UART_ID, true);

    if (cyw43_arch_init()) {
        printf("...failed to initialise cyw43_arch. Terminating\n");
        return -1;
    }

    spp_uart_init();

    printf("...init complete.\n");

    btstack_run_loop_execute();

    return 0;
}

