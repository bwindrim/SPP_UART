#include <stdio.h>
#include "hardware/uart.h"
#include "btstack_run_loop.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "btstack.h"
#include "picow_bt_example_common.h"

// UART defines
// By default the stdout UART is `uart0`, so we will use the second one
#define UART_ID uart1
#define BAUD_RATE 57600 // ToDo: change to 57600 for Lattice

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

    // Set up our UART
    uart_init(UART_ID, BAUD_RATE);
    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_GPIO, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_GPIO, GPIO_FUNC_UART);
    
    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID, false, false);

    // Turn on the Tx & Rx FIFOs
    uart_set_fifo_enabled(UART_ID, true);

    printf("Init complete\n");

    // In a default system, printf will also output via the default UART
    
    if (cyw43_arch_init()) {
        printf("failed to initialise cyw43_arch\n");
        return -1;
    }

    spp_uart_init();
    btstack_run_loop_execute();

    return 0;
}

