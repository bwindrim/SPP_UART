/*
 * Copyright (C) 2014 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BLUEKITCHEN
 * GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at 
 * contact@bluekitchen-gmbh.com
 *
 */

#define BTSTACK_FILE__ "spp_uart.c"

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>

#include "hardware/uart.h"
#include "hardware/irq.h"

#include "btstack.h"

#define RFCOMM_SERVER_CHANNEL 1
#define HEARTBEAT_PERIOD_MS 1000
#include "btstack_run_loop.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

// UART defines
// By default the stdout uses uart0, so we will use the uart1
#define UART_ID uart1
#define BAUD_RATE 57600

// Use GPIOs 4 and 5 for UART1
// GPIOs can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define UART_TX_GPIO 20
#define UART_RX_GPIO 21
#define UART_CTS_GPIO 22
#define UART_RTS_GPIO 23


static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

static uint16_t rfcomm_channel_id;
static uint8_t  spp_service_buffer[150];
static btstack_packet_callback_registration_t hci_event_callback_registration;
static bool led_state = false;

/* @section SPP Service Setup 
 *s
 * @text To provide an SPP service, the L2CAP, RFCOMM, and SDP protocol layers 
 * are required. After setting up an RFCOMM service with channel nubmer
 * RFCOMM_SERVER_CHANNEL, an SDP record is created and registered with the SDP server.
 * Example code for SPP service setup is
 * provided in Listing SPPSetup. The SDP record created by function
 * spp_create_sdp_record consists of a basic SPP definition that uses the provided
 * RFCOMM channel ID and service name. For more details, please have a look at it
 * in \path{src/sdp_util.c}. 
 * The SDP record is created on the fly in RAM and is deterministic.
 * To preserve valuable RAM, the result could be stored as constant data inside the ROM.   
 */

static void spp_service_setup(void){

    // register for HCI events
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    l2cap_init();

#ifdef ENABLE_BLE
    // Initialize LE Security Manager. Needed for cross-transport key derivation
    sm_init();
#endif

    rfcomm_init();
    rfcomm_register_service(packet_handler, RFCOMM_SERVER_CHANNEL, 0xffff);  // reserved channel, mtu limited by l2cap

    // init SDP, create record for SPP and register with SDP
    sdp_init();
    memset(spp_service_buffer, 0, sizeof(spp_service_buffer));
    spp_create_sdp_record(spp_service_buffer, 0x10001, RFCOMM_SERVER_CHANNEL, "SPP Lattice");
    sdp_register_service(spp_service_buffer);
    printf("SDP service record size: %u\n", de_get_len(spp_service_buffer));
}


struct CircBuffer{
    // Ping-pong flag, shared between thread level and interrupt level.
    // Thread level can only set it to true if false, interrupt level can only set it to false if true
    volatile bool irqIsEnabled;
    volatile uint8_t head, tail;
    char vChars[256];
};

static struct CircBuffer rxBuffer, txBuffer;

uint8_t chars_in_buffer(struct CircBuffer *pBuffer){
    return (pBuffer->head - pBuffer->tail);
}

uint8_t space_in_buffer(struct CircBuffer *pBuffer){
    return (uint8_t)(255u - chars_in_buffer(pBuffer));
}

char circ_buffer_get(struct CircBuffer *pBuffer){
    return pBuffer->vChars[pBuffer->tail++];
}

void circ_buffer_put(struct CircBuffer *pBuffer, char c){
    pBuffer->vChars[pBuffer->head++] = c;
}

uint16_t circ_buffer_get_block(uint8_t buffer[], uint16_t size){
    uint16_t count = 0;

    while (chars_in_buffer(&rxBuffer) && count < size) {
        buffer[count++] = circ_buffer_get(&rxBuffer);
    }

    // If the rx interrupt was disabled due to lack of buffer space then re-enable it.
    if (!rxBuffer.irqIsEnabled){
        rxBuffer.irqIsEnabled = true;
        uart_set_irqs_enabled(uart1, rxBuffer.irqIsEnabled, txBuffer.irqIsEnabled);
        irq_set_pending(UART1_IRQ);
    }

    return count;
}

/// @brief 
/// @param buffer 
/// @param size 
/// @return 
uint16_t circ_buffer_put_block(uint8_t buffer[], uint16_t size){
    uint16_t count = 0;

    while (space_in_buffer(&txBuffer) && count < size) {
        circ_buffer_put(&txBuffer, buffer[count++]);
    }

    if (count > 0){
        // If the tx interrupt was disabled due to lack of chars in the buffer then re-enable it.
        if (!txBuffer.irqIsEnabled){
            txBuffer.irqIsEnabled = true;
            uart_set_irqs_enabled(uart1, rxBuffer.irqIsEnabled, txBuffer.irqIsEnabled);
            //irq_set_pending(UART1_IRQ);
        }
    
        irq_set_pending(UART1_IRQ);
    }
}

static void uart_handler (struct btstack_data_source *ds, btstack_data_source_callback_type_t callback_type){
    bool send_now = false;

    switch (callback_type){
    case DATA_SOURCE_CALLBACK_POLL:
        if (chars_in_buffer(&rxBuffer)){
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
            rfcomm_request_can_send_now_event(rfcomm_channel_id);
        } else {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
        }
        break;
    case DATA_SOURCE_CALLBACK_READ:
        printf("uart_handler(): DATA_SOURCE_CALLBACK_READ\n");
        break;
    case DATA_SOURCE_CALLBACK_WRITE:
        printf("uart_handler(): DATA_SOURCE_CALLBACK_WRITE\n");
        break;
    default:
        printf("uart_handler(): unexpected callback_type %d\n", callback_type);
        break;
    }
}

static volatile uint32_t uart1_irq_count = 0;

static void uart1_irq_handler(void){
    ++uart1_irq_count;
    while (uart_is_readable(uart1) && space_in_buffer(&rxBuffer)){
        // Get char from UART and put it into the Rx circular buffer.
        circ_buffer_put(&rxBuffer, uart_getc(uart1));
        // Flag that we need a "send now" event, as we have data to send.
        btstack_run_loop_poll_data_sources_from_irq();
    }
    // If there's no space left the Rx buffer then we disable the UART Rx interrupt.
    if (rxBuffer.irqIsEnabled && !space_in_buffer(&rxBuffer)) {
        rxBuffer.irqIsEnabled = false;
        uart_set_irqs_enabled(uart1, rxBuffer.irqIsEnabled, txBuffer.irqIsEnabled);
    }

    while (uart_is_writable(uart1) && chars_in_buffer(&txBuffer)){
        // Get the next char from the Tx circular bufferand send it to the UART.
        uart_putc_raw(uart1, circ_buffer_get(&txBuffer));
    }
    // If there's nothing in the Tx buffer then we disable the UART Tx interrupt.
    if (txBuffer.irqIsEnabled && !chars_in_buffer(&txBuffer)){
        txBuffer.irqIsEnabled = false;
        uart_set_irqs_enabled(uart1, rxBuffer.irqIsEnabled, txBuffer.irqIsEnabled);
    }
}

static btstack_data_source_t uart_data_source;
static void uart_data_source_setup(void){
    uart_data_source.process = uart_handler;
    uart_data_source.source.handle = NULL;
    btstack_run_loop_add_data_source(&uart_data_source);
    btstack_run_loop_enable_data_source_callbacks(&uart_data_source, DATA_SOURCE_CALLBACK_POLL);

    rxBuffer.irqIsEnabled = true;
    txBuffer.irqIsEnabled = false;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART1_IRQ, uart1_irq_handler);
    irq_set_enabled(UART1_IRQ, true);

    uart_set_irqs_enabled(uart1, rxBuffer.irqIsEnabled, txBuffer.irqIsEnabled);
}


/* @section Bluetooth Logic 
 * @text The Bluetooth logic is implemented within the 
 * packet handler, see Listing SppServerPacketHandler. In this example, 
 * the following events are passed sequentially: 
 * - BTSTACK_EVENT_STATE,
 * - HCI_EVENT_PIN_CODE_REQUEST (Standard pairing) or 
 * - HCI_EVENT_USER_CONFIRMATION_REQUEST (Secure Simple Pairing),
 * - RFCOMM_EVENT_INCOMING_CONNECTION,
 * - RFCOMM_EVENT_CHANNEL_OPENED, 
* - RFCOMM_EVETN_CAN_SEND_NOW, and
 * - RFCOMM_EVENT_CHANNEL_CLOSED
 */

/* @text Upon receiving HCI_EVENT_PIN_CODE_REQUEST event, we need to handle
 * authentication. Here, we use a fixed PIN code "0000".
 *
 * When HCI_EVENT_USER_CONFIRMATION_REQUEST is received, the user will be 
 * asked to accept the pairing request. If the IO capability is set to 
 * SSP_IO_CAPABILITY_DISPLAY_YES_NO, the request will be automatically accepted.
 *
 * The RFCOMM_EVENT_INCOMING_CONNECTION event indicates an incoming connection.
 * Here, the connection is accepted. More logic is need, if you want to handle connections
 * from multiple clients. The incoming RFCOMM connection event contains the RFCOMM
 * channel number used during the SPP setup phase and the newly assigned RFCOMM
 * channel ID that is used by all BTstack commands and events.
 *
 * If RFCOMM_EVENT_CHANNEL_OPENED event returns status greater then 0,
 * then the channel establishment has failed (rare case, e.g., client crashes).
 * On successful connection, the RFCOMM channel ID and MTU for this
 * channel are made available to the heartbeat counter. After opening the RFCOMM channel, 
 * the communication between client and the application
 * takes place. In this example, the timer handler increases the real counter every
 * second. 
 *
 * RFCOMM_EVENT_CAN_SEND_NOW indicates that it's possible to send an RFCOMM packet
 * on the rfcomm_cid that is include

 */ 
static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);

    bd_addr_t event_addr;
    uint8_t   rfcomm_channel_nr;
    uint16_t  mtu;
    int i;
    rpn_baud_t baud;
    static const uint32_t vBaudRates[9] =
    {
        2400,   // 0
        4800,   // 1
        7200,   // 2
        9600,   // 3
        19200,  // 4
        38400,  // 5
        57600,  // 6
        115200, // 7
        230400  // 8
    };

    switch (packet_type) {
        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet)) {
                case HCI_EVENT_PIN_CODE_REQUEST:
                    // inform about pin code request
                    printf("Pin code request - using '0000'\n");
                    hci_event_pin_code_request_get_bd_addr(packet, event_addr);
                    gap_pin_code_response(event_addr, "0000");
                    break;

                case HCI_EVENT_USER_CONFIRMATION_REQUEST:
                    // ssp: inform about user confirmation request
                    printf("SSP User Confirmation Request with numeric value '%06"PRIu32"'\n", little_endian_read_32(packet, 8));
                    printf("SSP User Confirmation Auto accept\n");
                    break;

                case RFCOMM_EVENT_PORT_CONFIGURATION:
                    baud = rfcomm_event_port_configuration_get_baud_rate(packet);
                    printf("Port config event: baud = %d (%d)\n", baud, vBaudRates[baud]);
                    uart_set_baudrate(uart1, vBaudRates[baud]);
                    break;

                case RFCOMM_EVENT_INCOMING_CONNECTION:
                    rfcomm_event_incoming_connection_get_bd_addr(packet, event_addr);
                    rfcomm_channel_nr = rfcomm_event_incoming_connection_get_server_channel(packet);
                    rfcomm_channel_id = rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
                    printf("RFCOMM channel %u requested for %s\n", rfcomm_channel_nr, bd_addr_to_str(event_addr));
                    rfcomm_accept_connection(rfcomm_channel_id);
                    break;
               
                case RFCOMM_EVENT_CHANNEL_OPENED:
                    if (rfcomm_event_channel_opened_get_status(packet)) {
                        printf("RFCOMM channel open failed, status 0x%02x\n", rfcomm_event_channel_opened_get_status(packet));
                    } else {
                        rfcomm_channel_id = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
                        mtu = rfcomm_event_channel_opened_get_max_frame_size(packet);
                        printf("RFCOMM channel open succeeded. New RFCOMM Channel ID %u, max frame size %u\n", rfcomm_channel_id, mtu);
                        // Set up our UART
                        uart_init(uart1, 57600);
                        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state = true);
                    }
                    break;

                case RFCOMM_EVENT_CAN_SEND_NOW:
                    if (chars_in_buffer(&rxBuffer)){
                        rfcomm_reserve_packet_buffer();
                        uint8_t *buffer = rfcomm_get_outgoing_buffer();
                        uint16_t buffer_size = rfcomm_get_max_frame_size(rfcomm_channel_id);
                        // Set up data in buffer with len
                        uint16_t len = circ_buffer_get_block(buffer, buffer_size);
                        printf("Sending %d bytes\n", len);
                        rfcomm_send_prepared(rfcomm_channel_id, len);
                    }
                    break;

                case RFCOMM_EVENT_CHANNEL_CLOSED:
                    printf("RFCOMM channel closed\n");
                    uart_deinit(uart1);
                    rfcomm_channel_id = 0;
                    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state = false);
                    break;
                
                default:
                    break;
            }
            break;

        case RFCOMM_DATA_PACKET:
#if 1
            printf("RCV(%d): '", size);
            for (i=0;i<size;i++){
                putchar(packet[i]);
            }
            printf("'\n");
#endif
            circ_buffer_put_block(packet, size);
            break;

        default:
            break;
    }
}


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

    // inform about BTstack state
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    uart_data_source_setup();
    spp_service_setup();

    gap_discoverable_control(1);
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
    gap_set_local_name("Lattice 0 debug 00:00:00:00:00:00");

    // turn on!
    hci_power_control(HCI_POWER_ON);

    printf("...init complete.\n");

    btstack_run_loop_execute();

    return 0;
}

