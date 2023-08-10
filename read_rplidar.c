

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/uart.h>
#include <driver/gpio.h>

#define TXD_PIN_2 GPIO_NUM_17
#define RXD_PIN_2 GPIO_NUM_16
#define RXD_BUFFER_SIZE 1024

// static const uint8_t request_packet[] = {0xa5, 0x20};

typedef struct RPLidarMeasurement
{
    uint8_t quality;
    float angle;
    float distance;
} __packed RPLidarMeasurement; 

void uart_init_2()
{
    uart_config_t uart_config_2 = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
        //.rx_flow_ctrl_thresh = 122,
    };
    uart_param_config(UART_NUM_2, &uart_config_2);
    uart_set_pin(UART_NUM_2, TXD_PIN_2, RXD_PIN_2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_2, RXD_BUFFER_SIZE, 0, 0, NULL, 0);
}

void send_express_scan_request()
{
    static const uint8_t request_packet[] = {0xa5, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22};
    uart_write_bytes(UART_NUM_2, (const char *)request_packet, sizeof(request_packet));
    // vTaskDelay(10/portTICK_PERIOD_MS);
}

void send_reset_scan_request()
{
    static const uint8_t request_packet[] = {0xa5, 0x40};
    uart_write_bytes(UART_NUM_2, (const char*) request_packet, sizeof(request_packet));
    vTaskDelay(10/portTICK_PERIOD_MS);
}

void recieve_response_descriptor()
{
    uint8_t startFlag1[] = {0};
    uint8_t startFlag2[] = {0};

    //================check startFlag==========================
    do
    {
        uart_read_bytes(UART_NUM_2, startFlag1, sizeof(startFlag1), 100 / portTICK_PERIOD_MS);
        if (startFlag1[0] == 0xa5)
        {
            uart_read_bytes(UART_NUM_2, startFlag2, sizeof(startFlag2), 100 / portTICK_PERIOD_MS);
        }

        //======for Debug=========
        printf("read byte: 0x%02x 0x%02x\n", startFlag1[0], startFlag2[0]);

    } while (!(startFlag1[0] == 0xa5 && startFlag2[0] == 0x5a));

    uint8_t bytes[5];
    uart_read_bytes(UART_NUM_2, bytes, sizeof(bytes), 100 / portTICK_PERIOD_MS);

    //=========check for Debug==========
    printf("Response Descriptor: \n0x%02x | 0x%02x | 0x%02x | 0x%02x | 0x%02x | 0x%02x | 0x%02x\n", startFlag1[0], startFlag2[0], bytes[0], bytes[1], bytes[2], bytes[3], bytes[4]);
}

void receive_measurement_legacy_version()
{
    uint8_t sync1[] = {0};
    uint8_t sync2[] = {0};
    uint8_t chkSum = 0;

    //============check sync byte=================
    do
    {
        uart_read_bytes(UART_NUM_2, sync1, sizeof(sync1), 100/portTICK_PERIOD_MS);
        chkSum = sync1[0] & 0x0F;
        sync1[0] = sync1[0] >> 4;
        if(sync1[0]==0xA)
        {
            uart_read_bytes(UART_NUM_2, sync2, sizeof(sync2), 100/portTICK_PERIOD_MS);
        }
        chkSum |= sync2[0] << 4;
        sync2[0] = sync2[0] >> 4;

        //===============for Debug===============
        printf("0x%01x | 0x%01x | %u\n", sync1[0], sync2[0], chkSum);

    } while (! (sync1[0]==0xA && sync2[0]==0x5));

    uint8_t angle_data[2];
    uart_read_bytes(UART_NUM_2, angle_data, sizeof(angle_data), 100/portTICK_PERIOD_MS);
    uint16_t start_angle_q6 = (uint16_t)((angle_data[1] & !(0x80)) <<8 ) | angle_data[0];
    float actual_angle = start_angle_q6 / 64.0;

    //=============print start angle==============
    printf("Start angle (fix point number): %u\nstart flag bit: %1x\n", start_angle_q6, angle_data[1]>>7);
    printf("Actual angle: %.2f\n", actual_angle);

    //==========each cabin inside the data packet===========
    for(int i=0; i< 16; i++)
    {
        uint8_t data[5];
        uint16_t distance1=0, distance2=0;
        uint8_t angle_compensate_1=0, angle_compensate_2=0;
        uart_read_bytes(UART_NUM_2, data, sizeof(data), 100/portTICK_PERIOD_MS);
        distance1 = (uint16_t) (data[1] << 6) | (data[0]>>2);
        distance2 = (uint16_t) (data[3] << 6) | (data[2]>>2);
        angle_compensate_1 = (0x0F & data[4]) | ((data[0]<<4) & 0x20);
        angle_compensate_2 = (data[4] >> 4) | ((data[2]<<4) & 0x20);
        printf("Distance 1: %u\nDistance 2: %u\nAngle 1: %u\nAngle 2: %u\n", distance1,distance2, angle_compensate_1, angle_compensate_2);
        printf("0x%02x | 0x%02x | 0x%02x | 0x%02x | 0x%02x\n", data[0], data[1], data[2], data[3], data[4]);
    }
}


void app_main()
{
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    uart_init_2();
    //send_reset_scan_request();
    vTaskDelay(1000/portTICK_PERIOD_MS);
    send_express_scan_request();
    recieve_response_descriptor();
    receive_measurement_legacy_version();
    receive_measurement_legacy_version();
/*
    while (1)
    {
        // vTaskDelay(500 / portTICK_PERIOD_MS);
        receive_measurement_legacy_version();
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
    }*/
}
