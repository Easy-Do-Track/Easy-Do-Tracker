#include <stdio.h>
#include <vector>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "pico/cyw43_arch.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "pico/time.h"

typedef Quaternion sensorData[16];

#define TCA_ADDR_1 0x70
#define TCA_ADDR_2 0x71
#define MSG_MAX_LEN sizeof(sensorData)

#define BTN_IN 12
#define BTN_OUT 13

void waitForUSB(){
    while (!stdio_usb_connected()) { // blink the pico's led until usb connection is established
        sleep_ms(10);
        sleep_ms(10);
    }
}

void tcaSelect(uint8_t i){
    uint8_t t1, t2;

    if (i>7) {
        t1 = 0;
        t2 = 1<<(i-8);
    }else{
        t1 = 1<<i;
        t2 = 0;
    }

    i2c_write_blocking(i2c_default, TCA_ADDR_1, &t1, 1, false);
    i2c_write_blocking(i2c_default, TCA_ADDR_2, &t2, 1, false);
}

std::vector<uint8_t> scanTCAPorts(){
    std::vector<uint8_t> result;

    for (uint8_t i=0; i<16; i++){
        printf("TCA Port #%d\n", i);
        tcaSelect(i);

        for (uint8_t addr=0; addr<=127; addr++){
            if (addr==TCA_ADDR_1 || addr==TCA_ADDR_2) continue;

            int ret;
            uint8_t data;

            ret = i2c_read_blocking(i2c_default, addr, &data, 1, false);

            if (ret>0){
                printf("Found %02x\n", addr);
                result.push_back(i);
            }
        }
    }

    return result;
}

int main() {
    stdio_init_all();
    i2c_init(i2c_default, 100*1000);
    gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);
    gpio_pull_up(0);
    gpio_pull_up(1);

    cyw43_arch_init();
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    gpio_init(BTN_IN);
    gpio_set_dir(BTN_IN, GPIO_IN);
    gpio_pull_up(BTN_IN);

    gpio_init(BTN_OUT);
    gpio_set_dir(BTN_OUT, GPIO_OUT);
    gpio_put(BTN_OUT, false);

    //waitForUSB();

    printf("Hello!\n");

    printf("WIFI SSID: %s\n", WIFI_SSID);
    printf("WIFI Password: %s\n", WIFI_PASSWORD);
    printf("UDP Address: %s\n", UDP_ADDR);
    printf("UDP Port: %d\n", UDP_PORT);

    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi...\n");

    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)){
        printf("Failed to connect. Retrying...\n");
    }
    printf("Connected.\n");

    udp_pcb* pcb = udp_new();
    ip_addr_t  addr;
    ipaddr_aton(UDP_ADDR, &addr);

    auto ports = scanTCAPorts();

    if (ports.empty()){
        printf("MPU6050 Not Found\n");
        return 1;
    }

    MPU6050 mpu(0x68);
    for (auto port: ports){
        printf("Initializing Port #%d\n", port);
        tcaSelect(port);

        mpu.initialize();
        auto err = mpu.dmpInitialize();

        if (err==1U){
            printf("initial memory load failed\n");
            return 1;
        }
        if (err==2U){
            printf("DMP configuration update failed\n");
            return 2;
        }

        if (!mpu.testConnection()){
            printf("connection failed\n");
            return 3;
        }

        mpu.setXGyroOffset(0);
        mpu.setYGyroOffset(0);
        mpu.setZGyroOffset(0);
        mpu.setXAccelOffset(0);
        mpu.setYAccelOffset(0);
        mpu.setZAccelOffset(0);

        // 6초간 캘리브레이션
        mpu.CalibrateAccel(3);
        mpu.CalibrateGyro(3);

        printf("\n");

        mpu.setDMPEnabled(true);
    }

    sensorData data;
    while(1) {
        if (!gpio_get(BTN_IN)){
            sleep_ms(50);
            while(!gpio_get(BTN_IN));
            printf("Resetting...\n");
            watchdog_enable(1,1);
            while(1);
        }

        auto start = to_ms_since_boot(get_absolute_time());

        for (auto port: ports) {
            tcaSelect(port);

            uint8_t fifo_buffer[64];
            while (!mpu.dmpGetCurrentFIFOPacket(fifo_buffer));

            mpu.dmpGetQuaternion(&data[port], fifo_buffer);
        }

        pbuf *p = pbuf_alloc(PBUF_TRANSPORT, MSG_MAX_LEN, PBUF_RAM);
        char *req = (char *) p->payload;
        memset(req, 0, MSG_MAX_LEN);
        memcpy(req, data, MSG_MAX_LEN);

        err_t e = udp_sendto(pcb, p, &addr, UDP_PORT);
        pbuf_free(p);
        if (e != ERR_OK) {
            printf("Failed to send UDP packet: %d\n", e);
        }
        auto end = to_ms_since_boot(get_absolute_time());
        printf("Scan took: %lu ms\n", end-start);
    }

    return 0;
}
