#include <stdio.h>
#include <vector>
#include <string>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

#define TCA_ADDR 0x70
#define MSG_MAX_LEN 1024

void waitForUSB(){
    while (!stdio_usb_connected()) { // blink the pico's led until usb connection is established
        sleep_ms(10);
        sleep_ms(10);
    }
}

// TODO: TCA 2개 병렬 연결 시 12개까지 선택 가능하도록 수정
// i2cdevlib이 기본 버스만 지원해서 TCA9548 주소 변경 후 병렬 연결해야 할 것으로 보임
void tcaSelect(uint8_t i){
    if (i>7) {
        i = 0; // disable
    }else{
        i = 1<<i;
    }

    i2c_write_blocking(i2c_default, TCA_ADDR, &i, 1, false);
}

std::vector<uint8_t> scanTCAPorts(){
    std::vector<uint8_t> result;

    for (uint8_t i=0; i<16; i++){
        printf("TCA Port #%d\n", i);
        tcaSelect(i);

        for (uint8_t addr=0; addr<=127; addr++){
            if (addr==TCA_ADDR) continue;

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

    waitForUSB();

    printf("Hello!\n");

    printf("WIFI SSID: %s\n", WIFI_SSID);
    printf("WIFI Password: %s\n", WIFI_PASSWORD);
    printf("UDP Address: %s\n", UDP_ADDR);
    printf("UDP Port: %d\n", UDP_PORT);

    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)){
        printf("Failed to connect.\n");
        return 1;
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

    tcaSelect(0);
    MPU6050 mpu(0x68);

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
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);

    printf("\n");

    mpu.setDMPEnabled(true);

    char* buff = (char*)calloc(128, sizeof(char));
    if (buff==nullptr){
        printf("calloc error\n");
        return 1;
    }

    while(1) {
        loop:
        std::string result = "{";

        for (auto port: ports) {
            if (result.length()>1){
                result+=",";
            }

            tcaSelect(port);

            uint8_t fifo_buffer[64];
            if (!mpu.dmpGetCurrentFIFOPacket(fifo_buffer)) {
                goto loop;
            }

            Quaternion q;
            mpu.dmpGetQuaternion(&q, fifo_buffer);

            snprintf(buff, 128, R"(%u: [%f, %f, %f, %f])", port, q.w, q.x, q.y, q.z);
            result+=buff;
        }

        result+="}";

        pbuf *p = pbuf_alloc(PBUF_TRANSPORT, MSG_MAX_LEN, PBUF_RAM);
        char *req = (char *) p->payload;
        memset(req, 0, MSG_MAX_LEN);
        strncpy(req, result.c_str(), MSG_MAX_LEN);

        err_t e = udp_sendto(pcb, p, &addr, UDP_PORT);
        pbuf_free(p);
        if (e != ERR_OK) {
            printf("Failed to send UDP packet: %d\n", err);
        }
    }

    return 0;
}
