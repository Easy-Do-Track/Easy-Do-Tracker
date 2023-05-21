#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"

#define TCA_ADDR 0x70

void waitForUSB(){
    while (!stdio_usb_connected()) { // blink the pico's led until usb connection is established
        sleep_ms(10);
        sleep_ms(10);
    }
}

// TODO: TCA 2개 병렬 연결 시 12개까지 선택 가능하도록 수정
// i2cdevlib이 기본 버스만 지원해서 TCA9548 주소 변경 후 병렬 연결해야 할 것으로 보임
void tcaSelect(uint8_t i){
    if (i>7) return;

    i = 1<<i;

    i2c_write_blocking(i2c_default, TCA_ADDR, &i, 1, false);
}

void scanTCAPorts(){
    for (uint8_t i=0; i<8; i++){
        printf("TCA Port #%d", i);
        tcaSelect(i);

        for (uint8_t addr=0; addr<=127; addr++){
            if (addr==TCA_ADDR) continue;

            int ret;
            uint8_t data;

            ret = i2c_read_blocking(i2c_default, addr, &data, 1, false);

            if (ret>0){
                printf("Found %02x\n", addr);
            }
        }
    }
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

    //scanTCAPorts();

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

    while(1){
        uint8_t fifo_buffer[64];
        if (!mpu.dmpGetCurrentFIFOPacket(fifo_buffer)){
            continue;
        }

        Quaternion q;
        mpu.dmpGetQuaternion(&q, fifo_buffer);

        printf("x: %.5f y: %.5f z: %.5f w: %.5f\n",
               q.x, q.y, q.z, q.w);
    }

    return 0;
}
