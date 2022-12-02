#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include <boards/adafruit_qtpy_rp2040.h>
#include "pico/stdlib.h"

#include "app.h"
#include "mpu6050_i2c.h"

#define TRELLIS_ADDR  0x70;
#define SOLENOID_GPIO   20
#define WIFI_GPIO       22
#define ACCELERATION_THRESH     500

static int addr = 0x68;

//#ifdef i2c_default
static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, addr, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, addr, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, addr, &val, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, addr, &val, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

int main()
{

    //
    //=============================MPU INITIALISATION=============================================
    //
    
    stdio_init_all();
#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
    #warning i2c/mpu6050_i2c example requires a board with I2C pins
    puts("Default I2C pins were not defined");
#else
    printf("Hello, MPU6050! Reading raw data from registers...\n");
#endif

    // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    mpu6050_reset();

    int16_t acceleration[3], gyro[3], temp;

    //
    //==============================MPU INIT ENDS===============================================================
    //

    gpio_init(SOLENOID_GPIO);
    gpio_set_dir(SOLENOID_GPIO, GPIO_OUT);
    gpio_init(WIFI_GPIO);
    gpio_set_dir(WIFI_GPIO, GPIO_OUT);

    while(1)
    {
        mpu6050_read_raw(acceleration, gyro, &temp);

        if(acceleration[0] >= ACCELERATION_THRESH || acceleration[1] >= ACCELERATION_THRESH || acceleration[2] >= ACCELERATION_THRESH)
        {
            int count = 100000;
            while(count > 0){
            gpio_put(SOLENOID_GPIO, 1);
            gpio_put(WIFI_GPIO, 1);
            sleep_ms(1000);
            count--;
            }
        }

    }



}