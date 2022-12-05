#include <stdio.h>

#include "pico/stdlib.h"
#include "pio_i2c.h"

#define PIN_SDA 22
#define PIN_SCL 23

static int addr = 0x68;


static void mpu6050_reset( PIO pio, uint sm) {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    //i2c_write_blocking(i2c_default, addr, buf, 2, false);
    pio_i2c_write_blocking(pio, sm, addr, buf, 2);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp, PIO pio, uint sm) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    //i2c_write_blocking(i2c_default, addr, &val, 1, true); // true to keep master control of bus
    //i2c_read_blocking(i2c_default, addr, buffer, 6, false);

    pio_i2c_write_blocking(pio, sm, addr, &val, 1);  
    pio_i2c_read_blocking(pio, sm, addr, buffer, 6);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    //i2c_write_blocking(i2c_default, addr, &val, 1, true);
    //i2c_read_blocking(i2c_default, addr, buffer, 6, false);  // False - finished with bus
    pio_i2c_write_blocking(pio, sm, addr, &val, 1);  
    pio_i2c_read_blocking(pio, sm, addr, buffer, 6);

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    //i2c_write_blocking(i2c_default, addr, &val, 1, true);
    //i2c_read_blocking(i2c_default, addr, buffer, 2, false);  // False - finished with bus
    pio_i2c_write_blocking(pio, sm, addr, &val, 1);  
    pio_i2c_read_blocking(pio, sm, addr, buffer, 6);

    *temp = buffer[0] << 8 | buffer[1];
}


// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

int main() {
    stdio_init_all();

    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &i2c_program);
    i2c_program_init(pio, sm, offset, PIN_SDA, PIN_SCL);
 
    printf("\nPIO I2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    mpu6050_reset(pio, sm);

    int16_t acceleration[3], gyro[3], temp;

    while (1) {
        mpu6050_read_raw(acceleration, gyro, &temp, pio, sm);

        // These are the raw numbers from the chip, so will need tweaking to be really useful.
        // See the datasheet for more information
        printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1], acceleration[2]);
        printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
        // Temperature is simple so use the datasheet calculation to get deg C.
        // Note this is chip temperature.
        printf("Temp. = %f\n", (temp / 340.0) + 36.53);

        sleep_ms(100);
    }

/*
    while(1){

    for (int addr = 0; addr < 0xFF; ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }
        // Perform a 0-byte read from the probe address. The read function
        // returns a negative result NAK'd any time other than the last data
        // byte. Skip over reserved addresses.
        int result;
        if (reserved_addr(addr))
            result = -1;
        else
            result = pio_i2c_read_blocking(pio, sm, addr, NULL, 0);

        printf(result < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
    printf("Done.\n");
    sleep_ms(2000);
    }
    */
    return 0;
}
