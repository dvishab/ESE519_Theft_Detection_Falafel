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
#include "pio_i2c.h"
#include "string.h"

#define TRELLIS_ADDR  0x70;
#define SOLENOID_GPIO   29
#define WIFI_GPIO       22
#define ACCELERATION_THRESH     500

#define PIN_SDA 22
#define PIN_SCL 23

static int addr = 0x68;

#define REG_ACCEL_ZOUT_H 0x3F
#define REG_ACCEL_ZOUT_L 0x40
#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_CONFIG 0x1C
#define REG_SMPRT_DIV 0x19
#define REG_CONFIG 0x1A
#define REG_FIFO_EN 0x23
#define REG_USER_CTRL 0x6A
#define REG_FIFO_COUNT_L 0x72
#define REG_FIFO_COUNT_H 0x73
#define REG_FIFO 0x74
#define REG_WHO_AM_I 0x75


static void mpu6050_reset( PIO pio, uint sm) {

    /*
    i2c_write(REG_PWR_MGMT_1, 0x01);
	i2c_write(REG_ACCEL_CONFIG, 0x00);
	i2c_write(REG_SMPRT_DIV, 0x07);
	i2c_write(REG_CONFIG, 0x00);
	i2c_write(REG_FIFO_EN, 0x88);
	i2c_write(REG_USER_CTRL, 0x44);
    */

    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {REG_PWR_MGMT_1, 0x01};
    //i2c_write_blocking(i2c_default, addr, buf, 2, false);
    pio_i2c_write_blocking(pio, sm, addr, buf, 2);

    buf[0] = REG_ACCEL_CONFIG;
    buf[1] = 0x18;
    //i2c_write_blocking(i2c_default, addr, buf, 2, false);
    pio_i2c_write_blocking(pio, sm, addr, buf, 2);

    buf[0] = REG_SMPRT_DIV;
    buf[1] = 0x07;
    //i2c_write_blocking(i2c_default, addr, buf, 2, false);
    pio_i2c_write_blocking(pio, sm, addr, buf, 2);

    buf[0] = REG_CONFIG;
    buf[1] = 0x00;
    //i2c_write_blocking(i2c_default, addr, buf, 2, false);
    pio_i2c_write_blocking(pio, sm, addr, buf, 2);

    buf[0] = REG_FIFO_EN;
    buf[1] = 0x88;
    //i2c_write_blocking(i2c_default, addr, buf, 2, false);
    pio_i2c_write_blocking(pio, sm, addr, buf, 2);

    buf[0] = REG_USER_CTRL;
    buf[1] = 0x44;
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

uint16_t merge_bytes( uint8_t LSB, uint8_t MSB) {
	return  (uint16_t) ((( LSB & 0xFF) << 8) | MSB);
}

// 16 bits data on the MPU6050 are in two registers,
// encoded in two complement. So we convert those to int16_t
int16_t two_complement_to_int( uint8_t LSB, uint8_t MSB) {
	int16_t signed_int = 0;
	uint16_t word;

	word = merge_bytes(LSB, MSB);

	if((word & 0x8000) == 0x8000) { // negative number
		signed_int = (int16_t) -(~word);
	} else {
		signed_int = (int16_t) (word & 0x7fff);
	}

	return signed_int;
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

PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &i2c_program);
    i2c_program_init(pio, sm, offset, PIN_SDA, PIN_SCL);

    gpio_set_dir(SOLENOID_GPIO, GPIO_OUT);
    gpio_put(SOLENOID_GPIO, 1);


    // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
    //i2c_init(i2c_default, 400 * 1000);
    //gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    //gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    //gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    //gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    //bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

 mpu6050_reset(pio, sm);

    int16_t acceleration[3], gyro[3], temp;

    uint8_t accel_x_h,accel_x_l,accel_y_h,accel_y_l,accel_z_h,accel_z_l,temp_h,temp_l;

    uint8_t xavg = 0, yavg = 0, zavg = 0;

    int count = 0; 

    while(!stdio_usb_connected());
    char correctPass[10] = "1234";

    while (1) {


        if(-1 != getchar_timeout_us(5000))
        {
            printf("enter password\n");
            char password[10];
            scanf("%s", password); 

            if(!strcmp(password, correctPass))
            {
                printf("password accepted! here is your package. Triggering solenoid\n");
                 gpio_put(SOLENOID_GPIO, 0);
                 //sleep_ms(10000);
                 
            }
            else
            {
                printf("WRONG PASSWORD! BEGONE, YOU THIEF! TRIGGERING IMAGE CAPTURE\n");
                gpio_put(SOLENOID_GPIO, 1);
               
            }

            
        }


        uint8_t buffer[2];
        uint8_t val = REG_FIFO_COUNT_L ;
        pio_i2c_write_blocking(pio, sm, addr, &val, 1);  
        pio_i2c_read_blocking(pio, sm, addr, buffer, 2);


		uint16_t fifo_len = merge_bytes(buffer[0],buffer[1]);

		if(fifo_len == 1024) {
			//printf("fifo overflow !\n");
			buffer[0] = REG_USER_CTRL;
            buffer[1] = 0x44;
            //i2c_write_blocking(i2c_default, addr, buf, 2, false);
            pio_i2c_write_blocking(pio, sm, addr, buffer, 2);
			continue;
		}
        if(fifo_len >= 8) {

        uint8_t buffer[6];
        uint8_t val = REG_FIFO  ;
        pio_i2c_write_blocking(pio, sm, addr, &val, 1);  
        pio_i2c_read_blocking(pio, sm, addr, &accel_x_h, 1);

        pio_i2c_write_blocking(pio, sm, addr, &val, 1);  
        pio_i2c_read_blocking(pio, sm, addr, &accel_x_l, 1);

        int16_t x_accel= two_complement_to_int(accel_x_h,accel_x_l);
		float x_accel_g = ((float) x_accel);//16384;

        pio_i2c_write_blocking(pio, sm, addr, &val, 1);  
        pio_i2c_read_blocking(pio, sm, addr, &accel_y_h, 1);

        pio_i2c_write_blocking(pio, sm, addr, &val, 1);  
        pio_i2c_read_blocking(pio, sm, addr, &accel_y_l, 1);

        int16_t y_accel= two_complement_to_int(accel_y_h,accel_y_l);
		float y_accel_g = ((float) x_accel);///16384;

        pio_i2c_write_blocking(pio, sm, addr, &val, 1);  
        pio_i2c_read_blocking(pio, sm, addr, &accel_z_h, 1);

        pio_i2c_write_blocking(pio, sm, addr, &val, 1);  
        pio_i2c_read_blocking(pio, sm, addr, &accel_z_l, 1);

        int16_t z_accel= two_complement_to_int(accel_z_h,accel_z_l);
		float z_accel_g = ((float) z_accel);///16384;

        if(count ==0 )
        {
            xavg =  x_accel_g ;
            yavg = y_accel_g; 
            zavg = z_accel_g; 
            ++count;
        }

        //printf("xavg %f	yavg %f	z_accel %f \n", xavg, yavg, zavg);
        printf("x_accel %f	y_accel %f	z_accel %f \n", x_accel_g, y_accel_g, z_accel_g);


        sleep_ms(500);


        //printf("x_accel %f	y_accel %f	z_accel %f \r", x_accel_g, y_accel_g, z_accel_g);

        }


        //mpu6050_read_raw(acceleration, gyro, &temp, pio, sm);

        // These are the raw numbers from the chip, so will need tweaking to be really useful.
        // See the datasheet for more information
        //printf("Acc. X = %x, Y = %x, Z = %x\n", acceleration[0], acceleration[1], acceleration[2]);
        //printf("Gyro. X = %x, Y = %x, Z = %x\n", gyro[0], gyro[1], gyro[2]);
        // Temperature is simple so use the datasheet calculation to get deg C.
        // Note this is chip temperature.
        //printf("Temp. = %f\n", (temp / 340.0) + 36.53);

        sleep_ms(100);
    }



}