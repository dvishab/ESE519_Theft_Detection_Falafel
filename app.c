#include "app.h"


#define PASSWORD_INCORRECT_GPIO         3
#define PASSWORD_CORRECT_GPIO           4
#define WIFI_OUT_GPIO                   5
#define SOLENOID_OUT_GPIO               6


#define X_AXIS_PIN 29
#define Y_AXIS_PIN 28
#define Z_AXIS_PIN 27

#define PIN_SDA 22
#define PIN_SCL 23

static int addr_display = 0x3C;


int main()
{
    
    stdio_init_all();
#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
    #warning i2c/mpu6050_i2c example requires a board with I2C pins
    puts("Default I2C pins were not defined");
#else
    printf("Hello, MPU6050! Reading raw data from registers...\n");
#endif

    const char *words[]= {"ALERT!", "OPEN", "WRONG PASS"};

    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &i2c_program);
    i2c_program_init(pio, sm, offset, PIN_SDA, PIN_SCL);

    ssd1306_t disp;
    disp.external_vcc=false;
    ssd1306_init(&disp, 128, 64, 0x3C, i2c1, pio, sm);
    ssd1306_clear(&disp, pio, sm);

    gpio_init(PASSWORD_INCORRECT_GPIO);
    gpio_set_dir(PASSWORD_INCORRECT_GPIO, GPIO_IN);
    gpio_init(PASSWORD_CORRECT_GPIO);
    gpio_set_dir(PASSWORD_CORRECT_GPIO, GPIO_IN);
    gpio_init(WIFI_OUT_GPIO);
    gpio_set_dir(WIFI_OUT_GPIO, GPIO_OUT);
    gpio_put(WIFI_OUT_GPIO, 1);
    gpio_init(SOLENOID_OUT_GPIO);
    gpio_set_dir(SOLENOID_OUT_GPIO, GPIO_OUT);
    
    
    gpio_put(SOLENOID_OUT_GPIO, 1);

    adc_init();

    adc_gpio_init(X_AXIS_PIN);
    adc_gpio_init(Y_AXIS_PIN);
    adc_gpio_init(Z_AXIS_PIN);
    int level = 0;
    float x_pos = 0;
    float y_pos = 0;
    float z_pos = 0;
    float x;
    float y;
    float z;


    while(!stdio_usb_connected());

    bool corrPass = 0;

    bool wifiState = 1;


    while (1) 
    {
            ssd1306_clear(&disp, pio, sm);
            
            x = x_pos;
            y = y_pos;
            z = z_pos;

            adc_select_input(3);
            uint adc_x_raw = adc_read();
            adc_select_input(2);
            uint adc_y_raw = adc_read();
            adc_select_input(1);
            uint adc_z_raw = adc_read();
            
            const float conversion_factor = (1 << 12) - 1;
            uint16_t result = adc_read();
            x_pos = adc_x_raw / conversion_factor;
            y_pos = adc_y_raw / conversion_factor;
            z_pos = adc_z_raw / conversion_factor;
            printf("X-pos: %f, Y-pos: %f, Z-pos: %f \n", x_pos, y_pos, z_pos);
            printf("Level = %d\n", level);

            sleep_ms(500);

            

            if(x > 0.7 || x < 0.3 || y > 0.7 || y < 0.3 || z > 0.7 || z < 0.3){

                printf("SHAKING DETECTED \n");
                
                
                //if(wifiState==1){

                    for(int i = 0; i < 20; i++)
                    {
                    gpio_put(WIFI_OUT_GPIO, 0);
                    gpio_put(WIFI_OUT_GPIO, 1);
                    }

                    wifiState = 0;

               // }
                

                ssd1306_clear(&disp, pio, sm);
                ssd1306_draw_string(&disp, 8, 24, 2, words[0], pio, sm);
                ssd1306_show(&disp, pio, sm);

                //while(!gpio_get(PASSWORD_CORRECT_GPIO));

                sleep_ms(2000);

                
                //gpio_put(SOLENOID_OUT_GPIO, 0);
                //ssd1306_clear(&disp, pio, sm);
                //ssd1306_draw_string(&disp, 8, 24, 2, words[1], pio, sm);
                //ssd1306_show(&disp, pio, sm);

                continue;

                //continue;
                //sleep_ms(800);
                
            }

            if(gpio_get(PASSWORD_INCORRECT_GPIO) == 1)
            {
                corrPass=0;
                printf("INCORRECT PASSWORD \n");

                if(wifiState==1){

                   // for(int i = 0; i < 20; i++)
                   // {
                    gpio_put(WIFI_OUT_GPIO, 0);
                    gpio_put(WIFI_OUT_GPIO, 1);
                    //}

                    wifiState = 0;

                }
                gpio_put(SOLENOID_OUT_GPIO,0);

                ssd1306_clear(&disp, pio, sm);
                ssd1306_draw_string(&disp, 8, 24, 2, words[2], pio, sm);
                ssd1306_show(&disp, pio, sm);

                //gpio_put(WIFI_OUT_GPIO, 0);
               // sleep_ms(100);
                

                //while(!gpio_get(PASSWORD_CORRECT_GPIO));

               // sleep_ms(5000);

               //gpio_put(SOLENOID_OUT_GPIO, 1);

                continue;

            }

            if(gpio_get(PASSWORD_CORRECT_GPIO) == 1 && corrPass==0)
            {
                corrPass=1;
                wifiState=1;
                
                printf("CORRECT PASSWORD \n");

                gpio_put(WIFI_OUT_GPIO, 0);
                gpio_put(SOLENOID_OUT_GPIO, 1);
                ssd1306_clear(&disp, pio, sm);
                ssd1306_draw_string(&disp, 8, 24, 2, words[1], pio, sm);
                ssd1306_show(&disp, pio, sm);

                sleep_ms(5000);

                gpio_put(SOLENOID_OUT_GPIO, 0);

            }

            sleep_ms(100);

        }

}
