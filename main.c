#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/i2c.h"

#include "colorsensor.h"
#include "camera.h"
#include "compass.h"
#include "hcsr04.h"
#include "motornoencoder.h"
#include "servo.h"

#define COLOR_I2C_PORT i2c0
#define COLOR_I2C_SDA 8
#define COLOR_I2C_SCL 9
#define COLOR_I2C_FREQ 50000
#define COLOR_SENSOR_ADDR 0x29

// #define PIN_CS   5              // SPI chip select (enable/disable camera)
// #define PIN_MOSI 3              // SPI data out (Pico → Camera)
// #define PIN_MISO 4              // SPI data in (Camera → Pico)
// #define PIN_SCK  2   

#define DIST_TRIGGER_PIN 27
#define DIST_ECHO_PIN 28
#define REBOOT_BUTTON 12
#define MOTOR_LFWD_PIN 6
#define MOTOR_LREV_PIN 7
#define MOTOR_RFWD_PIN 10
#define MOTOR_RREV_PIN 11
#define LEFTENCODERA   18
#define LEFTENCODERB   19
#define RIGHTENCODERA   20
#define RIGHTENCODERB   21


#define scoop_PIN 1
#define sorter_PIN 0
#define BEAM_SENSOR_PIN 13
#define BEAM_BROKEN_STATE 0
#define GREENFLYWHEEL 16
#define REDFLYWHEEL 17







//        ┌──────────────────────────────┐
//        │        USB PORT              │
//        └──────────────────────────────┘
//
//LEFT SIDE                         RIGHT SIDE
//────────                         ──────────
//GP0   scoop_PIN              VBUS
//GP1   sorter_PIN             VSYS
//GND                          GND
//GP2   MOTOR_LFWD_PIN         3V3_EN
//GP3   MOTOR_LREV_PIN         3V3(OUT)
//GP4   MOTOR_RFWD_PIN         ADC_VREF
//GP5   MOTOR_RREV_PIN         GP28  DIST_ECHO_PIN
//GND                          GND
//GP6   FREE                   GP27  DIST_TRIGGER_PIN
//GP7   FREE                   GP26  FREE (ADC)
//GP8   I2C SDA (color)        RUN (reset)
//GP9   I2C SCL (color)        GP22  FREE
//GND                          GND
//GP10  FREE                   GP21  RIGHTENCODERB
//GP11  FREE                   GP20  RIGHTENCODERA
//GP12  REBOOT_BUTTON          GP19  LEFTENCODERB
//GP13  BEAM_SENSOR            GP18  LEFTENCODERA
//GND                          GND
//GP14  OVERCURRENT1           GP17  REDFLYWHEEL
//GP15  OVERCURRENT2           GP16  GREENFLYWHEEL  
int main() {
    // INIT
    stdio_init_all();
    sleep_ms(3000);

    gpio_init(REBOOT_BUTTON);
    gpio_set_dir(REBOOT_BUTTON, GPIO_IN);
    gpio_pull_up(REBOOT_BUTTON);

    // Initialize shared camera/color-sensor bus and SPI camera interface.
    setup();

    gpio_init(BEAM_SENSOR_PIN);
    gpio_set_dir(BEAM_SENSOR_PIN, GPIO_IN);
    gpio_pull_up(BEAM_SENSOR_PIN);

    // COLOR SENSOR INIT

    // TCS34725 sensor;
    // if (!tcs34725_init(&sensor, COLOR_I2C_PORT, COLOR_SENSOR_ADDR)) {
    //     printf("Color sensor init failed\n");
    //     while (true) {
    //         sleep_ms(1000);
    //     }
    // }

    // uint8_t sensor_id = 0;
    // if (tcs34725_get_sensor_id(&sensor, &sensor_id)) {
    //     printf("Color sensor ID: 0x%02X\n", sensor_id);
    // }


    // // DISTANCE SENSOR INIT

    // HCSR04 distance_sensor;
    // hcsr04_init(&distance_sensor, DIST_TRIGGER_PIN, DIST_ECHO_PIN, 0);

    // MOTOR INIT

    MotorNoEncoder motor;
    motor_noencoder_init(
        &motor,
        MOTOR_LREV_PIN,
        MOTOR_RFWD_PIN,
        MOTOR_LFWD_PIN,
        MOTOR_RREV_PIN,
        MOTOR_NOENCODER_NO_FAULT_PIN,
        MOTOR_NOENCODER_NO_FAULT_PIN,
        1000,
        300,
        150
    );
    motor_noencoder_stop_all(&motor);

    // // SERVOS INIT

    Servo scoop;
    // Servo sorter;
    servo_init(&scoop, scoop_PIN);
    // servo_init(&sorter, sorter_PIN);
    servo_move(&scoop, 0.0f);
    // servo_move(&sorter, 0.0f);

    bool beam_was_broken = false;

    // COMPASS / MAGNETOMETER INIT
    // const float lubbock_declination_deg = 5.44f;
    // Compass compass;
    // bool compass_ok = compass_init(&compass, COLOR_I2C_PORT, lubbock_declination_deg);
    // if (compass_ok) {
    //     printf("Magnetometer ready\n");
    // } else {
    //     printf("Magnetometer init failed\n");
    // }

    // CAMERA INIT

    // Give user time to open serial monitor before spamming output
    printf("Starting...\n");
    sleep_ms(1000);
   
    init_cam();  // configure camera registers
    printf("Cam init done\n");
    printf("Camera initialized...\n");

    while (true) {
        
        // BOOTSEL
        if (!gpio_get(REBOOT_BUTTON)) {
            printf("Rebooting to bootloader...\n");
            sleep_ms(500);
            reset_usb_boot(0, 0);
        }
        printf("working..\r\n");
     
        // Capture one frame and search for each color in sequence
        findblobs("RED");
        findblobs("GREEN");
        findblobs("BLUE");
        findblobs("YELLOW");
        findblobs("PURPLE");

        // if (compass_ok) {
        //     ak8963_vector_t magnetic;
        //     float heading_true = 0.0f;
        //     if (mpu9250_get_magnetic(&compass.sensor, &magnetic) &&
        //         compass_read_heading(&compass, NULL, &heading_true)) {
        //         printf("MAG | x: %.2f y: %.2f z: %.2f | heading_true: %.1f deg\n",
        //                magnetic.x, magnetic.y, magnetic.z, heading_true);
        //     } else {
        //         printf("MAG read failed\n");
        //     }
        // }


        // motor_noencoder_check_overcurrent(&motor);
        
        // bool beam_broken = gpio_get(BEAM_SENSOR_PIN) == 0;
        
        // while(gpio_get(BEAM_SENSOR_PIN) == 1) {
            // motor_noencoder_move(&motor, 1, true, true);
            // sleep_ms(500);
            // motor_noencoder_move(&motor, 1, false, false);
            // sleep_ms(500);
            // motor_noencoder_move(&motor, 1, false, false);
            // sleep_ms(500);
            // motor_noencoder_move(&motor, 1, true, true);
            // sleep_ms(500);
            // printf("beam noball %d \n" ,gpio_get(BEAM_SENSOR_PIN) == 0);
            // printf("motor noball: %d \n" ,gpio_get(MOTOR_RFWD_PIN) != 0);
            // printf("servo noball: %d \n" ,gpio_get(scoop_PIN) != 0);

            // motor_noencoder_move(&motor, 1, false, true);
            // sleep_ms(3000);
        // }
        // printf("motor noball: %d \n" ,gpio_get(MOTOR_RFWD_PIN) != 0);
        // printf("servo noball: %d \n" ,gpio_get(scoop_PIN) != 0);


        // printf("beam  noball%d \n",gpio_get(BEAM_SENSOR_PIN) == 0);
        // sleep_ms(500);
        // motor_noencoder_move(&motor, 0, true, false);

        // if (beam_broken && !beam_was_broken) {
        //     printf("Beam broken\n");
        //     servo_scoop_with(&scoop);
        // }
        // beam_was_broken = beam_broken;

        // tcs34725_raw_data_t data;
        // if (tcs34725_read_raw(&sensor, &data)) {
        //     const char *detected = tcs34725_detect_color_from_raw(&data);
        //     if(detected != NULL){
        //         printf("color sesnor %s\r\n", detected);
        //         servo_sort_with(&sorter, detected);
        //     }
        //     else {
        //         printf("color sensor NULL");
        //     }
        // } else {
        //     printf("Color sensor read failed\n");
        // }

        //While beam not broken, move forward to find mall
        

        // long distance_mm = 0;
        // if (hcsr04_distance_mm(&distance_sensor, &distance_mm)) {
        //     if(distance_mm < 100){
        //         printf("RUN\r\n");
        //         motor_noencoder_move(&motor, 0.5f, true, true);
        //     } else {
        //         motor_noencoder_stop_all(&motor);
        //     }
        // } else {
        //     printf("Distance: out of range or no echo\n");
        //     motor_noencoder_stop_all(&motor);
        // }

        sleep_ms(500);
    }
}