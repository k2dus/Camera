#include "global.h"
#include <stdlib.h>

// ===== GLOBAL VARIABLE DEFINITIONS =====
volatile int leftCount = 0;
volatile int rightCount = 0;
volatile int x_pos = 105;
volatile int y_pos = 0;
volatile int heading = 0;
int max_x = 205;
int min_x = 10;
int max_y = 245;
int min_y = 0;
float left_cm  = 0.0f;
float right_cm = 0.0f;
Encoder left_enc;
Encoder right_enc;


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
#define DIST_ECHO_PIN_RIGHT 26

#define LFWD_MOTOR     6
#define LREV_MOTOR     7
#define RFWD_MOTOR     10
#define RREV_MOTOR     11
#define LChannelA    18
#define LChannelB    19
#define RChannelA    20
#define RChannelB    21

#define scoop_PIN 1
#define sorter_PIN 0
// #define GOALIE_PIN 
#define BEAM_SENSOR_PIN 13
#define BEAM_BROKEN_STATE 0
#define GREENFLYWHEEL 17
#define REDFLYWHEEL 16
#define CHAMBER_STOP_PIN 12


//        ┌──────────────────────────────┐
//        │        USB PORT              │
//        └──────────────────────────────┘
//
//LEFT SIDE                         RIGHT SIDE
//────────                         ──────────
//GP0   scoop_PIN              VBUS
//GP1   sorter_PIN             VSYS
//GND                          GND
//GP2   MOTOR_LFWD_MOTOR         3V3_EN
//GP3   MOTOR_LREV_MOTOR         3V3(OUT)
//GP4   MOTOR_RFWD_MOTOR         ADC_VREF
//GP5   MOTOR_RREV_MOTOR         GP28  DIST_ECHO_PIN
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

// Distance sensor pulse tracking for interrupt-driven collision detection
volatile uint64_t pulse_start_left = 0;
volatile uint64_t pulse_start_right = 0;
volatile bool COLLISION = false;
volatile int COLLISION_PIN = 0; //aka none
volatile int left_distance_cm = 0;
volatile int right_distance_cm = 0;
volatile uint8_t collision_hits_left = 0;
volatile uint8_t collision_hits_right = 0;
volatile int L_collision_count = 0;
volatile int R_collision_count = 0;

MotorNoEncoder motor;
TCS34725 sensor;
Servo scoop;
Servo sorter;
Servo chamber_stop;
Compass compass;

static HCSR04 dist_left_sensor;
static HCSR04 dist_right_sensor;

int enemy_goal_x = 105; // heading (degrees, 0-360) recorded at startup when facing enemy goal
int enemy_goal_y = 365;
int own_goal_x = 105;
int own_goal_y = 0;
bool beam_was_broken = false;
bool searching = true;
struct repeating_timer timer;
int gb_in_chamber = 0;
int red_in_chamber = 0;

void collisionDetected();

// Poll both distance sensors sequentially. Returns true if both readings succeeded.
bool read_distances_cm(float *left_cm, float *right_cm) {
    bool left_ok  = hcsr04_distance_cm(&dist_left_sensor,  left_cm);
    sleep_ms(10);
    bool right_ok = hcsr04_distance_cm(&dist_right_sensor, right_cm);
    return left_ok && right_ok;
}

void sleepcheck(int x){
    if (x <= 0) {
        return;
    }

    int remaining_ms = x;
    while (remaining_ms > 0) {
        read_distances_cm(&left_cm, &right_cm);
        if ((left_cm < 10 || right_cm < 10)) {
            printf("collision");
            collisionDetected();\
            break;
        }

        //BOUNDARY CHECK
        
        // if ((x_pos < min_x && -90 < heading) || (x_pos > max_x && heading > 90)){
        //     printf("border trigger");
        //     turn(normalize_heading(heading + 90));
        // }
        // else if ((x_pos < min_x && heading < -90) || (x_pos > max_x && heading < 90)){
        //     printf("border trigger");
        //     turn(normalize_heading(heading - 90));
        // }
        
        int slice_ms = (remaining_ms > 100) ? 100 : remaining_ms;
        sleep_ms(slice_ms);
        remaining_ms -= slice_ms;
    }
}


#define DISTANCE_THRESHOLD_MM 100

bool timer_callback(struct repeating_timer *t) {
    // This callback runs every 70ms to check if a collision has been detected via distance sensors
    // gpio_put(DIST_TRIGGER_PIN, true);
    // sleep_us(10);
    // gpio_put(DIST_TRIGGER_PIN, false);
    
    heading = compass_get_relative_heading(&compass);
    float left_enc_del = -1 * encoder_delta_cm(&left_enc);
    float right_enc_del = encoder_delta_cm(&right_enc);
    float delta_cm = (left_enc_del + right_enc_del) / 2.0f;
    float heading_rad = heading * (float)M_PI / 180.0f;
    leftCount += left_enc_del;
    rightCount += right_enc_del;
    x_pos += (int)(delta_cm * cosf(heading_rad));
    y_pos += (int)(delta_cm * sinf(heading_rad));
    return true; // keep repeating
}
// GPIO interrupt handler for rear distance sensor (echo pin 26)
// void dist_callback(uint gpio, uint32_t events) {
//     printf("10");
//     if (events & GPIO_IRQ_EDGE_RISE) {
//         printf("1");
//         if (gpio == DIST_ECHO_PIN) {
//             printf("2");
//             pulse_start_left = time_us_64();
//         } else if (gpio == DIST_ECHO_PIN_RIGHT) {
//             printf("3");
//             pulse_start_right = time_us_64();
//         }
//         return;
//     } else if (events & GPIO_IRQ_EDGE_FALL) {
//         printf("4");
//         uint64_t start = (gpio == DIST_ECHO_PIN) ? pulse_start_left : pulse_start_right;
//         if (start == 0) return;
//         printf("5");

//         uint64_t duration = time_us_64() - start;
//         int distance = (int)(((float)duration / 2.0f) / 29.1f);
//         if (gpio == DIST_ECHO_PIN) {
//             printf("6");
//             left_distance_cm = distance;
//             pulse_start_left = 0;
//             printf("Left distance: %d cm\n", left_distance_cm);
//             if (distance < 15) {
//                 printf("7");
//                 L_collision_count++;
//                 if(L_collision_count >= 2) {
//                     COLLISION = true;
//                     COLLISION_PIN = DIST_ECHO_PIN;
//                 }
//             } else{ L_collision_count = 0;}
//         } else {
//             printf("8");
//             right_distance_cm = distance;
//             pulse_start_right = 0;
//             printf("Right distance: %d cm\n", right_distance_cm);
//             if (distance < 15) {
//                 printf("9");
//                 R_collision_count++;
//                 if(R_collision_count >= 2) {
//                     COLLISION = true;
//                     COLLISION_PIN = DIST_ECHO_PIN_RIGHT;
//                 }
//             } else { R_collision_count = 0;}
//         }
//     }
// }

// // ===== HELPER FUNCTIONS =====
// void init_pins() { //Initialize GPIO pins for encoders and motors
//     //Initialize Encoders
//     gpio_set_function(LREV_MOTOR, GPIO_FUNC_PWM);
//     gpio_set_function(LFWD_MOTOR, GPIO_FUNC_PWM);
//     gpio_set_function(RFWD_MOTOR, GPIO_FUNC_PWM);
//     gpio_set_function(RREV_MOTOR, GPIO_FUNC_PWM);
// }
#define TARGET 3
#define EXTRA_SEARCH_TIMEOUT_MS 12000


// Field geometry (cm): 7ft wide field, centered 3ft-wide goals.
#define FIELD_WIDTH_CM 213.36f
#define GOAL_WIDTH_CM 91.44f
#define FIELD_CENTER_X_CM (FIELD_WIDTH_CM * 0.5f)
#define GOAL_HALF_WIDTH_CM (GOAL_WIDTH_CM * 0.5f)
#define GOAL_MIN_X_CM (FIELD_CENTER_X_CM - GOAL_HALF_WIDTH_CM)
#define GOAL_MAX_X_CM (FIELD_CENTER_X_CM + GOAL_HALF_WIDTH_CM)

// //----------------STATE MACHINES----------------------

typedef void (*StateFunc)(void);
StateFunc current_state;


static const char *captured_ball_label = NULL;
static int chamber_ball_count = 0;

// Forward declarations
void state_initial(void);
void state_goalie(void);
void state_capture(void);
void state_deposit(void);


// // States in State Machine
// void state_initial(void) {// Rush towards middle line
//     move(1, 91, 80);
//     // transfer to captu
//     current_state = state_capture;
// }
// void state_goalie(void) {// Move back and forth in front of goal
//     move(0, 107, 90);
//     turn(1, 90, 90);
//     while(current_state == state_goalie){
//         move(1,48, 90);
//         sleepcheck(500);
//         move(0, 91, 90);
//         sleepcheck(500);
//     }
// //    if (/* trigger */) current_state = state_search;
// }


static bool is_ball(const char *label) {
    return label != NULL &&
           (strcmp(label, "RED") == 0 ||
            strcmp(label, "BLUE") == 0 ||
            strcmp(label, "GREEN") == 0);
}

int angle_diff(int target, int current) {
    int diff = target - current;
    while (diff > 180) diff -= 360;
    while (diff < -180) diff += 360;
    return diff;
}

void turn(float desiredHeading) {
    while (1) {
        //printf("xpos: %d", x_pos);
        //printf("ypos: %d", y_pos);
        printf("heading: %f \n", compass_get_relative_heading(&compass));
        int current = compass_get_relative_heading(&compass);
        //printf("current: %d:", current);
        int diff = angle_diff(desiredHeading, current);
        //printf("diff: %d", diff);

        if (abs(diff) <= 10) {
            break; // close enough
        }
        if (diff > 0) {
            // turn RIGHT
            pwm_set_gpio_level(LREV_MOTOR, 0);
            pwm_set_gpio_level(LFWD_MOTOR, 100);
            pwm_set_gpio_level(RFWD_MOTOR, 0);
            pwm_set_gpio_level(RREV_MOTOR, 100);
        } else {
            // turn LEFT
            pwm_set_gpio_level(LREV_MOTOR, 100);
            pwm_set_gpio_level(LFWD_MOTOR, 0);
            pwm_set_gpio_level(RFWD_MOTOR, 100);
            pwm_set_gpio_level(RREV_MOTOR, 0);
        }

        sleep_ms(15);
    }
    printf("STOP ALL THE MOTORS PLEASE PLEASE PLEASE PLEASe");
    // STOP motors
    pwm_set_gpio_level(LREV_MOTOR, 0);
    pwm_set_gpio_level(LFWD_MOTOR, 0);
    pwm_set_gpio_level(RFWD_MOTOR, 0);
    pwm_set_gpio_level(RREV_MOTOR, 0);

    encoder_delta_cm(&left_enc);
    encoder_delta_cm(&right_enc);
}

void initializeEverything() {
    // INIT
    stdio_init_all();
    sleep_ms(1500);

    gpio_init(GREENFLYWHEEL);
    gpio_set_dir(GREENFLYWHEEL, GPIO_OUT);
    gpio_init(REDFLYWHEEL);
    gpio_set_dir(REDFLYWHEEL, GPIO_OUT);
    gpio_put(GREENFLYWHEEL, 1);
    gpio_put(REDFLYWHEEL, 1);

    motor_noencoder_init(
        &motor,
        LFWD_MOTOR,
        RFWD_MOTOR,
        LREV_MOTOR,
        RREV_MOTOR,
        MOTOR_NOENCODER_NO_FAULT_PIN,
        MOTOR_NOENCODER_NO_FAULT_PIN,
        1000,
        300,
        150
    );
    motor_noencoder_stop_all(&motor);
    sleep_ms(1000);

    // ENCODER INIT
    

    // GPIO INIT
    sleep_ms(1000);

    
    // // ENCODER INIT
    // encoder_init(&left_enc, LChannelA, LChannelB);
    // encoder_init(&right_enc, RChannelA, RChannelB);

    // Initialize shared camera/color-sensor bus and SPI camera interface
    setup();
    camera_bus_diagnostic();
    // BEAM SENSOR CODE
    gpio_init(BEAM_SENSOR_PIN);
    gpio_set_dir(BEAM_SENSOR_PIN, GPIO_IN);
    gpio_pull_up(BEAM_SENSOR_PIN);


    // Distance sensor polling init (no interrupts)
    hcsr04_init(&dist_left_sensor,  DIST_TRIGGER_PIN, DIST_ECHO_PIN,       0);
    hcsr04_init(&dist_right_sensor, DIST_TRIGGER_PIN, DIST_ECHO_PIN_RIGHT, 0);

    // COLOR SENSOR INIT
    if (!tcs34725_init(&sensor, COLOR_I2C_PORT, COLOR_SENSOR_ADDR)) {
        printf("Color sensor init failed\n");
        while (true) {
            sleep_ms(1000);
        }
    }


    uint8_t sensor_id = 0;
    if (tcs34725_get_sensor_id(&sensor, &sensor_id)) {
        printf("Color sensor ID: 0x%02X\n", sensor_id);
    }


    // SERVOS INIT
    servo_init(&scoop, scoop_PIN);
    servo_init(&sorter, sorter_PIN);
    servo_init(&chamber_stop, CHAMBER_STOP_PIN);
    servo_move(&scoop, 0.0f);
    servo_move(&sorter, 0.0f);
    servo_move(&chamber_stop, 0.0f);

    ak8963_vector_t COMPASS_OFFSET = {
        .x = 30.335f,
        .y = 1.235f,
        .z = -18.330f
    };

    ak8963_vector_t COMPASS_SCALE = {
        .x = 1.215f,
        .y = 0.895f,
        .z = 0.944f
    };
    const float lubbock_declination_deg = -5.44f;
    // printf("Starting calibration — rotate robot slowly 360+ degrees...\n");
    // ak8963_vector_t cal_offset, cal_scale;
    // printf("begin soon");
    // sleep_ms(5000);
    // if (compass_calibrate(&compass, &cal_offset, &cal_scale)) {
    //     printf("COMPASS_OFFSET x=%.3f  y=%.3f  z=%.3f\n",
    //         cal_offset.x, cal_offset.y, cal_offset.z);
    //     printf("COMPASS_SCALE  x=%.3f  y=%.3f  z=%.3f\n",
    //         cal_scale.x, cal_scale.y, cal_scale.z);
    // } else {
    //     printf("Calibration failed\n");
    // }
    bool compass_ok = compass_init(&compass, COLOR_I2C_PORT, lubbock_declination_deg);

    if (compass_apply_calibration(&compass, COMPASS_OFFSET, COMPASS_SCALE)) {
        printf("Calibration applied");
        sleep_ms(200);
    }

    compass_init_reference(&compass);
    compass_get_relative_heading(&compass);
    // COMPASS / MAGNETOMETER INIT (commented out for now)
    // if (compass_ok) {
    //     float raw_h = 0.0f;
    //     // Warm up the low-pass filter with a few reads before storing the reference
    //     for (int i = 0; i < 10; i++) {
    //         compass__get_relative_heading(&compass, &raw_h, NULL);
    //         sleep_ms(20);
    //     }
    //     if (compass_read_heading(&compass, &enemy_goal_heading, NULL)) {
    //         printf("Enemy goal heading stored: %.1f deg\n", enemy_goal_heading);
    //     }
    // } else {
    //     printf("Compass init failed\n");
    // }

    

    // NOTE: Distance sensor triggers need regular pulses to work
    // Ensure hcsr04_send_pulse_and_wait() is called periodically in main loop
   
    // CAMERA INIT
    printf("Starting...\n");
    sleepcheck(1000);

    init_cam();  // configure camera registers
    printf("Cam init done\n");
    printf("Camera initialized...\n");

    // gpio_init(LFWD_MOTOR);
    // gpio_set_dir(LFWD_MOTOR, GPIO_OUT);
    // gpio_put(LFWD_MOTOR, false);

    // gpio_init(LREV_MOTOR);
    // gpio_set_dir(LREV_MOTOR, GPIO_OUT);
    // gpio_put(LREV_MOTOR, false);

    // gpio_init(RFWD_MOTOR);
    // gpio_set_dir(RFWD_MOTOR, GPIO_OUT);
    // gpio_put(RFWD_MOTOR, false);

    // gpio_init(RREV_MOTOR);
    // gpio_set_dir(RREV_MOTOR, GPIO_OUT);
    // gpio_put(RREV_MOTOR, false);
    encoder_init(&left_enc, LChannelA, LChannelB);
    encoder_init(&right_enc, RChannelA, RChannelB);

    add_repeating_timer_ms(-100, timer_callback, NULL, &timer);
}

void collisionDetected() {
    printf("Collision detected on LEFT sensor: %d cm\n", left_cm);
    printf("Collision detected on RIGHT sensor: %d cm\n", right_cm);
    motor_noencoder_move(&motor, .5, false, false); // Back up a little
    sleep_ms(500);
    motor_noencoder_stop_all(&motor);
}

void rightPivot() {
    motor_noencoder_move(&motor, .5, true, false); // R1
    sleep_ms(250);
    motor_noencoder_stop_all(&motor);
    sleep_ms(50);
    encoder_delta(&left_enc);
    encoder_delta(&right_enc);
}
// State machine functions
void state_capture(void) { //Search and Capture
    uint32_t search_start_ms = to_ms_since_boot(get_absolute_time());
    BlobDetection blob;
    bool found = findblobs(&blob);
    bool beam_broken = gpio_get(BEAM_SENSOR_PIN) == 0;
    //SEARCHING
    while (!found || !is_ball(blob.label)) {

        printf("begin");
        gpio_put(GREENFLYWHEEL, 1);
        gpio_put(REDFLYWHEEL, 1);
        // gpio_put(GREENFLYWHEEL, 1);
        // gpio_put(REDFLYWHEEL, 1);
        // sleepcheck(120000);
        // gpio_put(GREENFLYWHEEL, 0);
        // gpio_put(REDFLYWHEEL, 0);


        found = findblobs(&blob);
        // Forward burst
        printf("forward burst");
        motor_noencoder_move(&motor, .5, true, true);
        sleepcheck(1000);
        motor_noencoder_stop_all(&motor);
        if (findblobs(&blob) && is_ball(blob.label)) {
            printf("ball found: %s\n", blob.label);
            break;
        }
        if (gpio_get(BEAM_SENSOR_PIN) == 0) {
            printf("Beam BROKEN");
            break; }
       
        // TURN LEFT
       
        printf("turn LEFT");
        motor_noencoder_move(&motor, .5, false, true);
        sleepcheck(500);
        motor_noencoder_stop_all(&motor);
        if (findblobs(&blob) && is_ball(blob.label)) {
            printf("ball found: %s\n", blob.label);
            break;
        }
        if (gpio_get(BEAM_SENSOR_PIN) == 0) {
            printf("beam broken");
            break; }
       
        // Right pivot (8 in a row)
       
        printf("start RIGHT pivots");
        rightPivot();
        if (findblobs(&blob) && is_ball(blob.label)) {
            printf("ball found: %s\n", blob.label);
            break;
        }
        if (gpio_get(BEAM_SENSOR_PIN) == 0) { break; }
       
        rightPivot();
        if (findblobs(&blob) && is_ball(blob.label)) {
            printf("ball found: %s\n", blob.label);
            break;
        }
        if (gpio_get(BEAM_SENSOR_PIN) == 0) { break; }


        rightPivot();
        if (findblobs(&blob) && is_ball(blob.label)) {
            printf("ball found: %s\n", blob.label);
            break;
        }
        if (gpio_get(BEAM_SENSOR_PIN) == 0) { break; }


        rightPivot();
        if (findblobs(&blob) && is_ball(blob.label)) {
            printf("ball found: %s\n", blob.label);
            break;
        }
        if (gpio_get(BEAM_SENSOR_PIN) == 0) { break; }

        rightPivot();
        if (findblobs(&blob) && is_ball(blob.label)) {
            printf("ball found: %s\n", blob.label);
            break;
        }
        if (gpio_get(BEAM_SENSOR_PIN) == 0) { break; }

        rightPivot();
        if (findblobs(&blob) && is_ball(blob.label)) {
            printf("ball found: %s\n", blob.label);
            break;
        }
        if (gpio_get(BEAM_SENSOR_PIN) == 0) { break; }

        //One left Pivot
        motor_noencoder_move(&motor, .5, false, true); // R1
        sleepcheck(350);
        motor_noencoder_stop_all(&motor);
        sleepcheck(50);
        encoder_delta(&left_enc);
        encoder_delta(&right_enc);
        if(findblobs(&blob) && is_ball(blob.label)) {
            printf("ball found %s\n", blob.label);
            break;
        }
        if (gpio_get(BEAM_SENSOR_PIN) == 0) { break; }
        // Search motor run stuff
    }
    time_t seconds;
    seconds = time(NULL);
    motor_noencoder_stop_all(&motor); // STOP ALL
    sleepcheck(500); //Prepare to attack


    //Move towards the ball until the beam is broken
    tcs34725_raw_data_t data;
    bool moving = true;
    int retc = tcs34725_read_raw(&sensor, &data);
    const char *detected = tcs34725_detect_color_from_raw(&data);
    printf("get that thing fr ");
   
    // CAPTURE THE BALL
    while(moving && time(NULL) - seconds < 7){
        motor_noencoder_move(&motor, .4, true, true);
        beam_broken = gpio_get(BEAM_SENSOR_PIN) == 0;
        tcs34725_raw_data_t data;
        int retc = tcs34725_read_raw(&sensor, &data);
        const char *detected = tcs34725_detect_color_from_raw(&data);
        printf("driving towards target\n");
        sleepcheck(50);
        if (beam_broken && !beam_was_broken) { //STOP MOVING, READ
            sleep_ms(250);
            printf("Beam broken\n");
            scoop_up(&scoop);
            sleep_ms(1000);
            motor_noencoder_move(&motor, 0, false, false);
            moving = false;
            printf("the search is over.... Yurah");

            while (detected == NULL && time(NULL) - seconds < 5) {
                retc = tcs34725_read_raw(&sensor, &data);
                detected = tcs34725_detect_color_from_raw(&data);
                if (detected != NULL) {
                    printf("color sensor %s\r\n", detected);

                    if (strcmp(detected, "RED") == 0) { // Just want to track ball count for deposit state
                        red_in_chamber++; }
                    else if ((strcmp(detected, "GREEN") == 0) || (strcmp(detected, "BLUE") == 0)) {
                        gb_in_chamber++; }
                   
                    servo_sort_with(&sorter, detected);
                    break;
                }
            }
            scoop_down(&scoop);
            printf("while loop over");
            sleep_ms(500);
        }
        beam_was_broken = beam_broken;
    } 
    
    motor_noencoder_move(&motor, 0, false, false);
    
    // turn(scoreheading);
    
    gpio_put(GREENFLYWHEEL, 1);
    gpio_put(REDFLYWHEEL, 0);
    sleep_ms(4000);
    servo_chamber_redpass(&chamber_stop);
    sleep_ms(8000);
    servo_chamber_center(&chamber_stop);
    gpio_put(REDFLYWHEEL, 1); // Shut flywheel back off
    red_in_chamber = 0;
    sleepcheck(500);
    // turn(scoreheading);

    gpio_put(REDFLYWHEEL, 1);
    gpio_put(GREENFLYWHEEL, 0);
    sleep_ms(4000);
    servo_chamber_gbpass(&chamber_stop);
    sleep_ms(8000);
    servo_chamber_center(&chamber_stop);
    gpio_put(GREENFLYWHEEL, 1);
    gb_in_chamber = 0;

}
// Returns the smallest angular difference between two headings (0-360), range [0, 180]
float heading_diff(float a, float b) {
    float d = fabsf(a - b);
    if (d > 180.0f) d = 360.0f - d;
    return d;
}

int main() {
    
    // FLYWHEEL INIT
    
    // Initialize hardware
    initializeEverything();

    scoop_down(&scoop);
    servo_chamber_center(&chamber_stop);
    servo_sort("RED");
    current_state = state_capture;
    while(current_state) {
        current_state();
    }
    // --- BLACK LINE DETECTION TEST ---
    // Comment this block out and restore the state machine when done testing.
    // while (true) {
    //     LineDetection line;
    //     detect_boundary_line(&line);
    //     if (line.detected) {
    //         printf("BLACK LINE DETECTED | left=%d center=%d right=%d | top_row=%d\n",
    //                line.left, line.center, line.right, line.line_y);
    //     } else {
    //         printf("no line\n");
    //     }
    //     sleep_ms(200);
    // }
    // --- END TEST (restore below for normal operation) ---
    // current_state = state_capture;
    // while(current_state) { current_state(); }
    // while (true) {
        
    //     if (read_distances_cm(&left_cm, &right_cm)) {
    //         printf("dist L=%.1f cm  R=%.1f cm\n", left_cm, right_cm);
    //     } else {
    //         printf("dist timeout\n");
    //     }
    //     sleep_ms(100);
    // }

    // Main loop
    // while (true) {
    //     //run the current state
    //     // current_state();
    //     // sleep_ms(5000);
    //     // servo_chamber_center(&chamber_stop);
        
    //     if(true) {
    //         gpio_put(GREENFLYWHEEL, 1);
    //         gpio_put(REDFLYWHEEL, 0);
    //         sleep_ms(4000);
    //         servo_chamber_redpass(&chamber_stop);
    //         sleep_ms(8000);
    //         servo_chamber_center(&chamber_stop);
    //         gpio_put(REDFLYWHEEL, 1); // Shut flywheel back off
    //         red_in_chamber = 0;
    //         printf("red test done");
    //     }
    //     if(true) {
    //         gpio_put(REDFLYWHEEL, 1);
    //         gpio_put(GREENFLYWHEEL, 0);
    //         sleep_ms(6000);
    //         servo_chamber_gbpass(&chamber_stop);
    //         sleep_ms(3000);
    //         servo_chamber_center(&chamber_stop);
    //         gpio_put(GREENFLYWHEEL, 1);
    //         sleep_ms(2000);
    //         gb_in_chamber = 0;
    //         printf("green test done");
    //     }
        // printf("working..\r\n");

        // // MOTOR AND ENCODER
        // motor_noencoder_move(&motor, .5, true, true);
        // sleep_ms(200);
        // printf("heading: %d \n "
        //         "x pos: %d \n"
        //         "y_pos: %d \n", heading, x_pos, y_pos);
        // }

    return 0;
}