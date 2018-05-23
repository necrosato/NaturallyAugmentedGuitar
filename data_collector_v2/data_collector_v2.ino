
#define ESP32MPU
#define ESP32

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "MPU6050.h"
#include "I2Cdev.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high
// default sda is 21 and scl is 22 on esp 32

#define AG_MAP_MIN 0
#define AG_MAP_MAX 127
#define AG_MIN -32768
#define AG_MAX 32767
#define AX_CHANNEL 6
#define AY_CHANNEL 7
#define AZ_CHANNEL 8
#define GX_CHANNEL 9
#define GY_CHANNEL 10
#define GZ_CHANNEL 11
int16_t ax = 0;
int16_t ay = 0;
int16_t az = 0;
int16_t gx = 0;
int16_t gy = 0;
int16_t gz = 0;
int16_t ax_prev = 0;
int16_t ay_prev = 0;
int16_t az_prev = 0;
int16_t gx_prev = 0;
int16_t gy_prev = 0;
int16_t gz_prev = 0;

int16_t ax_mapped = 0;
int16_t ay_mapped = 0;
int16_t az_mapped = 0;
int16_t gx_mapped = 0;
int16_t gy_mapped = 0;
int16_t gz_mapped = 0;
int16_t ax_mapped_prev = 0;
int16_t ay_mapped_prev = 0;
int16_t az_mapped_prev = 0;
int16_t gx_mapped_prev = 0;
int16_t gy_mapped_prev = 0;
int16_t gz_mapped_prev = 0;

#define ANALOG_MAP_MIN 1
#define ANALOG_MAP_MAX_0 127
#define ANALOG_MAP_MAX_1 8
#define ANALOG_MAP_MAX_2 13
#define ANALOG_MIN 0
#ifdef ESP32
    #define ANALOG_MAX 4095
#else
    #define ANALOG_MAX 1023
#endif
#define BUTTON_CHANNEL 23   
#define FSR0_CHANNEL 27 // Behind the neck    
#define FSR1_CHANNEL 26 // Foot switch 
#define FSR2_CHANNEL 25 // Pick
#define HOTPOT_CHANNEL 14   
#define SOFTPOT0_CHANNEL 13 
#define SOFTPOT1_CHANNEL 12 
int16_t button = LOW;
int16_t fsr0 = 0;
int16_t fsr1 = 0;
int16_t fsr2 = 0;
int16_t hotpot = 0;
int16_t softpot0 = 0;
int16_t softpot1 = 0;
int16_t fsr0_mapped = 0;
int16_t fsr1_mapped = 0;
int16_t fsr2_mapped = 0;
int16_t hotpot_mapped = 0;
int16_t softpot0_mapped = 0;
int16_t softpot1_mapped = 0;
int16_t button_prev = LOW;
int16_t fsr0_prev = 0;
int16_t fsr1_prev = 0;
int16_t fsr2_prev = 0;
int16_t hotpot_prev = 0;
int16_t softpot0_prev = 0;
int16_t softpot1_prev = 0;
int16_t fsr0_mapped_prev = 0;
int16_t fsr1_mapped_prev = 0;
int16_t fsr2_mapped_prev = 0;
int16_t hotpot_mapped_prev = 0;
int16_t softpot0_mapped_prev = 0;
int16_t softpot1_mapped_prev = 0;

char channel[2] = {0};
char data[4] = {0};

void map_accel() {
    ax_mapped = map(ax, AG_MIN, AG_MAX, AG_MAP_MIN, AG_MAP_MAX);
    ay_mapped = map(ay, AG_MIN, AG_MAX, AG_MAP_MIN, AG_MAP_MAX);
    az_mapped = map(az, AG_MIN, AG_MAX, AG_MAP_MIN, AG_MAP_MAX);
}

void map_gyro() {
    gx_mapped = map(gx, AG_MIN, AG_MAX, AG_MAP_MIN, AG_MAP_MAX);
    gy_mapped = map(gy, AG_MIN, AG_MAX, AG_MAP_MIN, AG_MAP_MAX);
    gz_mapped = map(gz, AG_MIN, AG_MAX, AG_MAP_MIN, AG_MAP_MAX);
}

void map_analogs() {
    fsr0_mapped = fsr0 == 0 ? 0 : map(fsr0, ANALOG_MIN, ANALOG_MAX, ANALOG_MAP_MIN, ANALOG_MAP_MAX_0);
    fsr1_mapped = fsr1 == 0 ? 0 : map(fsr1, ANALOG_MIN, ANALOG_MAX, ANALOG_MAP_MIN, ANALOG_MAP_MAX_0);
    fsr2_mapped = fsr2 == 0 ? 0 : map(fsr2, ANALOG_MIN, ANALOG_MAX, ANALOG_MAP_MIN, ANALOG_MAP_MAX_0);
    hotpot_mapped = hotpot == 0 ? 0 : map(hotpot, ANALOG_MIN, ANALOG_MAX, ANALOG_MAP_MIN, ANALOG_MAP_MAX_2);
    softpot0_mapped = softpot0 == 0 ? 0 : map(softpot0, ANALOG_MIN, ANALOG_MAX, ANALOG_MAP_MIN, ANALOG_MAP_MAX_1);
    softpot1_mapped = softpot1 == 0 ? 0 : map(softpot1, ANALOG_MIN, ANALOG_MAX, ANALOG_MAP_MIN, ANALOG_MAP_MAX_1);
}

void read_accelgyro() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    // map accel and gyro
    map_accel();
    map_gyro();
}

void read_analog_sensors() {
    // read digital button
    button = digitalRead(BUTTON_CHANNEL);

    // read raw analog measurements from device
    fsr0 = analogRead(FSR0_CHANNEL);
    fsr1 = analogRead(FSR1_CHANNEL);
    fsr2 = analogRead(FSR2_CHANNEL);
    hotpot = analogRead(HOTPOT_CHANNEL);
    softpot0 = analogRead(SOFTPOT0_CHANNEL);
    softpot1 = analogRead(SOFTPOT1_CHANNEL);
    // map analogs
    map_analogs();
}

void print_ag_raw_all() {
    // display tab-separated accel/gyro x/y/z values
    Serial.print("a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.print(gz); Serial.print("\t");
    Serial.println();
}

void print_ag_mapped_all() {
    // display tab-separated accel/gyro x/y/z values
    Serial.print("a/g:\t");
    Serial.print(ax_mapped); Serial.print("\t");
    Serial.print(ay_mapped); Serial.print("\t");
    Serial.print(az_mapped); Serial.print("\t");
    Serial.print(gx_mapped); Serial.print("\t");
    Serial.print(gy_mapped); Serial.print("\t");
    Serial.print(gz_mapped); Serial.print("\t");
    Serial.println();
}

void print_analogs_mapped_all() {
    // display tab-separated accel/gyro x/y/z values
    Serial.print("fsr(0, 1, 2), hotpot, softpot0, softpot1:\t");
    Serial.print(fsr0_mapped); Serial.print("\t");
    Serial.print(fsr1_mapped); Serial.print("\t");
    Serial.print(fsr2_mapped); Serial.print("\t");
    Serial.print(hotpot_mapped); Serial.print("\t");
    Serial.print(softpot0_mapped); Serial.print("\t");
    Serial.print(softpot1_mapped); Serial.print("\t");
    Serial.println();
}

void send_channel_data() {
    Serial.print(channel[0]);
    Serial.print(channel[1]);
    Serial.print(data[0]);
    Serial.print(data[1]);
    Serial.print(data[2]);
    Serial.print(data[3]);
    Serial.println();
}

void zero_data() {
    for (int i = 0; i < 4; i++) {
        data[i] = 0;
    }
}

void set_channel(int ch) {
    channel[0] = ch / 10;
    channel[1] = ch % 10;
}

void set_data(int val) {
    zero_data();
    data[0] = val / 1000;
    data[1] = (val % 1000) / 100;
    data[2] = (val % 100) / 10;
    data[3] = val % 10;
}

void send_ag_mapped_all() {
    if (ax_mapped_prev != ax_mapped) {
        set_channel(AX_CHANNEL);
        set_data(ax_mapped);
        send_channel_data();
    }
    ax_prev = ax;
    ax_mapped_prev = ax_mapped;
    if (ay_mapped_prev != ay_mapped) {
        set_channel(AY_CHANNEL);
        set_data(ay_mapped);
        send_channel_data();   
    }
    ay_prev = ay;
    ay_mapped_prev = ay_mapped;
    if (az_mapped_prev != az_mapped) {
        set_channel(AZ_CHANNEL);
        set_data(az_mapped);
        send_channel_data();
    }
    az_prev = az;
    az_mapped_prev = az_mapped;
    if (gx_mapped_prev != gx_mapped) {
        set_channel(GX_CHANNEL);
        set_data(gx_mapped);
        send_channel_data();
    }
    gx_prev = gx;
    gx_mapped_prev = gx_mapped;
    if (gy_mapped_prev != gy_mapped) {
        set_channel(GY_CHANNEL);
        set_data(gy_mapped);
        send_channel_data();
    }
    gy_prev = gy;
    gy_mapped_prev = gy_mapped;
    if (gz_mapped_prev != gz_mapped) {
        set_channel(GZ_CHANNEL);
        set_data(gz_mapped);
        send_channel_data();
    }
    gz_prev = gz;
    gz_mapped_prev = gz_mapped;
}

void send_button() {
    if (button_prev != button) {
        set_channel(BUTTON_CHANNEL);
        set_data(button);
        send_channel_data();
    }
    button_prev = button;
}

void send_analogs_raw_all() {
    // FSR0
    if (fsr0_prev != fsr0) {
        set_channel(FSR0_CHANNEL);
        set_data(fsr0);
        send_channel_data();
    }
    fsr0_prev = fsr0;
    fsr0_mapped_prev = fsr0_mapped;
    // FSR1
    if (fsr1_prev != fsr1) {
        set_channel(FSR1_CHANNEL);
        set_data(fsr1);
        send_channel_data();   
    }
    fsr1_prev = fsr1;
    fsr1_mapped_prev = fsr1_mapped;
    // FSR2
    if (fsr2_prev != fsr2) {
        set_channel(FSR2_CHANNEL);
        set_data(fsr2);
        send_channel_data();
    }
    fsr2_prev = fsr2;
    fsr2_mapped_prev = fsr2_mapped;   
    // HOTPOT
    if (hotpot_prev != hotpot) {
        set_channel(HOTPOT_CHANNEL);
        set_data(hotpot);
        send_channel_data();
    }
    hotpot_prev = hotpot;
    hotpot_mapped_prev = hotpot_mapped;
    // SOFTPOT0
    if (softpot0_prev != softpot0) {
        set_channel(SOFTPOT0_CHANNEL);
        set_data(softpot0);
        send_channel_data();
    }
    softpot0_prev = softpot0;
    softpot0_mapped_prev = softpot0_mapped;
    // SOFTPOT1
    if (softpot1_prev != softpot1) {
        set_channel(SOFTPOT1_CHANNEL);
        set_data(softpot1);
        send_channel_data();
    }
    softpot1_prev = softpot1;
    softpot1_mapped_prev = softpot1_mapped;
}

void send_analogs_mapped_all() {
    // FSR0
    //if (fsr0_mapped_prev != fsr0_mapped) {
    if (fsr0_prev != fsr0) {
        set_channel(FSR0_CHANNEL);
        //set_data(fsr0_mapped);
        set_data(fsr0);
        send_channel_data();
    }
    fsr0_prev = fsr0;
    fsr0_mapped_prev = fsr0_mapped;
    // FSR1
    if (fsr1_mapped_prev != fsr1_mapped) {
        set_channel(FSR1_CHANNEL);
        set_data(fsr1_mapped);
        send_channel_data();   
    }
    fsr1_prev = fsr1;
    fsr1_mapped_prev = fsr1_mapped;
    // FSR2
    if (fsr2_mapped_prev != fsr2_mapped) {
        set_channel(FSR2_CHANNEL);
        set_data(fsr2_mapped);
        send_channel_data();
    }
    fsr2_prev = fsr2;
    fsr2_mapped_prev = fsr2_mapped;   
    // HOTPOT
    if (hotpot_mapped_prev != hotpot_mapped) {
    //if (hotpot_prev != hotpot) {
        set_channel(HOTPOT_CHANNEL);
        set_data(hotpot_mapped);
        //set_data(hotpot);
        send_channel_data();
    }
    hotpot_prev = hotpot;
    hotpot_mapped_prev = hotpot_mapped;
    // SOFTPOT0
    if (softpot0_mapped_prev != softpot0_mapped) {
        set_channel(SOFTPOT0_CHANNEL);
        set_data(softpot0_mapped);
        send_channel_data();
    }
    softpot0_prev = softpot0;
    softpot0_mapped_prev = softpot0_mapped;
    // SOFTPOT1
    if (softpot1_mapped_prev != softpot1_mapped) {
        set_channel(SOFTPOT1_CHANNEL);
        set_data(softpot1_mapped);
        send_channel_data();
    }
    softpot1_prev = softpot1;
    softpot1_mapped_prev = softpot1_mapped;
}

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    //Serial.begin(9600);
    Serial.begin(115200);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    pinMode(BUTTON_CHANNEL, INPUT);
    delay(1000);
}

void loop() {

    // read data
    read_analog_sensors();
    read_accelgyro();

    // send data
    send_button();
    //send_analogs_mapped_all();
    send_analogs_raw_all();
    send_ag_mapped_all();

    delay(50);
}
