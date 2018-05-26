#define ESP32MPU
#define ESP32

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"
#include <math.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high
// default sda is 21 and scl is 22 on esp 32

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t accelgyroIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

#define AG_MAP_MIN 0
#define AG_MAP_MAX 127
#define AG_MIN -32768
#define AG_MAX 32767
#define YAW_CHANNEL 3
#define PITCH_CHANNEL 4
#define ROLL_CHANNEL 5
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

int16_t yaw = 0;
int16_t pitch = 0;
int16_t roll = 0;

#define ANALOG_MIN 0
#ifdef ESP32
    #define ANALOG_MAX 4095
#else
    #define ANALOG_MAX 1023
#endif
#define FSR0_CHANNEL 27 // Behind the neck    
#define FSR1_CHANNEL 26 // Foot switch 
#define FSR2_CHANNEL 25 // Pick
#define HOTPOT_CHANNEL 14   
#define SOFTPOT0_CHANNEL 13 
#define SOFTPOT1_CHANNEL 12 
int16_t fsr0 = 0;
int16_t fsr1 = 0;
int16_t fsr2 = 0;
int16_t hotpot = 0;
int16_t softpot0 = 0;
int16_t softpot1 = 0;
int16_t fsr0_prev = 0;
int16_t fsr1_prev = 0;
int16_t fsr2_prev = 0;
int16_t hotpot_prev = 0;
int16_t softpot0_prev = 0;
int16_t softpot1_prev = 0;

char channel[2] = {0};
char data[4] = {0};

void get_ypr() {
    // get current FIFO count
    fifoCount = accelgyro.getFIFOCount();
    // check for overflow (this should never happen unless our code is too inefficient)
    if (fifoCount == 1024) {
        // reset so we can continue cleanly
        accelgyro.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } else {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = accelgyro.getFIFOCount();
        // read a packet from FIFO
        accelgyro.getFIFOBytes(fifoBuffer, packetSize);
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        // quaternion values in easy matrix form: w x y z
        accelgyro.dmpGetQuaternion(&q, fifoBuffer);
        // Euler angles in degrees
        accelgyro.dmpGetEuler(euler, &q);
        // ypr angles in degrees
        accelgyro.dmpGetGravity(&gravity, &q);
        accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);
        /*
        // real acceleration, adjusted to remove gravity
        accelgyro.dmpGetAccel(&aa, fifoBuffer);
        accelgyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        accelgyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        accelgyro.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        yaw = map(ypr[0] * 180/M_PI, -180, 180, 0, 127);
        pitch = map(ypr[1] * 180/M_PI, -90, 90, 0, 127);
        roll = map(ypr[2] * 180/M_PI, -90, 90, 0, 127);
        */
        yaw = int(ypr[0]* 180/M_PI) + 180;
        pitch = int(ypr[1]* 180/M_PI) + 180;
        roll = int(ypr[2]* 180/M_PI) + 180;
    }
}

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

void read_accelgyro() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    // map accel and gyro
    map_accel();
    map_gyro();
}

void read_analog_sensors() {
    // read raw analog measurements from device
    fsr0 = analogRead(FSR0_CHANNEL);
    fsr1 = analogRead(FSR1_CHANNEL);
    fsr2 = analogRead(FSR2_CHANNEL);
    hotpot = analogRead(HOTPOT_CHANNEL);
    softpot0 = analogRead(SOFTPOT0_CHANNEL);
    softpot1 = analogRead(SOFTPOT1_CHANNEL);
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

void send_ypr() {
    set_channel(YAW_CHANNEL);
    set_data(yaw);
    send_channel_data();
    set_channel(PITCH_CHANNEL);
    set_data(pitch);
    send_channel_data();
    set_channel(ROLL_CHANNEL);
    set_data(roll);
    send_channel_data();
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

void send_analogs_raw_all() {
    // FSR0
    if (fsr0_prev != fsr0) {
        set_channel(FSR0_CHANNEL);
        set_data(fsr0);
        send_channel_data();
    }
    fsr0_prev = fsr0;
    // FSR1
    if (fsr1_prev != fsr1) {
        set_channel(FSR1_CHANNEL);
        set_data(fsr1);
        send_channel_data();   
    }
    fsr1_prev = fsr1;
    // FSR2
    if (fsr2_prev != fsr2) {
        set_channel(FSR2_CHANNEL);
        set_data(fsr2);
        send_channel_data();
    }
    fsr2_prev = fsr2;
    // HOTPOT
    if (hotpot_prev != hotpot) {
        set_channel(HOTPOT_CHANNEL);
        set_data(hotpot);
        send_channel_data();
    }
    hotpot_prev = hotpot;
    // SOFTPOT0
    if (softpot0_prev != softpot0) {
        set_channel(SOFTPOT0_CHANNEL);
        set_data(softpot0);
        send_channel_data();
    }
    softpot0_prev = softpot0;
    // SOFTPOT1
    if (softpot1_prev != softpot1) {
        set_channel(SOFTPOT1_CHANNEL);
        set_data(softpot1);
        send_channel_data();
    }
    softpot1_prev = softpot1;
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

    delay(1000);

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = accelgyro.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    accelgyro.setZAccelOffset(1788); // 1688 factory default for my test chip
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        accelgyro.setDMPEnabled(true);
        dmpReady = true;
        // get expected DMP packet size for later comparison
        packetSize = accelgyro.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    delay(1000);
}

void loop() {
    // read data
    read_analog_sensors();
    read_accelgyro();
    get_ypr();
    // send data
    send_analogs_raw_all();
    send_ag_mapped_all();
    send_ypr();
    delay(50);
}
