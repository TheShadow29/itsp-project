// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
  
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68 if AD0 pin in grounded
MPU6050 mpu;

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
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
float previous_ypr[3];
float send_ypr[3];

struct {
    uint8_t buttons;
    int8_t x;
    int8_t y;
    int8_t wheel;	/* Not yet implemented */
} mouseReport;

uint8_t nullReport[4] = { 0, 0, 0, 0 };
int ind;


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    Serial.begin(9600);
    pinMode(5, INPUT);
    digitalWrite(5, HIGH);
    pinMode(4, INPUT);
    digitalWrite(4, HIGH);
    // initialize device
    mpu.initialize();

    // verify connection
    if(!mpu.testConnection())
    return;
    
    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        return;
    }
    
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here

        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(previous_ypr, &q, &gravity);
}

            
            
                  
                  
                          // if programming failed, don't try to do anything
                          if (!dmpReady) return;
                      
                          // wait for MPU interrupt or extra packet(s) available
                          while (!mpuInterrupt && fifoCount < packetSize) {
                              // other program behavior stuff here
                      
                              // if you are really paranoid you can frequently test in between other
                              // stuff to see if mpuInterrupt is true, and if so, "break;" from the
                              // while() loop to immediately process the MPU data
                          }
                      
                          // reset interrupt flag and get INT_STATUS byte
                          mpuInterrupt = false;
                          mpuIntStatus = mpu.getIntStatus();
                      
                          // get current FIFO count
                          fifoCount = mpu.getFIFOCount();
                      
                          // check for overflow (this should never happen unless our code is too inefficient)
                          if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
                              // reset so we can continue cleanly
                              mpu.resetFIFO();
                          // otherwise, check for DMP data ready interrupt (this should happen frequently)
                          } else if (mpuIntStatus & 0x02) {
                              // wait for correct available data length, should be a VERY short wait
                              while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
                      
                              // read a packet from FIFO
                              mpu.getFIFOBytes(fifoBuffer, packetSize);
                              
                              // track FIFO count here in case there is > 1 packet available
                              // (this lets us immediately read more without waiting for an interrupt)
                              fifoCount -= packetSize;
                      
                                  mpu.dmpGetQuaternion(&q, fifoBuffer);
                                  mpu.dmpGetGravity(&gravity, &q);
                                  mpu.dmpGetYawPitchRoll(previous_ypr, &q, &gravity);
                                  for(int i = 0; i < 3; ++i)
                                  previous_ypr[i]*=180/M_PI;
                          }
                          
                          
            mouseReport.buttons = 0;
            mouseReport.x = 0;
            mouseReport.y = 0;
            mouseReport.wheel = 0;

                
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here

        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            
            for(int i = 0; i < 3; ++i)
            {
              ypr[i]*=180/M_PI;
              if(ypr[i] > 90 && previous_ypr[i] < -90)
              {
                send_ypr[i] = ypr[i] - previous_ypr[i] - 360;
              }
              else if(ypr[i] < -90 && previous_ypr[i] > 90)
              {
                send_ypr[i] = ypr[i] - previous_ypr[i] + 360;
              }
              else
              {
                send_ypr[i] = ypr[i] - previous_ypr[i];
              }
            }
            
          
            
            if(digitalRead(4)==LOW)
            mouseReport.buttons = nullReport[0] = 1;
            else if(digitalRead(5)==LOW)
            mouseReport.buttons = nullReport[0] = 2;            
            else
            mouseReport.buttons = nullReport[0] = 0;
         
           send_ypr[0]*=20;
           send_ypr[2]*=20;
           mouseReport.x = send_ypr[0];
           mouseReport.y = send_ypr[2];
           Serial.write((uint8_t *)&mouseReport, 4);   
	     Serial.write((uint8_t *)&nullReport, 4);
            for(int i = 0; i < 3 ; ++i)
            previous_ypr[i] = ypr[i];
    }
}
