#include "PID_v1.h"
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

int MIN_ABS_SPEED = 80;

//#define LOG_INPUT

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//PID
double originalSetpoint = -1.625;
double setpoint = originalSetpoint;
double movingAngleOffset = 1;
double input, output;
//int moveState=0; //0 = balance; 1 = back; 2 = forth
double Kp = 50;
double Kd = 0.57;
double Ki = 119.5;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft =1;
double motorSpeedFactorRight = 1;
//MOTOR CONTROLLER
int ENA = 3;
int IN1 = 5;
int IN2 = 6;
int IN3 = 9;
int IN4 = 10;
int ENB = 11;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

//timers
long time1Hz = 0;
long time5Hz = 0;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}


void setup()
{    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        
        //setup PID
        
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);  
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    delay(2500);
}

char data = 0; 
boolean FB=false,balance=true,CWturn=false,CCWturn=false;
int turnOffset;
double set;

void loop()
{
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    if (Serial.available() > 0){
      data = Serial.read();
      if(data == 'u'){
        set+=0.05;
      }
      if(data == 'U'){
        set-=0.05;
      }
      if(data == 'p'){
        Kp+=0.1;
      }
      if(data == 'o'){
        Kd+=0.01;
      }
      if(data == 'i'){
        Ki+=0.5;
      }
      if(data == 'P'){
        Kp-=0.1;
      }
      if(data == 'O'){
        Kd-=0.01;
      }
      if(data == 'I'){
        Ki-=0.5;
      }

      if(data == 'w'){
        balance=false;
        FB = true;
      }
      if(data == 's'){
        balance=false;
        FB = false;
      }
      if(data == 'x'){
        balance = true;
        CWturn=false;
        CCWturn=false;
      }
      if(data == 'a'){
        CWturn = true;
      }
      if(data == 'd'){
        CCWturn = true;
      }

      if(data == 'y'){
        MIN_ABS_SPEED +=1;
      }
      if(data == 'Y'){
        MIN_ABS_SPEED -=1;
      }
    }

    if(CWturn){
      turnOffset=80;
    }
    else if(CCWturn){
      turnOffset=-80;
    }
    else{
      turnOffset=0;
    }

    pid.SetTunings(Kp,Ki,Kd);

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        //no mpu data - performing PID calculations and output to motors
        
        pid.Compute();
        motorController.move(output+turnOffset,output-turnOffset, MIN_ABS_SPEED);
        
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
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
        
        #ifdef LOG_INPUT
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        Serial.print(Kp);
        Serial.print(' ');
        Serial.print(Ki);
        Serial.print(' ');
        Serial.print(Kd);
        Serial.print(' ');
        Serial.print(set);
        Serial.print(' ');
        Serial.println(MIN_ABS_SPEED);

        if(balance){
          input = ypr[1] * 180/M_PI + set;
        }
        else{
          if(FB){
            input = ypr[1] * 180/M_PI + set + movingAngleOffset;
          }
          else{
            input = ypr[1] * 180/M_PI + set - movingAngleOffset;
          }
        }
   }
}



/////////////////////////TODO////////////////////////////
/*    CHECK MINIMUM PWM SPEED
 *     ATTACH BT
 *     UPDATE PID THROUGH BT
 */