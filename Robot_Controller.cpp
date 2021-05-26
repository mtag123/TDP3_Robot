#include "mbed.h"

//For the solenoid
#define OFF 0
#define ON 1


//For motor control
#define PWM_PERIOD_US 1000 //For setting PWM periods to 1 milliseconds. I made this number up
#define STOP 0
#define FORWARD 1
#define BACKWARD 2
//For line following, use the previous defines and the follwoing
#define LEFT 3
#define RIGHT 4

//For colour detection
#define COLOUR_THRESHOLD 20000 //Will have to tune this value


DigitalOut myled(LED1); // Debug led

//For the colour sensor
I2C i2c(I2C_SDA, I2C_SCL); //pins for I2C communication (SDA, SCL)

int sensor_addr = 41 << 1;

//Set PWMs for controlling the H-bridge for the motor speed
PwmOut PWMmotorLeft(PTA4); //Connect to EN1 of L298N
PwmOut PWMmotorRight(PTA5); //Connect to EN1 of L298N

BusOut leftMotorMode(PTC17,PTC16); //Connect D4 to IN1, D5 to IN2 of L298N
BusOut rightMotorMode(PTC13,PTC12); //Connect D6 to IN3, D7 to IN4 of L298N

DigitalOut solenoid(PTA3); //Switch for the solenoid

//For black line detection
DigitalIn lineSensor1(PTA6);
DigitalIn lineSensor2(PTA7);
DigitalIn lineSensor3(PTA8);

Serial bluetooth(PTE0,PTE1);

bool red_path = false;
bool blue_path = false;

class SolenoidController {
    public:
    bool state;
    
    void off();
    void on();
};

void SolenoidController::off() {
    state = OFF; 
    solenoid = OFF;
}

void SolenoidController::on() {
    state = ON;
    solenoid = ON;
}


class MotorController {
    public:
    int state;
    int speed;
    
    
    void initialize();
    void setSpeed(int pulsewidth_us);
    void setLeftMotorSpeed(int pulsewidth_us);
    void setRightMotorSpeed(int pulsewidth_us);
    void stopMotors();
    void goForward();
    void goBackward();
    void turnLeft();
    void turnRight();  
    void changeDirection(int direction);
    
    private:
    void setLeftMotorMode(int mode);
    void setRightMotorMode(int mode);
};

void MotorController::initialize()
{   
    state = STOP;
    speed = 0;
    PWMmotorLeft.period_us(PWM_PERIOD_US);
    PWMmotorRight.period_us(PWM_PERIOD_US);

}


void MotorController::setLeftMotorSpeed(int pulsewidth_us)
{
    PWMmotorLeft.pulsewidth_us(pulsewidth_us);
}


void MotorController::setRightMotorSpeed(int pulsewidth_us)
{
    PWMmotorRight.pulsewidth_us(pulsewidth_us);
}


void MotorController::setLeftMotorMode(int mode)
{
    leftMotorMode = mode;
}

void MotorController::setRightMotorMode(int mode)
{
    rightMotorMode = mode;
}


void MotorController::stopMotors()
{
    setLeftMotorMode(STOP);
    setRightMotorMode(STOP);
}

void MotorController::goForward()
{
    state = FORWARD;
    
    setLeftMotorMode(FORWARD);
    setRightMotorMode(FORWARD);

    setLeftMotorSpeed(speed);
    setRightMotorSpeed(speed);

}

void MotorController::goBackward()
{
    state =  BACKWARD;
    
    setLeftMotorMode(BACKWARD);
    setRightMotorMode(BACKWARD);

    setLeftMotorSpeed(speed);
    setRightMotorSpeed(speed);

}

void MotorController::turnLeft()
{   
    state = LEFT;
    
    setLeftMotorMode(BACKWARD);
    setRightMotorMode(FORWARD);

    setLeftMotorSpeed(speed);
    setRightMotorSpeed(speed);

}


void MotorController::turnRight()
{
    state = RIGHT;
    
    setLeftMotorMode(FORWARD);
    setRightMotorMode(BACKWARD);

    setLeftMotorSpeed(speed);
    setRightMotorSpeed(speed);
}

void MotorController::changeDirection(int direction) {
    
    switch(direction) {
        
        case STOP:
            stopMotors();
            break;
         
        case FORWARD:
            goForward();
            break;   
        
        case BACKWARD:
            goBackward();
            break;
        
        case LEFT:
            turnLeft();
            break;
            
        case RIGHT:
            turnRight();
            break;
            
        default:
            stopMotors();
            break;
            
    }
}

void MotorController::setSpeed(int pulsewidth_us) {
    speed = pulsewidth_us;   
}

class ColourSensor {
    public:
    bool blue_detected;
    bool red_detected;
    
    void initialize();
    void read();
};


//The colour sensing functions were written by Hannes Tschofenig
void ColourSensor::initialize() {
    
    i2c.frequency(200000);
    
    blue_detected = false;
    red_detected = false;
    
    char id_regval[1] = {146};
    char data[1] = {0};
    i2c.write(sensor_addr,id_regval,1, true);
    i2c.read(sensor_addr,data,1,false);
    
    if (data[0]==68) {
        myled = 0;
        wait (2); 
        myled = 1;
        } else {
        myled = 1; 
    }
    
    char timing_register[2] = {129,0};
    i2c.write(sensor_addr,timing_register,2,false);
    
    char control_register[2] = {143,0};
    i2c.write(sensor_addr,control_register,2,false);
    
    char enable_register[2] = {128,3};
    i2c.write(sensor_addr,enable_register,2,false);
}

void ColourSensor::read() {
    
        char clear_reg[1] = {148};
        char clear_data[2] = {0,0};
        i2c.write(sensor_addr,clear_reg,1, true);
        i2c.read(sensor_addr,clear_data,2, false);
        
        int clear_value = ((int)clear_data[1] << 8) | clear_data[0];
        
        char red_reg[1] = {150};
        char red_data[2] = {0,0};
        i2c.write(sensor_addr,red_reg,1, true);
        i2c.read(sensor_addr,red_data,2, false);
        
        int red_value = ((int)red_data[1] << 8) | red_data[0];
        
        char green_reg[1] = {152};
        char green_data[2] = {0,0};
        i2c.write(sensor_addr,green_reg,1, true);
        i2c.read(sensor_addr,green_data,2, false);
        
        int green_value = ((int)green_data[1] << 8) | green_data[0];
        
        char blue_reg[1] = {154};
        char blue_data[2] = {0,0};
        i2c.write(sensor_addr,blue_reg,1, true);
        i2c.read(sensor_addr,blue_data,2, false);
        
        int blue_value = ((int)blue_data[1] << 8) | blue_data[0];
        
        
        //Detect the colour of the paper
        
        if(red_value >= COLOUR_THRESHOLD) {
            red_detected = true;
        }
        
        else {
            red_detected = false;
        }
        
        if(blue_value >= COLOUR_THRESHOLD) {
            blue_detected = true;
        }
        
        else {
            blue_detected = false;   
        }
    
    
}

class LineFollower {
    public:
    bool lineDetected1;
    bool lineDetected2;
    bool lineDetected3;
    int direction;
    
    void initialize();
    
    void readSensor1();
    void readSensor2();
    void readSensor3();
    void readSensors();
    
    int chooseDirection();
        
};

void LineFollower::initialize() {
    lineDetected1 = false;
    lineDetected2 = false;
    lineDetected3 = false;
    direction = STOP;
}

void LineFollower::readSensor1() {
    lineDetected1 = lineSensor1;   
}

void LineFollower::readSensor2() {
    lineDetected2 = lineSensor2;   
}

void LineFollower::readSensor3() {
    lineDetected3 = lineSensor3;   
}

void LineFollower::readSensors() {
    readSensor1();
    readSensor2();
    readSensor3();   
}

int LineFollower::chooseDirection() {
    
    int sensorData = 0x00 & ((lineDetected1 << 2) + (lineDetected2 << 1) + (lineDetected3));
    sensorData = sensorData & 0x07;
    
    switch(sensorData) {
        
        //000
        case 0x0:
            direction = STOP;
            break;
        
        //001
        case 0x1:
            direction = RIGHT;
            break;
        
        //010
        case 0x2:
            direction = FORWARD;
            break;
        
        //011
        case 0x3:
            direction = RIGHT;
            break;
        
        //100
        case 0x4:
            direction = LEFT;
            break;
        
        //101
        case 0x5:
            if(red_path) {
                direction = LEFT;
            }
            
            if(blue_path) {
                direction = RIGHT;
            }
            
            break;
            
        //110    
        case 0x06:
            direction = RIGHT;
            break;
            
        //111
        case 0x7:
            direction = FORWARD;
            break;
        
        default:
            direction = FORWARD;
            break;
     }       
    return direction;
}


void bluetoothControl(MotorController motorController) {
     bluetooth.baud(9600);
    
    char c = '0';
    char state = 'F';
    int speed = 0;

    while(true) {
        
        c='0';
        
        if(bluetooth.readable()) {
            c = bluetooth.getc();
        }
        
        
        
        switch(c) {
            
            case 'F':
                if(state != 'F') {
                    state = 'F';
                    speed = 400;
                    motorController.setSpeed(speed);
                    motorController.goForward();
                }
                
                else {
                    speed += 100;
                    motorController.setSpeed(speed);
                    motorController.goForward();
                    }
                break;
                
            case 'B':
                if(state != 'B') {
                    state = 'B';
                    speed = 400;
                    motorController.setSpeed(speed);
                    motorController.goBackward();
                }
                
                else {
                    speed += 100;
                    motorController.setSpeed(speed);
                    motorController.goBackward();
                    }
                break;
                
             case 'L':
                if(state != 'L') {
                    state = 'L';
                    speed = 800;
                    motorController.setSpeed(speed);
                    motorController.turnLeft();
                }
                
                else {
                    speed += 100;
                    motorController.setSpeed(speed);
                    motorController.turnLeft();
                    }
                break;       
             
             case 'R':
                if(state != 'R') {
                    state = 'R';
                    speed = 800;
                    motorController.setSpeed(speed);
                    motorController.turnRight();
                }
                
                else {
                    speed += 100;
                    motorController.setSpeed(speed);
                    motorController.turnRight();
                    }
                break;
                
            case 'S':
                state = 'S';
                speed = 0;
                motorController.setSpeed(speed);
                motorController.stopMotors();
                break;     
                        
        }
    }
}



int main() {
    
    //Blink LED to let you know it's on
    myled = 0;
    wait(0.5);
    myled = 1;
    wait(0.5);
    myled = 0;

    bool paper_detected = false;
    
    MotorController motorController;
    SolenoidController solenoidController;
    LineFollower lineFollower;
    ColourSensor colourSensor;
    
    motorController.initialize();
    lineFollower.initialize();
    colourSensor.initialize();
    solenoidController.off();
    
    
    //Start off going straight
    motorController.setSpeed(700);
    motorController.goForward();


    while(true) {
        
        if(bluetooth.readable()) {
            bluetoothControl(motorController);
            }
        
        lineFollower.readSensors();
        motorController.changeDirection(lineFollower.chooseDirection());
        
        colourSensor.read();
        
        //Logic for the solenoid based on colour detected
            
        //Detect the first sheet of paper if blue and pick up the disc
        if(colourSensor.blue_detected && !paper_detected && !solenoidController.state) {
            paper_detected = true;
            blue_path = true;
            solenoidController.on(); 
        }
            
            //Detect the first sheet of paper if red and pick up the disc
        if(colourSensor.red_detected && !paper_detected && !solenoidController.state) {
            paper_detected = true;
            red_path = true;
            solenoidController.on(); 
        }
             
        //Detect the end of the first sheet of paper   
        if(!colourSensor.blue_detected && !colourSensor.red_detected && paper_detected) {
            paper_detected = false;
        }
            
        //Drop the disc once the second blue paper is detected
        if(colourSensor.blue_detected && blue_path && !paper_detected && solenoidController.state) {
            paper_detected = true;
            solenoidController.off();
        }
        
        //Drop the disc once the second red paper is detected   
        if(colourSensor.red_detected && red_path && !paper_detected && solenoidController.state) {
            paper_detected = true;
            solenoidController.off();
        }
                
    }              
        
}
