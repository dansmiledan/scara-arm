// PyBotArm Robotic Arm scara robot project 
// JJROBOTS

#include <stdint.h>
#include <math.h>

// Robot configuration
#define ROBOT_ARM1_LENGTH 91.61 //92.84  FISRT SEGMENT  
#define ROBOT_ARM2_LENGTH 105.92 //106.79 SECOND SEGMENT

#define ROBOT_ABSOLUTE_MAX_M1 121 //121 //114 //121   // max degrees
#define ROBOT_ABSOLUTE_MAX_M2 146 //146 //146 //142   // max degress

// This depends on the pulley teeth and microstepping. For 16/72 teeth  Microstepping=16 200*16 = 3200/360º = 8.8888888888*4.5 = 40.0
#define M1_AXIS_STEPS_PER_UNIT 40.0
// This depends on the pulley teeth and microstepping. For 16/62 33/62 teeth  Microstepping=16 200*16 = 3200/360º = 8.8888888888*7.2803 = 64.713
#define M2_AXIS_STEPS_PER_UNIT 64.713
// Steps to mm for axis 3(Z) paso =
#define M3_AXIS_STEPS_PER_UNIT 396

#define AXIS2_AXIS1_correction 33.0/62.0  // Correccion for robot structure (diference between reductions on each axis)

#define M1_M2_STEP_FACTOR (M1_AXIS_STEPS_PER_UNIT/M2_AXIS_STEPS_PER_UNIT)

// THIS VALUES DEPENDS ON THE MOTORS, PULLEYS AND ROBOT CONSTRUCTION
// Maximun motor acceleration in (steps/seg2)/1000 [acceleration in miliseconds] 30 => 30.000 steps/sec2
#define MAX_ACCEL_M1 40  //30
#define MAX_ACCEL_M2 MAX_ACCEL_M1/M1_M2_STEP_FACTOR // 48  //56  (MAX_ACCEL_M1*M2_AXIS_STEPS_PER_UNIT)/M1_AXIS_STEPS_PER_UNIT
#define MAX_ACCEL_M3 30

#define MIN_ACCEL_M1 3
#define MIN_ACCEL_M2 5
#define MIN_ACCEL_M3 4

// Maximun speed in steps/seg (max 32765)
#define MAX_SPEED_M1 16000 //32000
#define MAX_SPEED_M2 MAX_SPEED_M1/M1_M2_STEP_FACTOR //32000 //32000
#define MAX_SPEED_M3 20000

#define MIN_SPEED_M1 1000
#define MIN_SPEED_M2 1000
#define MIN_SPEED_M3 1000

// Define speeds in steps/sec
//#define MAX_SPEED_X_STEPS MAX_SPEED_X*X_AXIS_STEPS_PER_UNIT
//#define MAX_SPEED_Y_STEPS MAX_SPEED_Y*Y_AXIS_STEPS_PER_UNIT

// Servo definitions
// Servo1: Wrist orientation
#define SERVO1_NEUTRAL 1500  // Servo neutral position Gripped angle
#define SERVO1_MIN_PULSEWIDTH 800
#define SERVO1_MAX_PULSEWIDTH 2200
#define SERVO1_RANGE (SERVO1_MAX_PULSEWIDTH-SERVO1_MIN_PULSEWIDTH)

// Servo2: Gripper
#define SERVO2_NEUTRAL 1500  // Servo neutral position
#define SERVO2_MIN_PULSEWIDTH 1100
#define SERVO2_MAX_PULSEWIDTH 1900
#define SERVO2_RANGE (SERVO2_MAX_PULSEWIDTH-SERVO2_MIN_PULSEWIDTH)

#define SERVO_SPEED 3

// UNCOMMENT THIS LINES TO INVERT MOTORS (by default 1)
//#define INVERT_M1_AXIS 1
//#define INVERT_M2_AXIS 1
//#define INVERT_M3_AXIS 1

// Robot
#define ROBOT_MIN_A1 -120.0
#define ROBOT_MIN_A2 -140.0
#define ROBOT_MIN_A3 0
#define ROBOT_MAX_A1 120.0
#define ROBOT_MAX_A2 140.0
#define ROBOT_MAX_A3 120 // in mm
#define ROBOT_AXIS_DEFINITION -90*GRAD2RAD //-1.5708 // - 90 degrees of axis rotation for our robot configuration

// Robot restriction (signo al reves, cambiar)
//-120 => -190,50
//-100 => -190,70
// -80 => -190,90
// -60 => -185,100
// -40 => -180,120
// -20 => -160,140
//  0  => -150,150
//  20 => -140,160
//  40 => -120,170
//  60 => -100,180
//  80 => -90,190
// 100 => -70,190
// 120 => -50,190
//float robot_limits_max[]={ 190, 190, 185, 180, 170, 150, 140, 120, 110, 90, 70,  50,  40};
//float robot_limits_min[]={ -35, -45, -60,-80,-100,-125,-145,-160,-180,-185,-190,-190,-190};

float robot_limits_min[] = { -140, -140, -140, -140, -140, -140, -140, -130, -120, -110,  -95,  -88};
float robot_limits_max[] = {  88, 95, 110, 120, 130, 140, 140, 140, 140, 140, 140, 140};
uint8_t limits_index;

// Initial robot position in angle/mm
// The robot must be set at this position at start time (steps initialization)
#define ROBOT_INITIAL_POSITION_M1 0
#define ROBOT_INITIAL_POSITION_M2 0
#define ROBOT_INITIAL_POSITION_M3 0

#define INITIALIZE_TO_MAXIMUNS 1    // Comment this line if you dont want initialization to maximun extents


#define TELEMETRY "192.168.4.2"

#define MINIMUN_TIMER_PERIOD 32000 // For timer counters 

#define MSGMAXLEN 20  // Max message length. Message from the APP (parameters)
#define NODATA -20000

#define MAX_TRAJECTORY_POINTS 101
#define TRAJECTORY_TOLERANCE_M1 int(M1_AXIS_STEPS_PER_UNIT*2)
#define TRAJECTORY_TOLERANCE_M2 int(M2_AXIS_STEPS_PER_UNIT*2)
#define TRAJECTORY_TOLERANCE_M3 int(M3_AXIS_STEPS_PER_UNIT*2)

#define STOP_TOLERANCE 4  // Tolerance to check if the robot arrive at point

#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

#define GREEN_LED A1
#define RED_LED A2
#define SWITCH_IN 30

// Structure definition
struct angles {
  float A1;
  float A2;
};

struct times {
  uint8_t case_type;
  float t_A;
  float t_D;
  float t_F;
};

// Variable definitions

// Log and Timer variables
long loop_counter;
int16_t slow_loop_counter;
long timeout_counter;
long wait_counter;
int16_t timestamp=0;
int dt;
long timer_old;
long timer_value;
long slow_timer_old;
long slow_timer_value;
long laser_timer_old;
long laser_timer_value;
int debug_counter;
bool enable_udp_output = false;

// kinematic variables
// position, speed and acceleration are in step units
volatile int16_t position_M1;  // This variables are modified inside the Timer interrupts
volatile int16_t position_M2;
volatile int32_t position_M3;
bool working = false;
bool trajectory_processing = false;
bool M1stopping = false;
bool M2stopping = false;
bool M3stopping = false;

int8_t dir_M1;     //(dir=1 positive, dir=-1 negative)
int8_t dir_M2;
int8_t dir_M3;
float target_angleA1;
float target_angleA2;
int16_t target_position_M1;
int16_t target_position_M2;
int32_t target_position_M3;
int16_t diff_M1;
int16_t diff_M2;
int16_t diff_M3;
int16_t speed_M1;   // Real motor speed (change dinamically during movements)
int16_t speed_M2;
int16_t speed_M3;
int16_t config_speed_M1 = MAX_SPEED_M1;    // configured max speed
int16_t config_speed_M2 = MAX_SPEED_M2;
int16_t config_speed_M3 = MAX_SPEED_M3;
int16_t target_speed_M1 = config_speed_M1;       // target max speed for the actual movement                
int16_t target_speed_M2 = config_speed_M2;
int16_t target_speed_M3 = config_speed_M3;

int16_t acceleration_M1;    // Real motor acceleration
int16_t acceleration_M2;    // Real motor acceleration
int16_t acceleration_M3;    // Real motor acceleration
int16_t config_acceleration_M1 = MAX_ACCEL_M1;   // Acceleration configuration
int16_t config_acceleration_M2 = MAX_ACCEL_M2;
int16_t config_acceleration_M3 = MAX_ACCEL_M3;
float target_acceleration_M1 = MAX_ACCEL_M1;        // Acceleration for the actual movement
float target_acceleration_M2 = MAX_ACCEL_M2;
float target_acceleration_M3 = MAX_ACCEL_M3;

int16_t pos_stop_M1;
int16_t pos_stop_M2;
int32_t pos_stop_M3;
int16_t overshoot_compensation = 20;


int16_t actual_angleA1;
int16_t actual_angleA2;
int16_t actual_valueZ = 0;
int16_t actual_valueW = 0;
int16_t actual_valueG = 0;
int16_t actual_distance = 0;


//uint16_t com_pos_M1;
//uint16_t com_pos_M2;
//uint16_t com_pos_M3;
//uint16_t com_speed_M1;
//uint16_t com_speed_M2;
//uint16_t com_speed_M3;

long servo_counter;
int16_t servo_pos1;
int16_t servo_pos2;
bool servo1_ready = false;
bool servo2_ready = false;

int16_t iCH1;
int16_t iCH2;
int16_t iCH3;
int16_t iCH4;
int16_t iCH5;
int16_t iCH6;
int16_t iCH7;
int16_t iCH8;
uint8_t mode = 0;
uint8_t elbow = 0; // SCARA solution

float line_mode_ax;
float line_mode_ay;
int16_t line_mode_az;
float line_mode_tx;
float line_mode_ty;
int16_t line_mode_tz;
uint16_t line_mode_num_steps = 0;
uint16_t line_mode_step = 0;

uint8_t newMessage;
uint8_t MsgBuffer[MSGMAXLEN];

String MAC;

// Trajectory variables
bool trajectory_mode = false;
uint8_t trajectory_num_points = 0;
uint8_t trajectory_point = 0;
float trajectory_vector[MAX_TRAJECTORY_POINTS][5];
uint16_t trajectory_tolerance_M1 = TRAJECTORY_TOLERANCE_M1;
uint16_t trajectory_tolerance_M2 = TRAJECTORY_TOLERANCE_M2;
uint16_t trajectory_tolerance_M3 = TRAJECTORY_TOLERANCE_M3;
void initServo();
void moveServo1(int pwm);
void moveServo2(int pwm);
void timersConfigure();
void timersStart();
void configSpeed(int target_sM1, int target_sM2, int target_sM3);
void configAcceleration(int target_acc1, int target_acc2, int target_acc3);
void setSpeedAcc();
void motorsCalibration();
void setAxis_1_2(float angleA1,float angleA2);
void trajectory_motor_speed_adjust();
void setAxis3(float height);
void InverseKinematic(float x, float y, float len1, float len2, uint8_t elbow, float *A1, float *A2);
void setMotorM1Speed(int16_t tspeed, int16_t dt, int16_t overshoot_comp);
void setMotorM2Speed(int16_t tspeed, int16_t dt, int16_t overshoot_comp);
void setMotorM3Speed(int16_t tspeed, int16_t dt, int16_t overshoot_comp);
int16_t myAbs(int16_t param)
{
  if (param < 0)
    return -param;
  else
    return param;
}

long myAbsLong(long param)
{
  if (param < 0)
    return -param;
  else
    return param;
}

int sign(int val)
{
  if (val < 0)
    return (-1);
  else
    return (1);
}
