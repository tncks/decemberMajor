/*
 * module.h
 *
 *  Created on: 2024. 12. 17.
 *      Author: suchan
 */

#ifndef MODULE_H_
#define MODULE_H_
#include "msp.h"
#include "Clock.h"
#define TURN_ON &fsm[0]
#define HALF_INPUT &fsm[1]
#define STAND_BY &fsm[2]
#define RUNNING &fsm[3]
#define FINISH &fsm[4]
//#define R2 &fsm[5]                                                        //#define R3 &fsm[6]//#define LLC &fsm[7]//#define RLC &fsm[8]//#define LC1 &fsm[9]//#define LC2 &fsm[10]//#define LF &fsm[12]//#define FL &fsm[11]//#define RLLC &fsm[13]//#define RRLC &fsm[14]
#define LED_RED 1
#define LED_GREEN (LED_RED << 1)
#define LED_BLUE (LED_RED << 2)
#define FR_RGHT 0x20 ///< Front right blinker LED
#define FR_LEFT 0x01 ///< Front left blinker LED
#define BK_RGHT 0x80 ///< Back right blinker LED
#define BK_LEFT 0x40 ///< Back left blinker LED
#define w2 (W/2) // used for rounding
#define PI 8192   ///< representation of pi radians
#define TWOPI (2*PI) ///< 6.28...
#define XYTOLERANCE 10000      ///< tolerance in x,y goal seeking, 1 cm
#define THETATOLERANCE (4096/90)  ///< tolerance in angle goal seeking, 1 deg
#define N_ 1
#define S_ 2 // directions
#define E_ 3
#define W_ 4
#define N 360     ///< counts/rotation, just one edge of one tach
#define D 70000   ///< wheel diameter 0.0001cm
#define W 140000  ///< wheel base 0.0001 cm
#define C 219910  ///< wheel circumference 0.0001cm
#define NORTH 4096   ///< direction that is north
#define EAST  0      ///< direction that is east
#define SOUTH -4096  ///< direction that is south
#define WEST -8192   ///< direction that is west
#define MAX 14998/3  // 100%
#define HIGH 13000/3 // 80%
#define MED 10000/3   // 50%
#define NMED -10000/3 // -50% (Reverse Medium)
#define LOW 5000/3   // 20%

/*************************************/
/*************************************/

enum Ints{
    TURN_ON_S,
    HALF_INPUT_S,
    STAND_BY_S,
    RUNNING_S,
    FINISH_S
};
enum OdometryCommand{
  STOP,       ///< stop robot
  FORWARDTOX, ///< move forward straight until X is matched within tolerance XYTOLERANCE
  FORWARDTOY, ///< move forward straight until Y is matched within tolerance XYTOLERANCE
  LEFTTOTH,   ///< turn left until theta is matched within tolerance THETATOLERANCE
  RIGHTTOTH   ///< turn right until theta is matched within tolerance THETATOLERANCE
};

enum TachDirection{
  FORWARD, /**< Wheel is making robot move forward */
  STOPPED, /**< Wheel is stopped */
  REVERSE  /**< Wheel is making robot move backward */
};
enum RobotState{
  ISSTOPPED, ///< stopped
  GOFORWARD, ///< going forward
  HARDRIGHT, ///< turning hard right
  HARDLEFT,  ///< turning hard left
  SOFTRIGHT, ///< turning soft right
  SOFTLEFT   ///< turning soft left
};
struct State {
  int16_t leftDuty;
  int16_t rightDuty;
  uint32_t delay;                // Delay in ms
  const struct State *next[8];   // 3-bit input -> 8 next states
};
typedef const struct State State_t;

/*************************************/
/*************************************/




// prototypes

void led_init();
void turn_on_led(int);
void turn_off_led();
void switch_init();
int  read_switch();
void Motor_Init(void);
void Motor_Stop(void);
void Motor_Forward(uint16_t leftDuty, uint16_t rightDuty);
void Motor_Right(uint16_t leftDuty, uint16_t rightDuty);
void Motor_Left(uint16_t leftDuty, uint16_t rightDuty);
void Motor_Backward(uint16_t leftDuty, uint16_t rightDuty);
void my_motor_init(void);
void my_move(uint16_t leftDuty, uint16_t rightDuty);
void my_stop(void);
void my_left_forward();
void my_left_backward();
void my_right_forward();
void my_right_backward();
void call_motor(int16_t leftDuty, int16_t rightDuty);
void Reflectance_Init(void);
uint8_t Reflectance_Read(uint32_t time);
uint8_t Reflectance_Center(uint32_t time);
int32_t Reflectance_Position(uint8_t data);
void Reflectance_Start(void);
uint8_t Reflectance_End(void);
void start_fsm();
void left_ninety(void);
void right_ninety(void);
void line_follow(void);
char step(void);
void search(void);
void run_follow(void);
void PWM_Init1(uint16_t period, uint16_t duty);
void PWM_Init12(uint16_t period, uint16_t duty1, uint16_t duty2);
void my_pwm_init34(uint16_t period, uint16_t duty3, uint16_t duty4);
uint16_t PWM_RobotArmGetDuty2(void);
void PWM_RobotArmDuty1(uint16_t duty1);
uint16_t PWM_RobotArmGetDuty0(void);
uint16_t PWM_RobotArmGetDuty1(void);
void PWM_RobotArmDuty2(uint16_t duty2);
void PWM_RobotArmDuty0(uint16_t duty0);
void PWM_RobotArmInit(uint16_t period, uint16_t duty0, uint16_t duty1, uint16_t duty2);
void PWM_Duty4(uint16_t duty4);
void PWM_Duty3(uint16_t duty3);
void PWM_Init34(uint16_t period, uint16_t duty3, uint16_t duty4);
void PWM_Duty2(uint16_t duty2);
void PWM_Duty1(uint16_t duty1);
uint8_t ReadSensorData(void); // Returns the last read data from the sensor
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void LaunchPad_Output(uint8_t data);
void LaunchPad_LED(uint8_t data);
uint8_t LaunchPad_Input(void);
void LaunchPad_Init(void);
void software_coord_init(int arg_n, int arg_m);
void hardware_coord_init(void);
void Odometry_Init(int32_t initx, int32_t inity, int32_t initTheta);
void Odometry_Update(int32_t LCount, int32_t RCount);
void Odometry_SetPower(uint32_t fast, uint32_t slow);
int32_t Odometry_GetX(void);
int32_t Odometry_GetY(void);
int32_t Odometry_GetAngle(void);
void Odometry_Get(int32_t *x, int32_t *y, int32_t *theta);
void UpdatePosition(void);
void Display(void);
void follows(uint32_t time);
void SoftLeft(void);
void SoftRight(void);
void HardLeft(void);
void HardRight(void);
uint32_t ForwardUntilX(int32_t desiredX);
uint32_t ForwardUntilY(int32_t desiredY);
uint32_t SoftLeftUntilTh(int32_t desiredTh);
void ForwardUntilXStart(int32_t thedesiredX);
uint32_t ForwardUntilXStatus(void);
void ForwardUntilYStart(int32_t thedesiredY);
uint32_t ForwardUntilYStatus(void);
void SoftLeftUntilThStart(int32_t thedesiredTh);
uint32_t  ForwardUntilThStatus(void);
uint32_t SoftLeftUntilTh(int32_t desiredTh);
uint32_t HardLeftUntilTh(int32_t desiredTh);
void HardLeftUntilThStart(int32_t thedesiredTh);
uint32_t SoftRightUntilTh(int32_t desiredTh);
void SoftRightUntilThStart(int32_t thedesiredTh);
uint32_t HardRightUntilTh(int32_t desiredTh);
void HardRightUntilThStart(int32_t thedesiredTh);
uint32_t CheckGoal(void);
void Tachometer_Init(void);
void Tachometer_Get(uint16_t *leftTach, uint16_t *leftDir, int32_t *leftSteps, uint16_t *rightTach, uint16_t *rightDir, int32_t *rightSteps);
int16_t fixed_sin(int32_t theta);
int16_t fixed_cos(int32_t theta);
int32_t fixed_sin2(int32_t theta);
int32_t fixed_cos2(int32_t theta);
void PWM_Init12(uint16_t period, uint16_t duty1, uint16_t duty2);
void PWM_Init1(uint16_t period, uint16_t duty);
void PWM_Duty1(uint16_t duty1);
void PWM_Duty2(uint16_t duty2);
void PWM_Init34(uint16_t period, uint16_t duty3, uint16_t duty4);
void PWM_Duty3(uint16_t duty3);
void PWM_Duty4(uint16_t duty4);
void PWM_RobotArmInit(uint16_t period, uint16_t duty0, uint16_t duty1, uint16_t duty2);
void PWM_RobotArmDuty0(uint16_t duty0);
void PWM_RobotArmDuty1(uint16_t duty1);
void PWM_RobotArmDuty2(uint16_t duty2);
uint16_t PWM_RobotArmGetDuty0(void);
uint16_t PWM_RobotArmGetDuty1(void);
uint16_t PWM_RobotArmGetDuty2(void);
void TimerA0_Init(void(*task)(void), uint16_t period);
void TimerA0_Stop(void);
void TimerA1_Init(void(*task)(void), uint16_t period);
void TimerA1_Stop(void);
void TimerA2_Init(void(*task)(void), uint16_t period);
void TimerA2_Stop(void);
void Blinker_Init(void);
void Blinker_Output(uint8_t data);
void TimerA3Capture_Init01(void(*task0)(uint16_t time), void(*task1)(uint16_t time));
uint32_t HardLeftUntilTh(int32_t desiredTh);
void MoveForwardOneGrid(void);
int DetectIntersection(void);
void Rotate90Degrees(int clockwise);
void SysTick_Init(void);
void SysTick_Wait(uint32_t delay);
void SysTick_Wait10ms(uint32_t delay);
























/*************************************/
/*************************************/
/*************************************/
/*************************************/
/*************************************/
/*************************************/


// Abbreviations:
// OL -> On Line
// L1 -> Off Left 1 (smallest deviation from line)
// NL -> No Line
// LLC -> Left Lost Check (Lost but know we are left of line)
// LC1 -> Lost Check 1 (Very lost)
// LC2 -> Lost Check 2
// FL -> Fully Lost
// State order in &fsm
//
#endif /* MODULE_H_ */
