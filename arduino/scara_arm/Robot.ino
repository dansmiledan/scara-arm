// PyBotArm Robotic Arm scara robot project 
// JJROBOTS

// STEPPERS MOTOR CONTROL AND KINEMATICS
// SPEED, ACCELERATION AND POSITION CONTROL using M0 internal timers for best accuracy

// We control the speed of the motors with interrupts (TC3,TC5 and TCC2) tested up to 32Khz.
// The position,speed and acc of the motor is controlled at 1Khz (called in the main loop)


ISR(TIMER3_OVF_vect) // replace x by the proper timer
{
	

}


// MOTOR1
// TC5 interrupt
// void TC5_Handler (void) 
// {
//   TC5->COUNT16.INTFLAG.bit.MC0 = 1; //don't change this, it's part of the timer code
//   if (dir_M1 == 0)
//     return;
//   REG_PORT_OUTSET0 = PORT_PA15; // STEP Motor1
//   position_M1 += dir_M1;
//   delayMicroseconds(1);
//   REG_PORT_OUTCLR0 = PORT_PA15; // STEP Motor1
// }

// MOTOR2
// TC3 interrupt  
// void TC3_Handler (void) 
// {
//   TC3->COUNT16.INTFLAG.bit.MC0 = 1; // Interrupt reset
//   if (dir_M2 == 0)
//     return;
//   REG_PORT_OUTSET0 = PORT_PA21; // STEP Motor2
//   position_M2 += dir_M2;
//   delayMicroseconds(1);
//   REG_PORT_OUTCLR0 = PORT_PA21; // STEP Motor2
// }

// MOTOR3
// TCC2 interrupt
// void TCC2_Handler (void) 
// {
//   TCC2->INTFLAG.bit.MC0 = 1; 
//   if (dir_M3 == 0)
//     return;
//   REG_PORT_OUTSET0 = PORT_PA07; // STEP Motor3
//   position_M3 += dir_M3;
//   delayMicroseconds(1);
//   REG_PORT_OUTCLR0 = PORT_PA07; // STEP Motor3
// }

// We use a ramp for acceleration and deceleration
// To calculate the point we should start to decelerate we use this formula:
// stop_position = actual_posicion + (actual_speed*actual_speed)/(2*max_deceleration)
// Input parameters:
//    target_position_x
//    target_speed_x
//    max_acceleration_x

void positionControl(int dt)
{
  //int16_t pos_stop;
  int32_t temp;


  //dt = constrain(dt, 500, 4000); // Limit dt (it should be around 1000 most times)

  // S curve [optional]:
  //int16_t absspeed = abs(speed_M1);
  //if (absspeed<500)
  //  acceleration_M1 = map(absspeed,0,1000,MIN_ACCEL_M1,target_acceleration_M1);
  //else
  //  acceleration_M1 = target_acceleration_M1;

  acceleration_M1 = target_acceleration_M1;
  acceleration_M2 = target_acceleration_M2;
  acceleration_M3 = target_acceleration_M3;
  
  // MOTOR1 control
  // We first check if we need to start decelerating in order to stop at the final position
  // calculate the distance to decelerate until stop at target speed
  // kinematic formula   d = (sqr(v1)-sqr(v0))/(2*a) because v1=0 and acc(a) is negative in deceleration => d = sqr(v1)/2a
  // We calculate the stop position and check if we need to start decelerating right now...
  temp = (long)speed_M1 * speed_M1;
  temp = temp / (2000 * (long)acceleration_M1);
  pos_stop_M1 = position_M1 + sign(speed_M1) * temp;
  if (target_position_M1 > position_M1){ // Positive move
    if (pos_stop_M1 >= target_position_M1){  // Start decelerating?      
      int16_t overshoot = pos_stop_M1-target_position_M1;
      if (overshoot>overshoot_compensation) {//Serial.println("OV1"); 
        setMotorM1Speed(0, dt, overshoot/overshoot_compensation);  // Increase decelaration a bit (overshoot compensation)
      }
      M1stopping=true;
      setMotorM1Speed(0, dt,0);         // The deceleration ramp is done inside the setSpeed function
    }
    else{
      M1stopping=false;    
      setMotorM1Speed(target_speed_M1, dt,0);   // The aceleration ramp is done inside the setSpeed function
    }
  }
  else{   // Negative move
    if (pos_stop_M1 <= target_position_M1){  // Start decelerating?
      int16_t overshoot = target_position_M1-pos_stop_M1;
      if (overshoot>overshoot_compensation) { //Serial.println("OV2");   
        setMotorM1Speed(0, dt, overshoot/overshoot_compensation);  // Increase decelaration a bit (overshoot compensation)
      } 
      M1stopping=true;
      setMotorM1Speed(0, dt,0);
    } else {
      M1stopping=false;
      setMotorM1Speed(-target_speed_M1, dt,0);
    }
  }

  // MOTOR2 CONTROL
  temp = (long)speed_M2 * speed_M2;
  temp = temp / (2000 * (long)acceleration_M2);
  pos_stop_M2 = position_M2 + sign(speed_M2) * temp;
  if (target_position_M2 > position_M2) // Positive move
  {
    if (pos_stop_M2 >= target_position_M2){  // Start decelerating?
      int16_t overshoot = pos_stop_M2-target_position_M2;
      if (overshoot>overshoot_compensation) //Serial.println("OV3");
        setMotorM2Speed(0, dt, overshoot/overshoot_compensation);  // Increase decelaration a bit (overshoot compensation)
      M2stopping=true;
      setMotorM2Speed(0, dt,0);         // The deceleration ramp is done inside the setSpeed function
      }
    else{
      M2stopping=false;
      setMotorM2Speed(target_speed_M2, dt,0);   // The aceleration ramp is done inside the setSpeed function
    }
  }
  else   // Negative move
  {
    if (pos_stop_M2 <= target_position_M2){  // Start decelerating?
      int16_t overshoot = target_position_M2-pos_stop_M2;
      if (overshoot>overshoot_compensation) //Serial.prointln("OV4");
        setMotorM2Speed(0, dt, overshoot/overshoot_compensation);  // Increase decelaration a bit (overshoot compensation)    
      M2stopping=true;
      setMotorM2Speed(0, dt,0);
      }
    else{
      M2stopping=false;
      setMotorM2Speed(-target_speed_M2, dt,0);
    }
  }

  // MOTOR3 CONTROL
  temp = (long)speed_M3 * speed_M3;
  temp = temp / (2000 * (long)acceleration_M3);
  pos_stop_M3 = position_M3 + sign(speed_M3) * temp;
  if (target_position_M3 > position_M3) // Positive move
  {
    if (pos_stop_M3 >= target_position_M3){  // Start decelerating?
      int16_t overshoot = pos_stop_M3-target_position_M3;
      if (overshoot>overshoot_compensation) //Serial.prointln("OV5");
        setMotorM3Speed(0, dt, overshoot/overshoot_compensation);  // Increase decelaration a bit (overshoot compensation)
      M3stopping=true;
      setMotorM3Speed(0, dt,0);         // The deceleration ramp is done inside the setSpeed function
      }
    else{
      M3stopping=false;
      setMotorM3Speed(target_speed_M3, dt,0);   // The aceleration ramp is done inside the setSpeed function
    }
  }
  else   // Negative move
  {
    if (pos_stop_M3 <= target_position_M3){  // Start decelerating?
      int16_t overshoot = target_position_M3-pos_stop_M3;
      if (overshoot>overshoot_compensation) //Serial.prointln("OV6");    
        setMotorM3Speed(0, dt, overshoot/overshoot_compensation);  // Increase decelaration a bit (overshoot compensation)
      M3stopping=true;
      setMotorM3Speed(0, dt,0);
      }
    else{
      M3stopping=false;
      setMotorM3Speed(-target_speed_M3, dt,0);
    }
  }
    
  //diff_M1 = myAbs(target_position_M1 - position_M1);
  //diff_M2 = myAbs(target_position_M2 - position_M2);
  //diff_M3 = myAbs(target_position_M3 - position_M3);
  //if ((diff_M1<5)&&(diff_M2<5)&&(diff_M3<5))
  //  working = false;
  //else
  //  working = true;
  if ((dir_M1==0)&&(dir_M2==0)&&(dir_M3==0))
    working = false;   // Robot is stopped
  else
    working = true;    // Robot is moving...
  //CLR(PORTF,3); // for external timing debug

  
}


// Speed could be positive or negative
void setMotorM1Speed(int16_t tspeed, int16_t dt, int16_t overshoot_comp)
{
  long timer_period;
  int16_t accel;

  // Limit max speed
  tspeed = constrain(tspeed,-MAX_SPEED_M1,MAX_SPEED_M1);

  // We limit acceleration => speed ramp
  overshoot_comp = constrain(overshoot_comp,-10,10);
  accel = (((long)acceleration_M1 * dt) / 1000)+overshoot_comp;
  if (((long)tspeed - speed_M1) > accel)
    speed_M1 += accel;
  else if (((long)speed_M1 - tspeed) > accel)
    speed_M1 -= accel;
  else{
    //if (speed_M1!=tspeed)
    //  Serial.println("->M1FS");
    speed_M1 = tspeed;
  }

  // Check if we need to change the direction pins
  if ((speed_M1 == 0) && (dir_M1 != 0)){
    dir_M1 = 0;
    Serial.println("->M1 STOP");
  }
  else if ((speed_M1 > 0) && (dir_M1 != 1)) {
#ifdef INVERT_M1_AXIS 
    // REG_PORT_OUTCLR0 = PORT_PA20;
#else
    // REG_PORT_OUTSET0 = PORT_PA20;
#endif
    dir_M1 = 1;
  }
  else if ((speed_M1 < 0) && (dir_M1 != -1)){
#ifdef INVERT_M1_AXIS 
    // REG_PORT_OUTSET0 = PORT_PA20;
#else
    // REG_PORT_OUTCLR0 = PORT_PA20;
#endif
    dir_M1 = -1;
  }

  if (speed_M1 == 0)
    timer_period = MINIMUN_TIMER_PERIOD;
  else if (speed_M1 > 0)
    timer_period = 3000000 / speed_M1;   // 3Mhz timer 
  else
    timer_period = 3000000 / -speed_M1;

  if (timer_period > MINIMUN_TIMER_PERIOD)   // Check for minimun speed (maximun period without overflow)
    timer_period = MINIMUN_TIMER_PERIOD;

  // Change timer
  // TODO
  // TC5->COUNT16.CC[0].reg = (uint16_t) timer_period;
  // while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY); // wait for sync...
  
  // // Check  if we need to reset the timer...
  // if (TC5->COUNT16.COUNT.reg > (uint16_t)timer_period){
  //   TC5->COUNT16.COUNT.reg = (uint16_t)timer_period-4;
  //   while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY); // wait for sync...
  // } 
}

// Speed could be positive or negative
void setMotorM2Speed(int16_t tspeed, int16_t dt, int16_t overshoot_comp)
{
  long timer_period;
  int16_t accel;
  
  tspeed = constrain(tspeed,-MAX_SPEED_M2,MAX_SPEED_M2); // Limit max speed
  
  // We limit acceleration => speed ramp
  overshoot_comp = constrain(overshoot_comp,-10,10);
  accel = (((long)acceleration_M2 * dt) / 1000)+overshoot_comp; // We divide by 1000 because dt are in microseconds
  if (((long)tspeed - speed_M2) > accel) // We use long here to avoid overflow on the operation
    speed_M2 += accel;
  else if (((long)speed_M2 - tspeed) > accel)
    speed_M2 -= accel;
  else{
    //if (speed_M2!=tspeed)
    //  Serial.println("->M2FS");
    speed_M2 = tspeed;
  }

  // Check if we need to change the direction pins
  if ((speed_M2 == 0) && (dir_M2 != 0)){
    dir_M2 = 0;
    Serial.println("->M2 STOP");
  }
  else if ((speed_M2 > 0) && (dir_M2 != 1)){
#ifdef INVERT_M2_AXIS
    // REG_PORT_OUTCLR0 = PORT_PA06; // M2-DIR
#else
    // REG_PORT_OUTSET0 = PORT_PA06;
#endif
    dir_M2 = 1;
  }
  else if ((speed_M2 < 0) && (dir_M2 != -1)){
#ifdef INVERT_M2_AXIS
    // REG_PORT_OUTSET0 = PORT_PA06;  // M2-DIR 
#else
    // REG_PORT_OUTCLR0 = PORT_PA06;
#endif
    dir_M2 = -1;
  }
  if (speed_M2 == 0)
    timer_period = MINIMUN_TIMER_PERIOD;
  else if (speed_M2 > 0)
    timer_period = 3000000 / speed_M2; // 3Mhz timer (48Mhz / preescaler=16 = 3Mhz)
  else
    timer_period = 3000000 / -speed_M2;
  if (timer_period > MINIMUN_TIMER_PERIOD)   // Check for minimun speed (maximun period without overflow)
    timer_period = MINIMUN_TIMER_PERIOD;

  // Change timer
  // TODO
  // TC3->COUNT16.CC[0].reg = (uint16_t) timer_period;
  // while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY); // wait for sync...
 
  // // Check  if we need to reset the timer...
  // if (TC3->COUNT16.COUNT.reg > (uint16_t)timer_period){
  //   TC3->COUNT16.COUNT.reg = (uint16_t)timer_period-4;
  //   while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY); // wait for sync...
  // }
}

// Speed could be positive or negative
void setMotorM3Speed(int16_t tspeed, int16_t dt, int16_t overshoot_comp)
{
  long timer_period;
  int16_t accel;
  
  tspeed = constrain(tspeed,-MAX_SPEED_M3,MAX_SPEED_M3); // Limit max speed
  // We limit acceleration => speed ramp
  overshoot_comp = constrain(overshoot_comp,-10,10);
  accel = (((long)acceleration_M3 * dt) / 1000)+overshoot_comp; // We divide by 1000 because dt are in microseconds
  if (((long)tspeed - speed_M3) > accel) // We use long here to avoid overflow on the operation
    speed_M3 += accel;
  else if (((long)speed_M3 - tspeed) > accel)
    speed_M3 -= accel;
  else
    speed_M3 = tspeed;

  // Check if we need to change the direction pins
  if ((speed_M3 == 0) && (dir_M3 != 0))
    dir_M3 = 0;
  else if ((speed_M3 > 0) && (dir_M3 != 1)){
#ifdef INVERT_M3_AXIS
    // REG_PORT_OUTCLR0 = PORT_PA18; // M3-DIR 
#else
    // REG_PORT_OUTSET0 = PORT_PA18;
#endif
    dir_M3 = 1;
  }
  else if ((speed_M3 < 0) && (dir_M3 != -1)){
#ifdef INVERT_M3_AXIS
    // REG_PORT_OUTSET0 = PORT_PA18;  // M3-DIR 
#else
    // REG_PORT_OUTCLR0 = PORT_PA18;
#endif
    dir_M3 = -1;
  }
  if (speed_M3 == 0)
    timer_period = MINIMUN_TIMER_PERIOD;
  else if (speed_M3 > 0)
    timer_period = 3000000 / speed_M3; // 3Mhz timer (48Mhz / preescaler=16 = 3Mhz)
  else
    timer_period = 3000000 / -speed_M3;
  if (timer_period > MINIMUN_TIMER_PERIOD)   // Check for minimun speed (maximun period without overflow)
    timer_period = MINIMUN_TIMER_PERIOD;

  // // Change timer
  // TCC2->CCB[0].reg = timer_period;
  // while (TCC2->STATUS.reg & TC_STATUS_SYNCBUSY); // wait for sync...
  // Timer TCC2 has doble buffer (CCB register) so we can set the new period directly
}


// Adjust robot speed on each axis so both movements ends at the same time
// for simplicity we don´t take into account accelerations, only speed
void adjustSpeed()
{
  float diff_M1;
  float diff_M2; 
  
  // Speed adjust to draw straight lines
  diff_M1 = abs((float)target_position_M1 - (float)position_M1);
  diff_M2 = abs((float)target_position_M2 - (float)position_M2) * M1_M2_STEP_FACTOR;
  if (diff_M1 >= diff_M2) { // Wich axis will be slower?
    // X axis is the main axis
    target_speed_M1 = MAX_SPEED_M1;
    //acceleration_x = MAX_ACCEL_X;
    target_speed_M2 = (float)MAX_SPEED_M2 * diff_M2 / diff_M1;
    //acceleration_y = (float)MAX_ACCEL_Y * diff_y / diff_x;
    //if (acceleration_y<60)
    //  acceleration_y=60;
    }
  else {
    target_speed_M2 = MAX_SPEED_M2;
    //acceleration_y = MAX_ACCEL_Y;
    target_speed_M1 = (float)MAX_SPEED_M1 * diff_M1 / diff_M2;
    //acceleration_x = (float)MAX_ACCEL_X * diff_x / diff_y;
    //if (acceleration_x<120)
    //  acceleration_x=120;
    }
}


// Set speed in steps/sec
void configSpeed(int target_sM1, int target_sM2, int target_sM3)
{
  target_sM1 = constrain(target_sM1, 0, MAX_SPEED_M1);
  target_sM2 = constrain(target_sM2, 0, MAX_SPEED_M2);
  target_sM3 = constrain(target_sM3, 0, MAX_SPEED_M3);
  config_speed_M1 = target_sM1;
  config_speed_M2 = target_sM2;
  config_speed_M3 = target_sM3;
}

// Set acceleration
void configAcceleration(int target_acc1, int target_acc2, int target_acc3)
{
  target_acc1 = constrain(target_acc1, MIN_ACCEL_M1, MAX_ACCEL_M1);
  target_acc2 = constrain(target_acc2, MIN_ACCEL_M2, MAX_ACCEL_M2);
  target_acc3 = constrain(target_acc3, MIN_ACCEL_M3, MAX_ACCEL_M3);
  config_acceleration_M1 = target_acc1;
  config_acceleration_M2 = target_acc2;
  config_acceleration_M3 = target_acc3;
}

// Set the max speed and acceleration for a movement from config values
void setSpeedAcc()
{
  // Set speed...
  target_speed_M1 = config_speed_M1;
  target_speed_M2 = config_speed_M2;
  target_speed_M3 = config_speed_M3;
  // Set accelerations...
  target_acceleration_M1 = config_acceleration_M1;
  target_acceleration_M2 = config_acceleration_M2;
  target_acceleration_M3 = config_acceleration_M3;
}

//Configures the TC to generate output events at the sample frequency.
//Configures the TC in Frequency Generation mode, with an event output once
//each time the audio sample frequency period expires.
void timersConfigure()
{
//  // First we need to enable and configure the Generic Clock register 
//  // Enable GCLK for TC4, TC5, TCC2 and TC3 (timer counter input clock) GCLK_CLKCTRL_ID(GCM_TC4_TC5)
//  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TCC2_TC3));
//  while (GCLK->STATUS.bit.SYNCBUSY);
//  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
//  while (GCLK->STATUS.bit.SYNCBUSY);
 
//  // Configure Timer1
//  TC3->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
//  while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
//  while (TC3->COUNT16.CTRLA.bit.SWRST);

//  // Set Timer counter Mode to 16 bits
//  TC3->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
//  // Set TC5 mode as match frequency
//  TC3->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
//  //set prescaler and enable TC5
//  TC3->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV16 | TC_CTRLA_ENABLE;  // preescaler 16 48Mhz=>3Mhz
//  //set TC5 timer counter based off of the system clock and the user defined sample rate or waveform
//  TC3->COUNT16.CC[0].reg = (uint16_t) MINIMUN_TIMER_PERIOD;
 
//  while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
 
//  // Configure interrupt request
//  NVIC_DisableIRQ(TC3_IRQn);
//  NVIC_ClearPendingIRQ(TC3_IRQn);
//  NVIC_SetPriority(TC3_IRQn, 0);
//  NVIC_EnableIRQ(TC3_IRQn);

//  // Enable interrupt request
//  TC3->COUNT16.INTENSET.bit.MC0 = 1;
//  while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY); //wait until syncing

//  // Configure Timer2 on TC5
//  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
//  while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
//  while (TC5->COUNT16.CTRLA.bit.SWRST);

//  // Set Timer counter Mode to 16 bits
//  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
//  // Set TC5 mode as match frequency
//  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
//  //set prescaler and enable TC5
//  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV16 | TC_CTRLA_ENABLE;  // preescaler 16 48Mhz=>3Mhz
//  //set TC5 timer counter based off of the system clock and the user defined sample rate or waveform
//  TC5->COUNT16.CC[0].reg = (uint16_t) MINIMUN_TIMER_PERIOD;
 
//  while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY); // wait for sync...
 
//  // Configure interrupt request
//  NVIC_DisableIRQ(TC5_IRQn);
//  NVIC_ClearPendingIRQ(TC5_IRQn);
//  NVIC_SetPriority(TC5_IRQn, 0);
//  NVIC_EnableIRQ(TC5_IRQn);

//  // Enable interrupt request
//  TC5->COUNT16.INTENSET.bit.MC0 = 1;
//  while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY); //wait until syncing

//  // Configure Timer3 on TCC2
//  TCC2->CTRLA.reg = TC_CTRLA_SWRST;
//  //while (TCCC2->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
//  //while (TCCC2->COUNT16.CTRLA.bit.SWRST);
//  while (TCC2->SYNCBUSY.bit.ENABLE == 1); // wait for sync
//  while (TCC2->CTRLA.bit.SWRST);
 
//  // Set Timer counter Mode to 16 bits
//  //TCC2->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
//  // Set TCC2 mode as match frequency
//  //TCC2->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
//  TCC2->WAVE.reg |= TCC_WAVE_WAVEGEN_MFRQ;
//  //set prescaler and enable TCC2
//  TCC2->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV16 | TCC_CTRLA_ENABLE;  // preescaler 16 48Mhz=>3Mhz
//  //set TC5 timer counter based off of the system clock and the user defined sample rate or waveform
//  TCC2->CC[0].reg = (uint16_t)MINIMUN_TIMER_PERIOD;
//  TCC2->CTRLBCLR.reg |= TCC_CTRLBCLR_LUPD;   // Enable doble buffering
 
//  //while (TCC2->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY); // wait for sync...
//  while (TCC2->SYNCBUSY.bit.ENABLE == 1); // wait for sync 
 
//  // Configure interrupt request
//  NVIC_DisableIRQ(TCC2_IRQn);
//  NVIC_ClearPendingIRQ(TCC2_IRQn);
//  NVIC_SetPriority(TCC2_IRQn, 0);
//  NVIC_EnableIRQ(TCC2_IRQn);

//  // Enable interrupt request
//  //TCC2->COUNT16.INTENSET.bit.MC0 = 1;
//  TCC2->INTENSET.reg = 0;                 // disable all interrupts
//  //TCC2->INTENSET.bit.OVF = 1;          // enable overfollow
//  TCC2->INTENSET.bit.MC0 = 1;          // enable compare match to CC0
//  //while (TCC2->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY); //wait until syncing
//  while (TCC2->SYNCBUSY.bit.ENABLE == 1); // wait for sync 
} 

// This function enables Timers TC3 and TC5 and waits for it to be ready
void timersStart()
{
  // TC3->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
  // while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY); //wait until snyc'd
  // TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
  // while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY); //wait until snyc'd
  // TCC2->CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
  // while (TCC2->STATUS.reg & TC_STATUS_SYNCBUSY); //wait until snyc'd
}

//Reset timers TC3 and TC5 
void timersReset()
{
  // TC3->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  // while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
  // while (TC3->COUNT16.CTRLA.bit.SWRST);
  // TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  // while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
  // while (TC5->COUNT16.CTRLA.bit.SWRST);
}

// Disable timers TC3 and TC5
void timersDisable()
{
  // TC3->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  // while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
  // TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  // while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
}

void motorsCalibration()
{
//   // Enable motors
//   digitalWrite(11, LOW);  // Enable motors
//     // Robot calibration... moving robot to both ends until we loose some steps (brute method because the robot dont have final position switches
//   configSpeed(MAX_SPEED_M1 / 12, MAX_SPEED_M2 / 4, MAX_SPEED_M3);
//   setSpeedAcc();
//   // MOTOR1
//   // Robot calibration... moving robot to both ends until we loose some steps (brute method because the robot dont have final position switches
//   target_angleA1 = -(ROBOT_ABSOLUTE_MAX_M1); // Force the robot to loose steps and reach the final position
//   target_angleA2 = (ROBOT_INITIAL_POSITION_M2 + 90) + target_angleA1 * AXIS2_AXIS1_correction; // Force the robot to loose steps and reach the final position
//   target_position_M1 = target_angleA1 * M1_AXIS_STEPS_PER_UNIT;
//   target_position_M2 = target_angleA2 * M2_AXIS_STEPS_PER_UNIT;
//   working = true;
//   digitalWrite(11, LOW);  // Enable motors
//   while (working) {
//     positionControl(1000);
//     delay(1); // 1Khz loop
//   }
//   // Force missing steps on MOTOR1
// #ifdef INVERT_M1_AXIS
//   // REG_PORT_OUTSET0 = PORT_PA20;
// #else
//   // REG_PORT_OUTCLR0 = PORT_PA20;
// #endif
//   for (int cal = 0; cal < 1000; cal++)
//   {
//     // REG_PORT_OUTSET0 = PORT_PA15; // STEP Motor1
//     delay(1);
//     // REG_PORT_OUTCLR0 = PORT_PA15; // STEP Motor1
//     delay(2);
//   }

//   // GO TO CENTER
//   configSpeed(MAX_SPEED_M1 / 4, MAX_SPEED_M2 / 10, MAX_SPEED_M3);
//   setSpeedAcc();
//   target_angleA1 = ROBOT_INITIAL_POSITION_M1; // Force the robot to loose steps and reach the final position
//   target_angleA2 = ROBOT_INITIAL_POSITION_M2 + target_angleA1 * AXIS2_AXIS1_correction; // Force the robot to loose steps and reach the final position
//   target_position_M1 = target_angleA1 * M1_AXIS_STEPS_PER_UNIT;
//   target_position_M2 = target_angleA2 * M2_AXIS_STEPS_PER_UNIT;
//   working = true;
//   digitalWrite(11, LOW);  // Enable motors
//   while (working) {
//     positionControl(1000);
//     delay(1); // 1Khz loop
//   }

//   // MOTOR2
//   // Robot calibration... moving robot to both ends until we loose some steps (brute method because the robot dont have final position switches
//   target_angleA1 = ROBOT_INITIAL_POSITION_M1; // Force the robot to loose steps and reach the final position
//   target_angleA2 = (ROBOT_ABSOLUTE_MAX_M2) + target_angleA1 * AXIS2_AXIS1_correction; // Force the robot to loose steps and reach the final position
//   target_position_M1 = target_angleA1 * M1_AXIS_STEPS_PER_UNIT;
//   target_position_M2 = target_angleA2 * M2_AXIS_STEPS_PER_UNIT;
//   working = true;
//   digitalWrite(11, LOW);  // Enable motors
//   while (working) {
//     positionControl(1000);
//     delay(1); // 1Khz loop
//   }
//   // Force missing steps on MOTOR2
// #ifdef INVERT_M2_AXIS
//   REG_PORT_OUTCLR0 = PORT_PA06;
// #else
//   REG_PORT_OUTSET0 = PORT_PA06;
// #endif
//   for (int cal = 0; cal < 1600; cal++)
//   {
//     REG_PORT_OUTSET0 = PORT_PA21; // STEP Motor2
//     delay(1);
//     REG_PORT_OUTCLR0 = PORT_PA21; // STEP Motor2
//     delay(1);
//   }

//   // Force the robot to its initial position
//   //target_angleA1 = ROBOT_INITIAL_POSITION_M1; // Force the robot to loose steps (if necessary) and reach the final position
//   //target_angleA2 = (ROBOT_ABSOLUTE_MAX_M2+20) + target_angleA1*AXIS2_AXIS1_correction;;
//   //target_position_M1 = target_angleA1*M1_AXIS_STEPS_PER_UNIT;
//   // target_position_M2 = target_angleA2*M2_AXIS_STEPS_PER_UNIT;
//   //working=true;
//   //digitalWrite(11, LOW);  // Enable motors
//   //while (working){
//   //  positionControl(1000);
//   //  delay(1); // 1Khz loop
//   //}
//   //target_angleA2 = (ROBOT_ABSOLUTE_MAX_M2) + target_angleA1*AXIS2_AXIS1_correction;
//   //target_position_M2 = target_angleA2*M2_AXIS_STEPS_PER_UNIT;
//   //position_M2 = target_position_M2;

//   configSpeed(MAX_SPEED_M1 / 4, MAX_SPEED_M2 / 4, MAX_SPEED_M3);
//   setSpeedAcc();

//   // Force the robot to its initial centered position
//   target_angleA1 = ROBOT_INITIAL_POSITION_M1; // Force the robot to loose steps (if necessary) and reach the final position
//   target_angleA2 = ROBOT_INITIAL_POSITION_M2 + ROBOT_INITIAL_POSITION_M1 * AXIS2_AXIS1_correction;
//   target_position_M1 = target_angleA1 * M1_AXIS_STEPS_PER_UNIT;
//   target_position_M2 = target_angleA2 * M2_AXIS_STEPS_PER_UNIT;
//   working = true;
//   digitalWrite(11, LOW);  // Enable motors
//   while (working) {
//     positionControl(1000);
//     delay(1); // 1Khz loop
//   }

//   moveServo1(SERVO1_MIN_PULSEWIDTH);
//   delay(600);
//   moveServo1(SERVO1_MAX_PULSEWIDTH);
//   delay(600);
//   moveServo1(SERVO1_NEUTRAL);
//   delay(600);
//   moveServo2(SERVO2_MIN_PULSEWIDTH);
//   delay(600);
//   moveServo2(SERVO2_NEUTRAL);
}

// Set Robot Axis1:
void setAxis1(float angleA1)
{
  angleA1 = constrain(angleA1,ROBOT_MIN_A1,ROBOT_MAX_A1);
  target_position_M1 = angleA1*M1_AXIS_STEPS_PER_UNIT;
}

// Set Robot Axis2. Axis 2 angle depends on Axis1 angle
void setAxis_1_2(float angleA1,float angleA2)
{
  if (angleA1<-199.0) // If angleA1 is not specified (NULL) we take it from robot position
    angleA1 = target_angleA1;
  if (angleA2<-199.0) // If angleA1 is not specified (NULL)we take it from robot position
    angleA2 = target_angleA2; // If NODATA on iCH1, we calculate the current angle
  //Serial.print("L ");
  //Serial.print(angleA1);
  //Serial.print(" ");
  //Serial.print(angleA2);
  //Serial.print(" ");
  angleA1 = constrain(angleA1,ROBOT_MIN_A1,ROBOT_MAX_A1);
  limits_index = int((angleA1+ROBOT_MIN_A1)/-20.0);
  limits_index = constrain(limits_index,0,11); // index on limits array
  // Robot limits... (actually on a fixed table)
  angleA2 = constrain(angleA2,robot_limits_min[limits_index],robot_limits_max[limits_index]); // Calculate the limits of A2 based on A1 angle
  //angleA2 = constrain(angleA2,ROBOT_MIN_A2,ROBOT_MAX_A2);
  //Serial.println(AXIS2_AXIS1_correction);
  angleA2 = angleA2 + angleA1*AXIS2_AXIS1_correction;
  //Serial.print(angleA2);
  //Serial.print(" ");
  //Serial.print(ROBOT_MIN_A1);
  //Serial.print(" ");
  //Serial.print(index);
  //Serial.print(" ");
  //Serial.print(robot_limits_min[index]);
  //Serial.print(" ");
  //Serial.println(robot_limits_max[index]);
  target_angleA1 = angleA1;
  target_angleA2 = angleA2;
  target_position_M1 = target_angleA1*M1_AXIS_STEPS_PER_UNIT;
  target_position_M2 = target_angleA2*M2_AXIS_STEPS_PER_UNIT; 
}

// Set Robot axis 3 (height)
void setAxis3(float height)
{
  height = constrain(height,0,ROBOT_MAX_A3);
  target_position_M3 = height*M3_AXIS_STEPS_PER_UNIT;
}

// Inverse kinematic formulas for SCARA ROBOT
// This function returns C angle using law of cosines
float lawOfCosines(float a, float b, float c) 
{
  return acosf((a*a + b*b - c*c) / (2.0f * a * b));
}
// Euclidean distance between 2 points (0,0 to x,y)
float distance(float x, float y)
{
  return sqrt(x*x +y*y);
}

// time profile: between 520-540us
void InverseKinematic(float x, float y, float len1, float len2, uint8_t elbow, float *A1, float *A2)
{
  float dist;
  float D1,D2;

  if (elbow==1)  // inverse elbow solution: reverse X axis, and final angles.
    x = -x;
  dist = distance(x,y);
  if (dist > (ROBOT_ARM1_LENGTH+ROBOT_ARM2_LENGTH)){
    dist = (ROBOT_ARM1_LENGTH+ROBOT_ARM2_LENGTH)-0.001f;
    Serial.println("IK overflow->limit");
  }
  D1 = atan2(y,x);   // 140us
  D2 = lawOfCosines(dist, len1, len2);   // 175us
  *A1 = (D1+D2+ROBOT_AXIS_DEFINITION)*RAD2GRAD;
  *A2 = lawOfCosines(len1,len2,dist)*RAD2GRAD-180;  // 165us
  if (elbow==1){
    *A1 = -*A1;
    *A2 = -*A2;
  }
}

// Calculate the total time of a motor movement (trapezoidal profile)
// INPUT: d=distance, v0=initial speed, v1=target max speed, a=acceleration
// OUTUT: total time of the full movement
// There are 3 posible cases:
//  CASE1: Only a deceleration motion is needed (because of the initial v0 speed) The robot will overpass and start a new movement to the final position
//  CASE2: Only an aceleration/deceleration movement is done (triangular profile) because there are no time to reach the full target speed(v1) [very common]
//  CASE3: Full trapezoidal movement with an acceleration period (until tA), a full speed stage until the deceleration point(tD) where the robot start deceleration until stop.
// tipical timing for a Case2: 85us
float move_total_time(float d,float v0,float v1,float a)
{
  float d_D_v0,d_D_v1,d_A_v1,d_A,d_D,d_FS,d_OVERPASS;
  float t_A,t_D,t_FS,t_F,t_F_2;
  float vp; 
  uint8_t case_type;

  // calculate the distance to decelerate until stop at target speed
  // kinematic formula   d = (sqr(v1)-sqr(v0))/(2*a) because v1=0 and acc(a) is negative in deceleration => d = sqr(v1)/2a
  d_D_v1 = (v1*v1)/(2*a);
  //Serial.print("->d_D_V1:");
  //Serial.println(d_D_v1);
  if (d_D_v1 > d)
  {
    d_D_v0 = (v0*v0)/(2*a);
    // Robot could not reach full speed => case 1 or 2.
    if (d_D_v0 > d)   // With the initial speed the robot will overpass => case 1 start deceleration now!
      case_type = 1;
    else
      case_type = 2;  // The movement will have an aceleration/deceleration (triangular profile).
  }
  else
  {
    // kinematic formula   d = (sqr(v1)-sqr(v0))/(2*a)
    d_A_v1 = ((v1*v1)-(v0*v0))/(2*a);  // distance in acceleration ramp to full speed
    // if the total distance greater than the sum of the distance in the acceleration and deceleration ramps? => distance enough for a full speed part?
    if ((d_A_v1+d_D_v1)<d)
      case_type = 3;  // Full trapezoidal profile (we have a full speed part)
    else
      case_type = 2;  // The movement will have an aceleration/deceleration (triangular profile).
  }

  if (case_type==1){
    Serial.print("->C1:");
    return 0; //CASE 1 not usable right now... => exit without correction...
    t_F = v0/a;  // This is really (0-v0)/(-a) because we are decelerating
    //Serial.print(t_F);
    // There will be overpass in this case so we need to calculate the new movement!
    d_OVERPASS = d_D_v0 - d;  // This is the new movement that we need to do
    //New conditions:
    d = d_OVERPASS;
    v0 = 0;
    // To do...
    d_A_v1 = (v1*v1)/(2*a);
    d_D_v1 = d_A_v1;
    if ((d_A_v1+d_D_v1)<d){
      // Full trapezoidal movement...
      Serial.print(" C3:");
      d_FS = d - (d_A_v1+d_D_v1);  // Calculate the total distance that we are at full speed
      t_FS = d_FS/v1;
      t_A = v1/a;
      t_D = t_A;         // Because we start and end at zero speed
      t_F_2 = t_A+t_FS+t_D;
      Serial.print(t_F_2);
      Serial.print(" T:");
      t_F += t_F_2;
    }
    else{
      // Triangular movement...
      Serial.print(" C2:");
      vp = sqrt(a*d);  // simplification because v0=0
      t_A = vp/a;
      t_D = t_A;       // Because we start and end at zero speed
      t_F_2 = t_A+t_D;  // total time = time_accelerating + time_decelerating
      //Serial.print(t_F_2);
      //Serial.print(" T:");
      t_F += t_F_2;
    }
  }
  if (case_type==2){
    //Serial.print("->C2:");
    // Basic condition: d = d_A + d_D  (total distance = distance_acceleration + distance_decelerating
    // vp is the peek velocity (where the movement change from acceleration to deceleration), the point of the triangle
    // d_A = ((vp*vp)-(v0*v0))/2a ; d_D = (vp*vp)/2a ; d = d_A + d_D
    // equation to solve vp: vp = sqrt((2*a*d + v0*v0)/2)
    //vp = quadratic_equation(2,-2*v0,((v0*v0)-(2*d*a)));
    vp = sqrt((2*a*d-(v0*v0))/2);
    //Serial.print(" vp:");
    //Serial.print(vp);
    //Serial.print(" ");
    t_A = abs(vp-v0)/a;  // always suppose positive values...
    t_D = vp/a;  // This is really (0-vp)/(-a) because we are decelerating
    t_F = t_A+t_D;  // total time = time_accelerating + time_decelerating
    //Serial.print(t_A);
    //Serial.print(",");
    //Serial.print(t_D);
    //Serial.print(":");
  }
  if (case_type==3){
    //Serial.print("->C3:");
    d_FS = d - (d_A_v1+d_D_v1);  // Calculate the total distance that we are at full speed
    t_FS = d_FS/v1;
    t_A = abs(v1-v0)/a;  // always suppose positive values...
    t_D = v1/a; // This is really (0-v1)/(-a) because we are decelerating
    t_F = t_A+t_FS+t_D;
    //Serial.print(t_A);
    //Serial.print(",");
    //Serial.print(t_FS);
    //Serial.print(",");
    //Serial.print(t_D);
    //Serial.print(":");
  }
  //Serial.println(t_F);
  return t_F;
}

// This function adjust the speed of the motors so the movement will be syncronized
// so both motors end at the same time. 
// We only run this adjustment if motors are running in the correct direction (if a motor need to invert its movement we dont make the adjust)
// time profile: around 300-350us
void trajectory_motor_speed_adjust()
{
  float t_M1,t_M2;
  float factor;
  int16_t M1_diff,M2_diff;
  int16_t actual_speed_M1,actual_speed_M2;
  
  Serial.print("->PLAN ");
  //long t0 = micros();
  M1_diff = target_position_M1 - position_M1;
  actual_speed_M1 = speed_M1;
  if (sign(M1_diff)!=sign(actual_speed_M1)){
    // Check if speed is low enought to suppose speed=0
    if (myAbs(actual_speed_M1)<(4*target_acceleration_M1)){
      Serial.print("V1adj ");
      Serial.print(actual_speed_M1);
      actual_speed_M1=0;  // suppose speed=0 and continue with trajectory motor speed adjust
    }
    else{
      Serial.print("ExitM1 ");
      Serial.print(M1_diff);
      Serial.print(",");
      Serial.println(actual_speed_M1);
      return;
    }
  }
  // Target position and actual speed has the same sign, we are going in the right way...
  M1_diff = myAbs(M1_diff);
  actual_speed_M1 = myAbs(actual_speed_M1);

  // We are going to resolve the total time of movement
  //Serial.print("->PM1:");
  //Serial.print(M1_diff);
  //Serial.print(",");
  //Serial.print(speed_M1);
  //Serial.print(",");
  //Serial.print(target_speed_M1);
  //Serial.print(",");
  //Serial.println(target_acceleration_M1*1000);
  long t1 = micros();
  t_M1 = move_total_time(M1_diff,actual_speed_M1,target_speed_M1,target_acceleration_M1*1000);
  long t2 = micros();

  // Axis2
  M2_diff = target_position_M2 - position_M2;
  actual_speed_M2 = speed_M2;
  if (sign(M2_diff)!=sign(actual_speed_M2)){
    // Check if speed is low enought to suppose speed=0
    if (myAbs(actual_speed_M2)<(4*target_acceleration_M2)){
      Serial.print("V2adj ");
      Serial.print(actual_speed_M2);
      actual_speed_M2=0;  // suppose speed=0 and continue with trajectory motor speed adjust
    }
    else{
      Serial.print("ExitM2 ");
      Serial.print(M2_diff);
      Serial.print(",");
      Serial.println(actual_speed_M2);
      return;
    }
  }
  // Target position and actual speed has the same sign, we are going in the right way...
  M2_diff = myAbs(M2_diff);
  actual_speed_M2 = myAbs(actual_speed_M2);
  // We are going to resolve the total time of movement
  //Serial.print("->PM2:");
  //Serial.print(M2_diff);
  //Serial.print("[");
  //Serial.print(target_position_M2);
  //Serial.print(",");
  //Serial.print(position_M2);
  //Serial.print("]");
  //Serial.print(speed_M2);
  //Serial.print(",");
  //Serial.print(target_speed_M2);
  //Serial.print(",");
  //Serial.println(target_acceleration_M2);
  t_M2 = move_total_time(M2_diff,actual_speed_M2,target_speed_M2,target_acceleration_M2*1000);

  if ((t_M1==t_M2)||(t_M1<0.01)||(t_M2<0.01)){
    Serial.println("->P Small t!");
    return;
  }
    
  // Decide wich axis will be slower. That will be the master axis
  if (t_M1>t_M2){
    // Motor1 is slower so its the master, we adapt motor2...
    factor = t_M2/t_M1;
    Serial.print("F2:");
    Serial.print(t_M1);
    Serial.print(",");
    Serial.print(t_M2);
    // Modify speed and acceleration of motor 2 acording to the time factor
    target_speed_M2 = target_speed_M2*factor;
    target_acceleration_M2 = target_acceleration_M2*factor*factor;
    //Serial.print("TS2:");
    //Serial.print(target_speed_M2);
    //Serial.print(",");
    //Serial.print(target_acceleration_M2);
    if (target_acceleration_M2<4)
      target_acceleration_M2 = 4;
    //Serial.print("->NEW2:");
    //Serial.print(target_speed_M2);
    //Serial.print(",");
    //Serial.println(target_acceleration_M2*1000);
    //Serial.print(" t:");
    //Serial.print(t_M1);
    //Serial.print(",");
    //Serial.print(t_M2);
    //Serial.print(",");
    //t_M2 = move_total_time(M2_diff,speed_M2,target_speed_M2,target_acceleration_M2*1000);  // TO CHECK ONLY
    //Serial.print(t_M2);
  }
  else{
    // Motor2 is slower so its the master, we adapt motor2...
    factor = t_M1/t_M2;
    Serial.print("F1:");
    Serial.print(t_M1);
    Serial.print(",");
    Serial.print(t_M2);
    // Modify speed and acceleration of motor 1 acording to the time factor
    target_speed_M1 = target_speed_M1*factor;
    target_acceleration_M1 = target_acceleration_M1*factor*factor;
    if (target_acceleration_M1<4)
      target_acceleration_M1 = 4;
    //Serial.print("->NEW1:");
    //Serial.print(target_speed_M1);
    //Serial.print(",");
    //Serial.println(target_acceleration_M1*1000);
    //t_M1 = move_total_time(M1_diff,speed_M1,target_speed_M1,target_acceleration_M1*1000); // TO CHECK ONLY
  }
  //long t3 = micros();
  //Serial.print(t2-t1);
  //Serial.print(" ");
  //Serial.println(t3-t0);
  Serial.println();
}

// Servos functions...
void initServo()
{
  servo1.attach(3);
  servo2.attach(4);
  servo1_ready=true;
  servo2_ready=true;
}

void disableServo1()
{
  servo1_ready=false;
  servo1.detach();
}

void disableServo2()
{
  servo2_ready=false;
  servo2.detach();
}

void enableServo1()
{
  servo1_ready=true;
  servo1.attach(3);
}

void enableServo2()
{
  servo2_ready=true;
  servo2.attach(4);
}


// move servo1 on OC4B (pin10)
void moveServo1(int pwm)
{
  pwm = constrain(pwm,SERVO1_MIN_PULSEWIDTH,SERVO1_MAX_PULSEWIDTH);
  servo1.writeMicroseconds(pwm);
  actual_valueW = pwm;
}

// move servo2 on OC4A (pin13)
void moveServo2(int pwm)
{
  pwm = constrain(pwm,SERVO2_MIN_PULSEWIDTH,SERVO2_MAX_PULSEWIDTH);
  servo2.writeMicroseconds(pwm);
  actual_valueG = pwm;
}
