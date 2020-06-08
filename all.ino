#include "Servo.h"

void Motor_move(uint8_t motor, uint8_t direct, uint8_t pwm);
void Stop(uint8_t motor);
void Stop_all();
void Clockwise(uint8_t motor, uint8_t speed_pwm);
void CounterCW(uint8_t motor, uint8_t speed_pwm);
void Forward(uint8_t speed_pwm);
void Backward(uint8_t speed_pwm);
void Left(uint8_t speed_pwm);
void Right(uint8_t speed_pwm);
void Increase_speed(uint8_t motor);
void Decrease_speed(uint8_t motor);
uint8_t motor_number(uint8_t motor);
float count_speed_rpm(uint8_t motor_pin);
void Assing_speed_rpm();
void Align_motors_speed(uint8_t motor1, uint8_t motor2);
void continous_servo_write(Servo sev, uint8_t deg);
void move_CW(Servo sev, uint8_t deg);
void move_CCW(Servo sev, uint8_t deg);
void move_up(uint8_t pwm);
void move_down(uint8_t pwm);
void stand_straight();
void change_hight(uint8_t pwm);
uint8_t count_deg(uint8_t distance);
void open_wheels();
void close_wheels();
float count_distance();
float obst_hight();
uint8_t check_wall();
void climb();
void go_under();
void error();
void climb_back(uint8_t deg, uint8_t x);
void find_way();


//***********************************************************************//
//variables

#define CH1 13     //receiver channel 1 connected to pin D13
#define CH2 10     //receiver channel 2 connected to pin D10
#define CH3 3      //receiver channel 3 connected to pin D3
#define CH4 1      //receiver channel 4 connected to pin D1

uint8_t CH_pwm[6]={0, 0, 0, 0, 0, 0};  //table that contains pwm signal from 6 channels

//motor states
#define BREAK   0
#define CW      2
#define CCW     14

//dc motors
#define MOTOR_1 3
#define MOTOR_2 4
#define MOTOR_3 5
#define MOTOR_4 6

//first motor controller - motors 1 and 2
#define CW1    33   //clockwise
#define CCW1   31   //counter clockwise
#define EN_M1  A2   //enable
#define PWM1   11

#define CW2    37   //clockwise
#define CCW2   35   //under clockwise
#define EN_M2  A3   //enable
#define PWM2   12

//second motor controller - motors 3 and 4
#define CW3    49   //clockwise
#define CCW3   47   //under clockwise
#define EN_M3  A0   //enable
#define PWM3   8

#define CW4    53   //clockwise
#define CCW4   51   //under clockwise
#define EN_M4  A1   //enable
#define PWM4   9

//Motor 1 - encoder
#define MOTOR_A1_PIN  28
#define MOTOR_B1_PIN  26

//Motor 2 - encoder
#define MOTOR_A2_PIN  24
#define MOTOR_B2_PIN  22

//Motor 3 - encoder
#define MOTOR_A3_PIN  52
#define MOTOR_B3_PIN  50

//Motor 4 - encoder
#define MOTOR_A4_PIN  48
#define MOTOR_B4_PIN  46

uint8_t Speed_pwm[4] = {0, 0, 0, 0};  //default motor speed
float Speed_rpm[4] = {0.0, 0.0, 0.0, 0.0};
unsigned short Motor_Status[4] = { BREAK, BREAK, BREAK, BREAK };  // Motors are off at the beggining

Servo Sev1;
Servo Sev2;
Servo Sev3;
Servo Sev4;
Servo Sev_wheel;

#define DistSen1 A7  //Distance Sensors 2-15cm  1 - under, 2 - at the front, 3 - on
#define DistSen2 A6
#define DistSen3 A5

uint8_t speed_value = 128;    //pwm, speed 0
uint8_t rotation = 128;       //pwm, no rotation
uint8_t hight = 0;            //pwm, lowest hight
uint8_t hight_r = 0;          //pwm, hight, signal on transmitter
uint8_t wheel_c = 128;        //pwm, wheel closed

  
//***********************************************************************//
//main

void setup() 
{
  Serial.begin(115200);

  //receiver
  pinMode(CH1,INPUT);   //move forward or backward
  pinMode(CH2,INPUT);   //turn right or left
  pinMode(CH3,INPUT);   //get lower or higher
  pinMode(CH4,INPUT);   //open/close wheels

  //motors controllers
  pinMode(CW1, OUTPUT);
  pinMode(CW2, OUTPUT);
  pinMode(CW3, OUTPUT);
  pinMode(CW4, OUTPUT);

  pinMode(CCW1, OUTPUT);
  pinMode(CCW2, OUTPUT);
  pinMode(CCW3, OUTPUT);
  pinMode(CCW4, OUTPUT);

  pinMode(EN_M1, OUTPUT);
  pinMode(EN_M2, OUTPUT);
  pinMode(EN_M3, OUTPUT);
  pinMode(EN_M4, OUTPUT);

  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(PWM4, OUTPUT);

  //encoders
  pinMode(MOTOR_A1_PIN, INPUT_PULLUP);
  pinMode(MOTOR_B1_PIN, INPUT_PULLUP);
  pinMode(MOTOR_A2_PIN, INPUT_PULLUP);
  pinMode(MOTOR_B2_PIN, INPUT_PULLUP);
  pinMode(MOTOR_A3_PIN, INPUT_PULLUP);
  pinMode(MOTOR_B3_PIN, INPUT_PULLUP);
  pinMode(MOTOR_A4_PIN, INPUT_PULLUP);
  pinMode(MOTOR_B4_PIN, INPUT_PULLUP);

  //Servo motors
  Sev1.attach(7);
  Sev2.attach(5);
  Sev3.attach(6);
  Sev4.attach(4);
  Sev_wheel.attach(2);

  stand_straight();
}

 
void loop() 
{

  while ((obst_hight() < 2)&&(count_distance(DistSen2) > 5))
  {
    speed_value = map(pulseIn(CH1, HIGH), 1000, 2000, 0, 255);
    rotation = map(pulseIn(CH2, HIGH), 1000, 2000, 0, 255);
    hight_r = map(pulseIn(CH3, HIGH), 1000, 2000, 0, 255);
    wheel_c = map(pulseIn(CH4, HIGH), 1000, 2000, 0, 255);

    //open or close wheels
    if(wheel_c > 128) open_wheels();
    else if(wheel_c < 128) close_wheels();
    
    //stop or move forward or backward, channel 1
    if(speed_value == 128)
    {
      Stop_all();
    } 
    else if (speed_value > 128)
    {
      speed_value = (speed_value - 128)*2;
      Forward(speed_value);
    } 
    else
    {
      speed_value = (128 - speed_value)*2;
      Backward(speed_value);
    }
  
    //turn right or left, channel 2
    if(rotation > 128)
    {
      Stop_all();
      rotation = (rotation - 128);        //turn right/left 255-half of maximum speed
      Right(rotation);
    } 
    else if (rotation < 128)
    {
      Stop_all();
      rotation = (128 - rotation);
      Left(rotation);
    }
    
    //change hight, channel 3
    change_hight(hight_r);
  }

  //finding and overcoming obstacles
  if(obst_hight() >= 2)
  {
    climb();
  }
  if(count_distance(DistSen2 <=5))
  {
    go_under();
  }
} 



//***********************************************************************//
//DC motors functions

void Motor_move(uint8_t motor, uint8_t direct, uint8_t pwm)         //function that controls the variables: motor(1-4), direction (cw/ccw), pwm (entra 0-255);
{
  if (motor == MOTOR_1)
  {
    if (direct == CW)
    {
      digitalWrite(CW1, HIGH);
      digitalWrite(CCW1, LOW);
    }
    else if (direct == CCW)
    {
      digitalWrite(CCW1, HIGH);
      digitalWrite(CW1, LOW);
    }
    else
    {
      digitalWrite(CW1, LOW);
      digitalWrite(CCW1, LOW);
    }

    analogWrite(PWM1, pwm);
  }
  else if (motor == MOTOR_2)
  {
    if (direct == CW)
    {
      digitalWrite(CW2, HIGH);
      digitalWrite(CCW2, LOW);
    }
    else if (direct == CCW)
    {
      digitalWrite(CCW2, HIGH);
      digitalWrite(CW2, LOW);
    }
    else
    {
      digitalWrite(CW2, LOW);
      digitalWrite(CCW2, LOW);
    }

    analogWrite(PWM2, pwm);
  }
  else if (motor == MOTOR_3)
  {
    if (direct == CW)
    {
      digitalWrite(CW3, HIGH);
      digitalWrite(CCW3, LOW);
    }
    else if (direct == CCW)
    {
      digitalWrite(CCW3, HIGH);
      digitalWrite(CW3, LOW);
    }
    else
    {
      digitalWrite(CW3, LOW);
      digitalWrite(CCW3, LOW);
    }

    analogWrite(PWM3, pwm);
  }
  else if (motor == MOTOR_4)
  {
    if (direct == CW)
    {
      digitalWrite(CW4, HIGH);
      digitalWrite(CCW4, LOW);
    }
    else if (direct == CCW)
    {
      digitalWrite(CCW4, HIGH);
      digitalWrite(CW4, LOW);
    }
    else
    {
      digitalWrite(CW4, LOW);
      digitalWrite(CCW4, LOW);
    }
    analogWrite(PWM4, pwm);
  }
}

void Stop(uint8_t motor) // stop a specific motor
{
  Motor_Status[motor_number(motor)] = BREAK;
  Motor_move(motor, Motor_Status[motor_number(motor)], 0);
}

void Stop_all()   // stop all motors
{
  Stop(MOTOR_1);
  Stop(MOTOR_2);
  Stop(MOTOR_3);
  Stop(MOTOR_4);
}

void Clockwise(uint8_t motor, uint8_t speed_pwm)   // go clockwise with a specific wheel
{
  Motor_Status[motor_number(motor)] = CW;
  Motor_move(motor, Motor_Status[motor_number(motor)], speed_pwm);
}

void CounterCW(uint8_t motor, uint8_t speed_pwm)   // go counter clockwise with a specific wheel
{
  Motor_Status[motor_number(motor)] = CCW;
  Motor_move(motor, Motor_Status[motor_number(motor)], speed_pwm);
}

void Forward(uint8_t speed_pwm)   // move straight forward (whole robot)
{
  Clockwise(MOTOR_1, speed_pwm);
  Clockwise(MOTOR_2, speed_pwm);
  CounterCW(MOTOR_3, speed_pwm);
  CounterCW(MOTOR_4, speed_pwm);
  delay(50);

  Assing_speed_rpm();
  Align_motors_speed(MOTOR_1,MOTOR_2);
  Align_motors_speed(MOTOR_1,MOTOR_3);
  Align_motors_speed(MOTOR_1,MOTOR_4); 
}

void Backward(uint8_t speed_pwm)    // move straght backward
{
  Clockwise(MOTOR_3, speed_pwm);
  Clockwise(MOTOR_4, speed_pwm);
  CounterCW(MOTOR_1, speed_pwm);
  CounterCW(MOTOR_2, speed_pwm);
  delay(50);

  Assing_speed_rpm();
  Align_motors_speed(MOTOR_1,MOTOR_2);
  Align_motors_speed(MOTOR_1,MOTOR_3);
  Align_motors_speed(MOTOR_1,MOTOR_4);
}

void Left(uint8_t speed_pwm)
{
  Clockwise(MOTOR_1, speed_pwm);
  Clockwise(MOTOR_2, speed_pwm);
  Clockwise(MOTOR_3, speed_pwm);
  Clockwise(MOTOR_4, speed_pwm);
  delay(50);
}

void Right(uint8_t speed_pwm)
{
  CounterCW(MOTOR_1, speed_pwm);
  CounterCW(MOTOR_2, speed_pwm);
  CounterCW(MOTOR_3, speed_pwm);
  CounterCW(MOTOR_4, speed_pwm);
  delay(50);
}

void Increase_speed(uint8_t motor)
{
  Speed_pwm[motor_number(motor)] = Speed_pwm[motor_number(motor)] + 1;       //increase speed (0,4% of maximum speed)
  if(Speed_pwm[motor_number(motor)] > 255)
  {
    Speed_pwm[motor_number(motor)] = 255;  
  }
  Motor_move(motor, Motor_Status[motor_number(motor)], Speed_pwm[motor_number(motor)]);
  delay(50);
}

void Decrease_speed(uint8_t motor)
{
  Speed_pwm[motor_number(motor)] = Speed_pwm[motor_number(motor)] - 1;       //decrease speed (0,4% of maximum speed)
  if(Speed_pwm[motor_number(motor)] < 0)
  {
    Speed_pwm[motor_number(motor)] = 0;  
  }
  Motor_move(motor, Motor_Status[motor_number(motor)], Speed_pwm[motor_number(motor)]);
  delay(50);
}

uint8_t motor_number(uint8_t motor)     // function that returns motor number
{
  uint8_t n = 0;
  
  if(motor == MOTOR_1){
    n = 0;
  } else if (motor == MOTOR_2){
    n = 1;
  } else if (motor == MOTOR_3){
    n = 2;
  } else if (motor == MOTOR_4){
    n = 3;
  }
  return n;
}

float count_speed_rpm(uint8_t motor_pin)
{
  float V = 0;
  unsigned long time1 = 0;
  unsigned long time2 = 0;
  uint8_t counter = 0;

  time1 = millis();
  do
  {
    if(motor_pin == RISING)
    {
      counter ++;
    }
  } while (counter < 180);  //encoder output 180 imp/obr
  time2 = millis();
  time2 = time1 - time2;
  time2 = time2/60000;    //time in minutes
  
  return V = 1/time2;       //speed in rpm
}

void Assing_speed_rpm()
{
  Speed_rpm[0] = count_speed_rpm(MOTOR_A1_PIN);
  Speed_rpm[1] = count_speed_rpm(MOTOR_A2_PIN);
  Speed_rpm[2] = count_speed_rpm(MOTOR_A3_PIN);
  Speed_rpm[3] = count_speed_rpm(MOTOR_A4_PIN);
}

void Align_motors_speed(uint8_t motor1, uint8_t motor2)
{
  uint8_t a = motor_number(motor1);
  uint8_t b = motor_number(motor2);
  float error = 0.001;    //acceptable deviation 0.1%

  do {
    if(Speed_rpm[b] < ((1-error)*Speed_rpm[a]))
    {
      Increase_speed(motor2);
    } 
    else if(Speed_rpm[b] > ((1+error)*Speed_rpm[a]))
    {
      Decrease_speed(motor2);
    }
  } while (!((Speed_rpm[b] >= ((1-error)*Speed_rpm[a])) && (Speed_rpm[b] >= ((1+error)*Speed_rpm[a]))));
}


//***********************************************************************//
//Servo motors functions

void continous_servo_write(Servo sev, uint8_t deg)    //functions that counts how long Servo has to move with highest speed to rotate to set degree
{
  float time = 0.1/60.0 * deg * 1000;
  sev.write(180);
  delay(time);
  sev.write(90);
}

void move_CW(Servo sev, uint8_t deg)
{
  for(uint8_t i = sev.read(); i <= (sev.read()+deg); i++)
  {
    uint8_t j = 0;
    if(i < 360) j = i;
    else 
    {
      uint8_t x = i/360;
      j = i - (x*360);
    }
    continous_servo_write(sev, j);
    delay(10);
  }
}

void move_CCW(Servo sev, uint8_t deg)
{
    for(uint8_t i = sev.read(); i >= (sev.read()-deg); i--)
  {
    uint8_t j = 0;
    if(i >= 0) j = i;
    else 
    {
      uint8_t x = i/360;
      j = i - (x*360);
      j = 360 + j;
    }
    continous_servo_write(sev, j);
    delay(10);
  }
}

void move_up(uint8_t pwm)
{
  uint8_t dist = pwm/2 + 25;      //min 50, max 150 cm
  uint8_t deg = abs(count_deg(hight)-count_deg(dist));
  
  move_CW(Sev1, deg);
  move_CCW(Sev2, deg);
  move_CCW(Sev3, deg);
  move_CW(Sev4, deg);
  delay(50);
}

void move_down(uint8_t pwm)
{
  uint8_t dist = pwm/2 + 25;
  uint8_t deg = abs(count_deg(hight)-count_deg(dist));
  
  move_CCW(Sev1, deg);
  move_CW(Sev2, deg);
  move_CW(Sev3, deg);
  move_CCW(Sev4, deg);
  delay(50);
}

void stand_straight()   //beginning position
{
  Sev1.write(90);
  Sev2.write(90);
  Sev3.write(90);
  Sev4.write(90);
  move_CCW(Sev1, count_deg(100));
  move_CW(Sev2, count_deg(100));
  move_CW(Sev3, count_deg(100));
  move_CCW(Sev4, count_deg(100));
}

void change_hight(uint8_t pwm)
{
  if(hight_r > pwm)
  {
    move_down(pwm);
  }
  else if (hight_r < pwm)
  {
    move_up(pwm);
  }
  hight_r = hight;
}

uint8_t count_deg(uint8_t distance)     //function that counts degrees for servo motors
{
  uint8_t alfa = asin((distance-28.2)/130) * (180/3.14);
  return alfa;
}

void open_wheels()
{
  Sev_wheel.write(180);
}

void close_wheels()
{
  Sev_wheel.write(0);
}


//***********************************************************************//
//Distance Sensor
                                
float count_distance(uint8_t distance_sensor)
{
  float volts = analogRead(distance_sensor)*(5/1024);   //5V/1024
  float distance = (0.22)*pow(volts, -1.28);      //counted in Excel
  return distance;  //in cm
}

float obst_hight()    //function that counts hight of the obstacle
{
  float h = hight - count_distance(DistSen1);
  return h;
}

uint8_t check_wall()
{
  if(count_distance(DistSen2) > 5) return 1;
  else return 0;
}


//***********************************************************************//
//other functions

void climb()
{
  Stop_all();
  Forward(5);
  delay(1500);
  Stop_all();
  uint8_t deg = 360 - count_deg(hight)+ count_deg(count_distance(DistSen1)); 
  uint8_t i = 0;
  if(!check_wall()) error();
  else
  {
    move_CW(Sev1, deg);
    move_CCW(Sev2, deg);
    for(i=0; i < 30; i++)
    {
      Forward(10);
      delay(100);
      if(!check_wall()) 
      {
        climb_back(deg, i);
        i=31;
      }
    }

    if(i!=31)
    {
      Stop_all();
      move_CW(Sev3, deg);
      move_CCW(Sev4, deg);
      i=0;
      for(i=0; i < 20; i++)
      {
        Forward(10);
        delay(100);
        if(!check_wall()) 
        {
          find_way();
        }
      }
      change_hight(hight);
    }
  }
}

void go_under()
{
  Stop_all();
  uint8_t lower = hight;
  while(count_distance(DistSen2 <= 5))
  {
    lower = lower - 10;
    if (lower >= 0) change_hight(lower);
    else error();
    delay(50);
  }

  if(lower > 24)
  {
    lower = lower - 10;
    move_down(lower);
  
    while(count_distance(DistSen3) > 10)
    {
       if(check_wall())  
       {
        Forward(20);
        delay(100);
       }
       else  find_way();
    }
    while(count_distance(DistSen3) <= 10)
    {
       if(check_wall())  
       {
        Forward(10);
        delay(100);
       }
       else  find_way();
    }

    if(count_distance(DistSen2) > 15)  
    {
      Forward(20);
      delay(1000);
      Stop_all();
    }
    else  find_way();
    }
  else error();
  change_hight(hight);
}

void error()
{
  Backward(10);
  delay(4000);
  Stop_all();
  delay(500);
}

void climb_back(uint8_t deg, uint8_t x)
{
  Stop_all();
  Backward(x);
  delay(4000);
  Stop_all();

  for(uint8_t i=0; i < deg; i++)
  {
    move_CCW(Sev1, 1);
    move_CW(Sev2, 1);
  }

  Backward(10);
  delay(4000);
  Stop_all();
}

void find_way()
{
  Stop_all();
  while(!check_wall())  
  {
    Right(10);
    delay(50);
  }
  Stop_all();
}
