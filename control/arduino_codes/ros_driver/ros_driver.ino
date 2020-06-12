#include <math.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

char t;
int LEFT_IN1 = 8;
int LEFT_IN2 = 9;
int RIGHT_IN1 = 10;
int RIGHT_IN2 = 11;
int PWM_RIGHT = 5;
int PWM_LEFT = 6;

int PWM_MIN = 90;
int PWMRANGE = 255;

ros::NodeHandle  nh;
bool force_stop = false;
float linear_speed_stateValue, angular_speed_stateValue;

void speedCb( const geometry_msgs::Twist& speed_msg){
  linear_speed_stateValue = speed_msg.linear.x; //linear speed component
  angular_speed_stateValue = speed_msg.linear.x; //angular_speed_stateValue speed component
  
  //command the driving motors
  if(force_stop){
    set_velocity_percent(0, 0);
  }
  else{
    linear_speed_stateValue = linear_speed_stateValue * 50;
    angular_speed_stateValue = angular_speed_stateValue * 50;
    set_velocity_percent(linear_speed_stateValue, angular_speed_stateValue);
  }
  
  
}

ros::Subscriber<geometry_msgs::Twist> speedSub("/cmd_vel", speedCb );

void setup() {
  nh.initNode();
  nh.subscribe(speedSub);
  init_motor_driver();

  pinMode(13, OUTPUT);  //Led
  digitalWrite(13, HIGH);

  Serial.begin(9600);

  set_velocity_percent(0, 0);
}

void init_motor_driver() {
  //Left motors init
  pinMode(PWM_LEFT, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);

  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);

  //right motors init
  pinMode(PWM_RIGHT, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);
}


void set_velocity_percent(float linear_velocity_percent, float angular_velocity_percent) {
  if (linear_velocity_percent < -100) {
    linear_velocity_percent = -100;
  }
  if (linear_velocity_percent > 100) {
    linear_velocity_percent = 100;
  }
  if (angular_velocity_percent < -100) {
    angular_velocity_percent = -100;
  }
  if (angular_velocity_percent > 100) {
    angular_velocity_percent = 100;
  }

  float left_wheel_speed_percent = (linear_velocity_percent - angular_velocity_percent) / 2;
  float right_wheel_speed_percent = (linear_velocity_percent + angular_velocity_percent) / 2;

  set_wheels_speed_percent(left_wheel_speed_percent, right_wheel_speed_percent);
}

void set_wheels_speed_percent(float left_wheel_speed_percent, float right_wheel_speed_percent) {
  digitalWrite(LEFT_IN1, left_wheel_speed_percent > 0);
  digitalWrite(LEFT_IN2, left_wheel_speed_percent < 0);
  digitalWrite(RIGHT_IN1, right_wheel_speed_percent > 0);
  digitalWrite(RIGHT_IN2, right_wheel_speed_percent < 0);
  analogWrite(PWM_LEFT, mapPwm(fabs(left_wheel_speed_percent / 100), PWM_MIN, PWMRANGE));
  analogWrite(PWM_RIGHT, mapPwm(fabs(right_wheel_speed_percent / 100), PWM_MIN, PWMRANGE));
}

// Map x value from [0 .. 1] to [out_min .. out_max]
float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}


void loop() {
  if (Serial.available()) {
    t = Serial.read();
    Serial.println(t);
  }

  if (t == 'W') {  //turn led on or off)
    digitalWrite(13, HIGH);
    force_stop = true;
  }
  else if (t == 'w') {
    digitalWrite(13, LOW);
    force_stop = false;
  }
  nh.spinOnce();
  delay(100);
}
