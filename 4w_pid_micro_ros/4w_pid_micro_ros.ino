#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS

#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

#include <Encoder_v1.h>
#include <PID_v1_bc.h>
#include "CytronMotorDriver.h"


int cpr = 12532;

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
unsigned long loopTime = 100;  // Loop time in milliseconds


Encoder front_left(11,12,cpr);
Encoder front_right(8,7,cpr);
Encoder back_right(14,15,cpr);
Encoder back_left(1,0,cpr);

CytronMD FL_motor(PWM_DIR, 5,6);
CytronMD FR_motor(PWM_DIR, 19,18);
CytronMD BR_motor(PWM_DIR, 9,10);
CytronMD BL_motor(PWM_DIR, 3,4);
  
// Motor feedback RPM's
double BR_fb_rpm = 0;
double FR_fb_rpm = 0;
double FL_fb_rpm = 0;
double BL_fb_rpm = 0;

int bl_rpm;
int fl_rpm;
int br_rpm;
int fr_rpm;

// Motor Ouput PWM Value's
double BR_op_pwm = 0;
double FR_op_pwm = 0;
double BL_op_pwm = 0;
double FL_op_pwm = 0;  

double setpoint = 20;

// 
double BL_req_rpm;
double BR_req_rpm;
double FL_req_rpm;
double FR_req_rpm;

double BL_target_rpm;
double BR_target_rpm;
double FL_target_rpm;
double FR_target_rpm;


geometry_msgs__msg__Twist vel;

rcl_publisher_t velocity_feedback; //("velocity_feedback", &vel); // todo


double linear_x_mins,linear_y_mins,ang_z_mins;
double linear_x,linear_y,ang_z;
double wheel_circumference = 2*PI*0.1;
double tangential_vel;
float wheels_x_distance = 0.25;
float wheels_y_distance = 0.20;

float x_rpm;
float y_rpm;
float tan_rpm;

float average_rps_x;
float average_rps_y;
float average_rps_a;


// PID PARAMS 

PID PID_BL(&BL_fb_rpm, &BL_op_pwm, &BL_req_rpm, 1.85,0.0,0.1, DIRECT);
PID PID_FL(&FL_fb_rpm, &FL_op_pwm, &FL_req_rpm, 1.85,0.0,0.1, DIRECT);
PID PID_BR(&BR_fb_rpm, &BR_op_pwm, &BR_req_rpm, 1.85,0.0,0.1, DIRECT);
PID PID_FR(&FR_fb_rpm, &FR_op_pwm, &FR_req_rpm, 1.78,0.0,0.1, DIRECT);


// ros::NodeHandle nh;
rcl_node_t nh;
// geometry_msgs__msg__Twist msg;

void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

void messageCb(const void *msgin)  
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  linear_x = msg->linear.x * 60;
  linear_y = msg->linear.y * 60;
  ang_z    = msg->angular.z *60;

  tangential_vel = ang_z * ((wheels_x_distance / 2) + (wheels_y_distance / 2));

  x_rpm = linear_x / wheel_circumference;
  y_rpm = linear_y / wheel_circumference;
  tan_rpm = tangential_vel / wheel_circumference;

  FL_target_rpm = x_rpm - y_rpm - tan_rpm;
  FL_target_rpm = constrain(FL_target_rpm,-60,60);

  FR_target_rpm = x_rpm + y_rpm + tan_rpm;
  FR_target_rpm = constrain(FR_target_rpm,-60,60);

  BL_target_rpm = x_rpm + y_rpm - tan_rpm;
  BL_target_rpm = constrain(BL_target_rpm,-60,60);

  BR_target_rpm = x_rpm - y_rpm + tan_rpm;
  BR_target_rpm = constrain(BR_target_rpm,-60,60);

  BL_req_rpm = BL_target_rpm;
  FL_req_rpm = FL_target_rpm;
  BR_req_rpm = BR_target_rpm;
  FR_req_rpm = FR_target_rpm;

  
}
  
// ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel",&messageCb);
rcl_subscription_t sub;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;


void setup() {
  set_microros_transports();
  Serial.begin(115200);
  
  setpoint = 10; 

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&nh, "micro_ros_arduino_node", "", &support);
  rclc_subscription_init_default(
    &sub,
    &nh,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel");

  rclc_publisher_init_default(
    &velocity_feedback,
    &nh,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "velocity_feedback");

 

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &sub, &vel, messageCb, ON_NEW_DATA);

  // PID CONFIGURATIONS

  PID_BL.SetMode(AUTOMATIC);
  PID_BL.SetOutputLimits(-255, 255); 
  PID_BL.SetSampleTime(100);


  PID_FL.SetMode(AUTOMATIC);
  PID_FL.SetOutputLimits(-255, 255); 
  PID_FL.SetSampleTime(100);

  PID_BR.SetMode(AUTOMATIC);
  PID_BR.SetOutputLimits(-255, 255); 
  PID_BR.SetSampleTime(100);
  
  PID_FR.SetMode(AUTOMATIC);
  PID_FR.SetOutputLimits(-255, 255); 
  PID_FR.SetSampleTime(100);
  
  // nh.initNode();      // initialzing the node handle object
  // nh.subscribe(sub);  // subscribing to cmd vel with sub object
  // nh.advertise(velocity_feedback); 
  
  // while(!nh.connected())
  // {
  //   nh.spinOnce();
  // }

  //  nh.loginfo("Base Connected....");
}

void loop() {
  

unsigned long currentMillis = millis();
  
 if(currentMillis - previousMillis >= loopTime) 
 {
  
  calc_rpm();

  BR_fb_rpm = -br_rpm;
  BL_fb_rpm = bl_rpm;
  FR_fb_rpm = -fr_rpm;
  FL_fb_rpm = fl_rpm;
   
  
  PID_BR.Compute();
  PID_FR.Compute();
  PID_BL.Compute();
  PID_FL.Compute();
  
  BR_motor.setSpeed(BR_op_pwm);
  FR_motor.setSpeed(FR_op_pwm);
  BL_motor.setSpeed(BL_op_pwm);
  FL_motor.setSpeed(FL_op_pwm);

  calc_vel();
  RCSOFTCHECK(rcl_publish(&velocity_feedback, &vel, NULL));

  

  previousMillis = currentMillis;
}
    //debug print statement
//  Serial.println("setpoint:"+ String(setpoint) + "\t" + "feedback:" + String(FL_fb_rpm)+ "\t"+ "op pwm:"+ String(FL_op_pwm));

// nh.spinOnce();
rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

}

void calc_rpm()
  {
    fl_rpm = front_left.getRPM();
    
    fr_rpm = front_right.getRPM();
    
    bl_rpm = back_left.getRPM();
    
    br_rpm = back_right.getRPM();
    
  }


void calc_vel()
  {

   average_rps_x = ((float)(fl_rpm + (-fr_rpm)  + bl_rpm + (-br_rpm)) / 4) / 60; 
   vel.linear.x = average_rps_x * wheel_circumference; 

   average_rps_y = ((float)(-fl_rpm + (-fr_rpm) + bl_rpm - (-br_rpm)) / 4) / 60; // RPM
   vel.linear.y = average_rps_y * wheel_circumference;

   average_rps_a = ((float)(-fl_rpm + (-fr_rpm) - bl_rpm + (-br_rpm)) / 4) / 60;
   vel.angular.z =  (average_rps_a * wheel_circumference) / ((wheels_x_distance / 2) + (wheels_y_distance / 2));
  
  }
