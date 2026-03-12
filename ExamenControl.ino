#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

// ----------- MOTOR A -----------
#define IN1_GPIO 25
#define IN2_GPIO 26
#define PWM_A    27
#define PHASE_A 35 
#define PHASE_B 34

// ----------- MOTOR B -----------
#define IN3_GPIO 21
#define IN4_GPIO 19
#define PWM_B    18
#define PHASE_A 32
#define PHASE_B 33

// PWM configuración
const int freq = 1000;
const int resolution = 8;

const int channel_A = 0;
const int channel_B = 1;

// robot parameters
float wheel_base = 0.19;

// ROS
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ----------- MOTOR CONTROL -----------

void setMotorA(float speed)
{
  int pwm = constrain(abs(speed)*255,0,255);

  if(speed > 0){
    digitalWrite(IN1_GPIO, HIGH);
    digitalWrite(IN2_GPIO, LOW);
  }
  else{
    digitalWrite(IN1_GPIO, LOW);
    digitalWrite(IN2_GPIO, HIGH);
  }

  ledcWrite(channel_A, pwm);
}

void setMotorB(float speed)
{
  int pwm = constrain(abs(speed)*255,0,255);

  if(speed > 0){
    digitalWrite(IN3_GPIO, HIGH);
    digitalWrite(IN4_GPIO, LOW);
  }
  else{
    digitalWrite(IN3_GPIO, LOW);
    digitalWrite(IN4_GPIO, HIGH);
  }

  ledcWrite(channel_B, pwm);
}

// ----------- CMD_VEL CALLBACK -----------

void cmd_vel_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * twist = (const geometry_msgs__msg__Twist *)msgin;

  float v = twist->linear.x;
  float w = twist->angular.z;

  float v_r = v + (wheel_base/2.0)*w;
  float v_l = v - (wheel_base/2.0)*w;

  setMotorA(v_l);
  setMotorB(v_r);
}

// ----------- SETUP -----------

void setup()
{
  set_microros_transports();

  pinMode(IN1_GPIO, OUTPUT);
  pinMode(IN2_GPIO, OUTPUT);
  pinMode(IN3_GPIO, OUTPUT);
  pinMode(IN4_GPIO, OUTPUT);

  ledcSetup(channel_A, freq, resolution);
  ledcAttachPin(PWM_A, channel_A);

  ledcSetup(channel_B, freq, resolution);
  ledcAttachPin(PWM_B, channel_B);

  delay(2000);

  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(
    &node,
    "esp32_motor_node",
    "",
    &support);

  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel");

  rclc_executor_init(&executor, &support.context, 1, &allocator);

  rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &msg,
    &cmd_vel_callback,
    ON_NEW_DATA);
}

// ----------- LOOP -----------

void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}