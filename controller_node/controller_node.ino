#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>

#include <PID_v1.h>

rcl_publisher_t publisher_as;
rcl_publisher_t publisher_voltage;
rcl_publisher_t publisher_pid;
rcl_subscription_t subscriber_sp;

std_msgs__msg__Float32 msg;
std_msgs__msg__Int32 msg_v;
std_msgs__msg__Float32 msg_pid;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_timer_t timer_as;
rcl_timer_t timer_motor;

#define LED_PIN 13
//Modificar
#define ENCODERA_PIN 12
#define ENCODERB_PIN 14
#define MOTOR_EN 26
#define MOTOR_PIN1 27
#define MOTOR_PIN2 25

int encoderTics = 0;
unsigned long prevMillis = 0;
double desired_angular_velocity = 0;
double current_angular_velocity = 0;
float duty_cycle = 0;

//ITAE
double kp = 3.31661;
double ki = 2.10717;
double kd = 0.6821;

double pwm_pid = 0;
double dt = 0;
double past_time = 0;
double pwm = 0;
double integral = 0;
double previous = 0;

const int PWM_CHANNEL = 0;
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback_as(rcl_timer_t * timer_as, int64_t last_call_time){  
  RCLC_UNUSED(last_call_time);
  unsigned long currMillis = millis();
  if (currMillis - prevMillis >= 100){
    current_angular_velocity = ((2 * PI * encoderTics) / (1620)) * 10;

    desired_angular_velocity = ((duty_cycle * 7.5) / 1);
    double current_time = millis();
    dt = (current_time - past_time) / 100;
    past_time = current_time;
    double error = desired_angular_velocity - current_angular_velocity;
    pwm = pid(error);

    encoderTics = 0;
    prevMillis = currMillis;
  }
}

void timer_callback_motor(rcl_timer_t * timer_motor, int64_t last_call_time){  
  RCLC_UNUSED(last_call_time);
  
  if(pwm < 0){
    digitalWrite(MOTOR_PIN1, HIGH);
    digitalWrite(MOTOR_PIN2, LOW);
    ledcWrite(PWM_CHANNEL, pwm *(-1));
  } 
  else{
    digitalWrite(MOTOR_PIN1, LOW);
    digitalWrite(MOTOR_PIN2, HIGH);
    ledcWrite(PWM_CHANNEL, pwm);
  }
  msg_v.data = (duty_cycle*12);
  RCSOFTCHECK(rcl_publish(&publisher_voltage, &msg_v, NULL));
  msg.data = current_angular_velocity;
  RCSOFTCHECK(rcl_publish(&publisher_as, &msg, NULL));
  msg_pid.data = pwm;
  RCSOFTCHECK(rcl_publish(&publisher_pid, &msg_pid, NULL));
}

double pid(double error)
{
  double proportional = error;
  integral += error * dt;
  double derivative = (error - previous) / dt;
  previous = error;
  double pid = (kp * proportional) + (ki * integral) + (kd * derivative);
  return pid;
}

void subscription_callback_sp(const void * msgin){
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  duty_cycle = msg->data;
}

void updateTics() {
  if (digitalRead(ENCODERB_PIN) == HIGH) {
    encoderTics++;
  } else {
    encoderTics--;
  }
}

void setup() {
  set_microros_transports();
  
  //ERROR_LED SETUP
  pinMode(LED_PIN, OUTPUT);

  //ENCODERS SETUP
  pinMode(ENCODERA_PIN, INPUT);
  pinMode(ENCODERB_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODERA_PIN), updateTics, RISING);
  
  //MOTOR SETUP
  pinMode(MOTOR_EN, OUTPUT);
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
  digitalWrite(MOTOR_PIN1, HIGH);
  digitalWrite(MOTOR_PIN2, LOW);

  //PWM SETUP
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_EN, PWM_CHANNEL);

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "controller", "", &support));

  // create angular_speed publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_as,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/angular_speed"));

  // create voltage publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_voltage,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/voltage"));

  // create pid publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_pid,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/pid"));

  // create subscriber_sp, setpoint -> duty_cycle
  RCCHECK(rclc_subscription_init_default(
    &subscriber_sp,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/setpoint"));

  // create timer_as, 100 ms, angular_speed and pid
  const unsigned int timer_timeout_as = 100;
  RCCHECK(rclc_timer_init_default(
    &timer_as,
    &support,
    RCL_MS_TO_NS(timer_timeout_as),
    timer_callback_as));

  // create timer_motor, 20 ms, motor
  const unsigned int timer_timeout_motor = 20;
  RCCHECK(rclc_timer_init_default(
    &timer_motor,
    &support,
    RCL_MS_TO_NS(timer_timeout_motor),
    timer_callback_motor));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_sp, &msg, &subscription_callback_sp, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_as));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_motor));

  msg.data = 0;
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}