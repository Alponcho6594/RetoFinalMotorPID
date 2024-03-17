#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float64.h>

#include <driver/ledc.h>

const int encoderPinA = 21;  // Connect encoder pin A to digital pin 2
const int encoderPinB = 18;  // Connect encoder pin B to digital pin 3
int ena = 14;
int in1 = 27;
int in2 = 26;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_timer_t timer_1;
rcl_timer_t timer_2;

//ENCENDIDO DEL MOTOR Y PUBLISHER DE DATOS
rcl_publisher_t speed_publisher;
std_msgs__msg__Float64 speed_msg;

//CALCULO DE VELOCIDADES ANGULARES
rcl_publisher_t angular_speed_publisher;
std_msgs__msg__Float64 angular_speed_msg;

//SUBSCRIBER DE SETPOINT
rcl_subscription_t pwm_subscriber;
std_msgs__msg__Float64  pwm_msg;

//LEDC
const int freq = 5000;
const int pwmChannel = 0;
const int resolution = 8;

//EXECUTOR 
rclc_executor_t executor;

//CONSTANTES DE CONTROL
float Kp = 0.895;;//1.3852;
float Kd = 1.478;//0.5338;
float Ki = 0.644;//2.1352;

//VALOR SETPOINT DADO POR USUARIO
float duty_cycle;

//OBTENCION DE TIEMPOS EN EL TIMER
unsigned long prevMillis = 0;

//TICS O PULSOS DETECTADOS
int encoderPulseCount = 0;

//VALORES DE VELOCIDAD ANGULAR
double angular_vel_actual = 0;
double angular_vel_deseada = 0;

//VALOR DE RETROALIMENTACION ENVIADO AL MOTOR
double pwm = 0;

//PULSOS CADA 100 ms
int pulsesSample = 113;

//MAXIMO RPM
int maxRPM = 140;

double integrative = 0;

//Sampling Time
int sampleTime = 100;

//Errores
double lastError = 0;
double error = 0;

#define LED_PIN 15
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

double PID(double error){

  double proportional = error;
  double derivative = (error - lastError) / sampleTime;
  integrative += error * sampleTime;
      
  double gain = Kp * proportional + Kd * derivative + Ki * integrative;

  lastError = error;

  return gain;
}

void updateEncoder(){
  if(digitalRead(encoderPinB) == HIGH){

    encoderPulseCount ++;

  }else{

    encoderPulseCount --;
  }

}


void timer_1_callback(rcl_timer_t * timer_1, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);

  unsigned long actualMillis = millis();

  if(actualMillis - prevMillis >= sampleTime){

    double RPMdesired = (duty_cycle * maxRPM);
    angular_vel_deseada = (RPMdesired * 2 * PI)/60;
    double RPMactual = (encoderPulseCount * maxRPM) / pulsesSample;
    angular_vel_actual = (RPMactual * 2 * PI)/60;
    error = angular_vel_deseada - angular_vel_actual;

    pwm = PID(error);

    encoderPulseCount = 0;
    prevMillis = actualMillis;      
  }
}

void timer_2_callback(rcl_timer_t * timer_2, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  //double pwmMotor = abs(pwm);
  if (duty_cycle <= 0){

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(pwmChannel, pwm);

  }else {

    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(pwmChannel, pwm*(-1));
  }

  speed_msg.data = (double) angular_vel_actual;
  RCSOFTCHECK(rcl_publish(&speed_publisher, &speed_msg, NULL));
  angular_speed_msg.data = (double) angular_vel_deseada;
  RCSOFTCHECK(rcl_publish(&angular_speed_publisher, &angular_speed_msg, NULL));  
}

void subscription_callback(const void * msgin) {

  const std_msgs__msg__Float64 * msg = static_cast<const std_msgs__msg__Float64 *>(msgin);
  duty_cycle = static_cast<float>(msg->data);

}

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(ena, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  delay(2000);
  
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ena, pwmChannel);

  allocator = rcl_get_default_allocator();

  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, RISING);

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "Controller", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &speed_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "/angular_speed_control"));

  RCCHECK(rclc_publisher_init_default(
      &angular_speed_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
      "/angular_speed"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &pwm_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
      "/setpoint"));
      
  // create timer,
  const unsigned int timer_1_timeout = 10;
  RCCHECK(rclc_timer_init_default(
    &timer_1,
    &support,
    RCL_MS_TO_NS(timer_1_timeout),
    timer_1_callback));

  const unsigned int timer_2_timeout = 10;
  RCCHECK(rclc_timer_init_default(
    &timer_2,
    &support,
    RCL_MS_TO_NS(timer_2_timeout),
    timer_2_callback));
   

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_1));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_2));
  RCCHECK(rclc_executor_add_subscription(&executor, &pwm_subscriber, &pwm_msg, &subscription_callback, ON_NEW_DATA));
  pwm_msg.data = 0;
}

void loop() {
  delay(10);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
