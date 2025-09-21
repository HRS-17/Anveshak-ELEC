#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>

#define LED_PIN 13
#define NUM_ENC 3

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { (void)fn; }

rcl_subscription_t subscriber;
std_msgs__msg__Int32MultiArray pwm_int;  // message storage for subscriber

rcl_publisher_t publisher;
std_msgs__msg__Float32MultiArray enc_msg;  // message storage for publisher

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

volatile long enc_pos[NUM_ENC] = {0};
volatile bool A_set[NUM_ENC] = {false};
volatile bool B_set[NUM_ENC] = {false};

int PWMpin[6] = {4, 6, 16, 18, 9, 11};
int dirpin[6] = {5, 7, 15, 17, 8, 10};
int defaultdir[6] = {1, 1, 1, 1, 1, 1};

int encB[NUM_ENC] = {42, 38, 47};  // ESP32-S3 pins
int encA[NUM_ENC] = {41, 37, 21};

int pwm_int_buffer[6] = {0};
float enc_feed[NUM_ENC] = {0};

// LED blink on error
void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Subscriber callback: copy incoming Int32MultiArray data into pwm_int_buffer
void subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32MultiArray *msg = 
      (const std_msgs__msg__Int32MultiArray *)msgin;
  for (int i = 0; i < 6; i++) {
    pwm_int_buffer[i] = msg->data.data[i];
  }
}

// Timer callback: apply PWM to motors and publish encoders
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);

  // Drive motors based on pwm_int_buffer
  for (int i = 0; i < 6; i++) {
    if (pwm_int_buffer[i] >= 0) {
      analogWrite(PWMpin[i], min(255, pwm_int_buffer[i]));
      digitalWrite(dirpin[i], defaultdir[i]);
    } else {
      analogWrite(PWMpin[i], min(255, -pwm_int_buffer[i]));
      digitalWrite(dirpin[i], 1 - defaultdir[i]);
    }
  }

  // Read encoders and convert to degrees
  for (int j = 0; j < NUM_ENC; j++) {
    enc_feed[j] = (enc_pos[j] * 90.0 / 55157.0);
  }

  // Prepare and publish Float32MultiArray message
  enc_msg.data.data = enc_feed;   // set pointer to static array
  enc_msg.data.size = NUM_ENC;
  enc_msg.data.capacity = NUM_ENC;
  RCSOFTCHECK(rcl_publish(&publisher, &enc_msg, NULL));
}

void setup() {
  set_microros_transports();
  Serial.begin(9600);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(2000);  // wait for agent

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "microros_bevel_arm", "", &support));

  // Create subscriber (stm_write)
  RCCHECK(rclc_subscription_init_default(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "stm_write"));

  // Create publisher (enc_arm)
  RCCHECK(rclc_publisher_init_default(
    &publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "enc_arm"));

  // Timer for periodic updates
  const unsigned int timer_timeout = 50;
  RCCHECK(rclc_timer_init_default(
    &timer, &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Executor with 2 handles: subscriber and timer
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &pwm_int,
                                         &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Encoder pin setup and interrupts
  for (int i = 0; i < NUM_ENC; i++) {
    pinMode(encA[i], INPUT_PULLUP);
    pinMode(encB[i], INPUT_PULLUP);
    pinMode(PWMpin[i], OUTPUT);
    pinMode(dirpin[i], OUTPUT);
  }
  attachInterrupt(digitalPinToInterrupt(encA[0]), callback_0, RISING);
  attachInterrupt(digitalPinToInterrupt(encA[1]), callback_1, RISING);
  attachInterrupt(digitalPinToInterrupt(encA[2]), callback_2, RISING);

  // Initialize Float32MultiArray layout (empty) and data sequence
  enc_msg.layout.data_offset = 0;
  enc_msg.layout.dim.data = NULL;
  enc_msg.layout.dim.size = 0;
  enc_msg.layout.dim.capacity = 0;
  enc_msg.data.data = enc_feed;
  enc_msg.data.size = NUM_ENC;
  enc_msg.data.capacity = NUM_ENC;
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

// Encoder interrupt service routines (same as original rosserial code)
void callback_0() {
  int i = 0;
  A_set[i] = digitalRead(encA[i]);
  B_set[i] = digitalRead(encB[i]);
  if (A_set[i] == B_set[i]) enc_pos[i]++; else enc_pos[i]--;
}
void callback_1() {
  int i = 1;
  A_set[i] = digitalRead(encA[i]);
  B_set[i] = digitalRead(encB[i]);
  if (A_set[i] == B_set[i]) enc_pos[i]++; else enc_pos[i]--;
}
void callback_2() {
  int i = 2;
  A_set[i] = digitalRead(encA[i]);
  B_set[i] = digitalRead(encB[i]);
  if (A_set[i] == B_set[i]) enc_pos[i]++; else enc_pos[i]--;
}
