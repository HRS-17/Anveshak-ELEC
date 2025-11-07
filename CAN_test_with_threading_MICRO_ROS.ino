#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>
#include "driver/twai.h"

#define RX_PIN 25
#define TX_PIN 26
#define CAN_ID_PWM 0x01

#define CAN_ID_ENC1 0x17
#define CAN_ID_ENC2 0x18
#define CAN_ID_ENC3 0x19
#define CAN_ID_ENC4 0x1A

#define NUM_MOTORS 8
#define NUM_ENCODERS 4

// ---------------- ROS Entities ----------------
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
std_msgs__msg__Int32MultiArray msg_motor_pwm;
std_msgs__msg__Float32MultiArray msg_enc_auto;
rclc_executor_t executor;

// ---------------- Buffers ----------------
int32_t pwm_data_array[NUM_MOTORS];
int32_t last_motor_pwm[NUM_MOTORS] = {0};
float enc_data_array[NUM_ENCODERS] = {0};
bool enc_received_flags[NUM_ENCODERS] = {false};

bool new_pwm_received = false;

// ---------------- Utility Function ----------------
int get_encoder_index(uint32_t id) {
  switch (id) {
    case CAN_ID_ENC1: return 0;
    case CAN_ID_ENC2: return 1;
    case CAN_ID_ENC3: return 2;
    case CAN_ID_ENC4: return 3;
    default: return -1;
  }
}

// ---------------- Subscriber Callback ----------------
void motor_pwm_callback(const void *msgin) {
  const std_msgs__msg__Int32MultiArray *msg =
      (const std_msgs__msg__Int32MultiArray *)msgin;

  if (msg->data.size < NUM_MOTORS) {
    Serial.println("Warning: /motor_pwm_2 has less than 8 elements!");
    return;
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    last_motor_pwm[i] = msg->data.data[i];
  }
  new_pwm_received = true;
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  Serial.println("micro-ROS CAN bridge with encoder sync starting...");

  // --- Initialize CAN ---
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("Failed to install CAN driver");
    while (1);
  }

  if (twai_start() != ESP_OK) {
    Serial.println("Failed to start CAN driver");
    while (1);
  }

  // --- micro-ROS Setup ---
  set_microros_transports();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rclc_support_init(&support, 0, NULL, &allocator);

  rcl_node_t node;
  rclc_node_init_default(&node, "can_motor_node", "", &support);

  // --- Subscriber Setup ---
  msg_motor_pwm.data.data = pwm_data_array;
  msg_motor_pwm.data.size = NUM_MOTORS;
  msg_motor_pwm.data.capacity = NUM_MOTORS;
  msg_motor_pwm.layout.dim.data = NULL;
  msg_motor_pwm.layout.dim.size = 0;
  msg_motor_pwm.layout.dim.capacity = 0;
  msg_motor_pwm.layout.data_offset = 0;

  rclc_subscription_init_default(
      &subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
      "/motor_pwm_2");

  // --- Publisher Setup ---
  msg_enc_auto.data.data = enc_data_array;
  msg_enc_auto.data.size = NUM_ENCODERS;
  msg_enc_auto.data.capacity = NUM_ENCODERS;
  msg_enc_auto.layout.dim.data = NULL;
  msg_enc_auto.layout.dim.size = 0;
  msg_enc_auto.layout.dim.capacity = 0;
  msg_enc_auto.layout.data_offset = 0;

  rclc_publisher_init_default(
      &publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "/enc_auto");

  // --- Executor Setup ---
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg_motor_pwm,
                                 &motor_pwm_callback, ON_NEW_DATA);

  Serial.println("micro-ROS CAN bridge ready ✅");
}

// ---------------- Loop ----------------
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // --- Send PWM over CAN when received ---
  if (new_pwm_received) {
    twai_message_t tx_msg = {};
    tx_msg.identifier = CAN_ID_PWM;
    tx_msg.extd = 0;
    tx_msg.data_length_code = NUM_MOTORS;

    for (int i = 0; i < NUM_MOTORS; i++) {
      tx_msg.data[i] = (uint8_t)last_motor_pwm[i];
    }

    if (twai_transmit(&tx_msg, pdMS_TO_TICKS(10)) == ESP_OK) {
      Serial.print("[TX] PWM: ");
      for (int i = 0; i < NUM_MOTORS; i++) {
        Serial.print(tx_msg.data[i]);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      Serial.println("Failed to send CAN message");
    }

    new_pwm_received = false;
  }

  // --- Read encoder data from CAN ---
  twai_message_t rx_msg;
  if (twai_receive(&rx_msg, pdMS_TO_TICKS(1)) == ESP_OK) {
    int index = get_encoder_index(rx_msg.identifier);
    if (index != -1) {
      // Extract 2-byte encoder value
      int16_t enc_raw = 0;
      memcpy(&enc_raw, rx_msg.data, sizeof(int16_t));
      enc_data_array[index] = (float)enc_raw;
      enc_received_flags[index] = true;

      Serial.printf("[RX] ID: 0x%X | Encoder[%d]: %d\n",
                    rx_msg.identifier, index, enc_raw);
    }
  }

  // --- Check if all encoders received once ---
  bool all_received = true;
  for (int i = 0; i < NUM_ENCODERS; i++) {
    if (!enc_received_flags[i]) {
      all_received = false;
      break;
    }
  }

  // --- Publish when all encoder values available ---
  if (all_received) {
    msg_enc_auto.data.data = enc_data_array;
    msg_enc_auto.data.size = NUM_ENCODERS;
    msg_enc_auto.data.capacity = NUM_ENCODERS;

    if (rcl_publish(&publisher, &msg_enc_auto, NULL) == RCL_RET_OK) {
      Serial.print("[PUB] /enc_auto: ");
      for (int i = 0; i < NUM_ENCODERS; i++) {
        Serial.printf("%.2f ", enc_data_array[i]);
      }
      Serial.println();
    }

    // Reset flags
    for (int i = 0; i < NUM_ENCODERS; i++) {
      enc_received_flags[i] = false;
    }
  }

  delay(1);
}
