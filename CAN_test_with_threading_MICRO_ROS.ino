#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include "driver/twai.h"

#define RX_PIN 2
#define TX_PIN 1
#define CAN_ID_PWM 0x01

#define NUM_MOTORS 8

rcl_subscription_t subscriber;
std_msgs__msg__Int32MultiArray msg_motor_pwm;
rclc_executor_t executor;

// Storage for subscriber data
int32_t last_motor_pwm[NUM_MOTORS] = {0};
bool new_pwm_received = false;

// Static buffer for subscriber
int32_t pwm_data_array[NUM_MOTORS];

// Callback
void motor_pwm_callback(const void *msgin) {
    const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)msgin;

    if (msg->data.size < NUM_MOTORS) {
        Serial.println("Warning: /motor_pwm has less than 8 elements!");
        return;
    }

    for (int i = 0; i < NUM_MOTORS; i++) {
        last_motor_pwm[i] = msg->data.data[i];
    }
    new_pwm_received = true;
}

void setup() {
    Serial.begin(115200);
    Serial.println("micro-ROS CAN subscriber started");

    // --- Initialize CAN ---
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
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

    // --- micro-ROS initialization ---
    set_microros_transports();
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    rcl_node_t node;
    rclc_node_init_default(&node, "can_motor_node", "", &support);

    // --- Subscriber setup ---
    msg_motor_pwm.data.data = pwm_data_array;  // static buffer
    msg_motor_pwm.data.size = NUM_MOTORS;
    msg_motor_pwm.data.capacity = NUM_MOTORS;
    msg_motor_pwm.layout.dim.data = NULL;
    msg_motor_pwm.layout.dim.size = 0;
    msg_motor_pwm.layout.dim.capacity = 0;
    msg_motor_pwm.layout.data_offset = 0;

    rclc_subscription_init_default(
        &subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "/motor_pwm");

    // Executor
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg_motor_pwm,
                                   &motor_pwm_callback, ON_NEW_DATA);
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    // --- Send received PWM values over CAN ---
    if (new_pwm_received) {
        twai_message_t can_msg = {};
        can_msg.identifier = CAN_ID_PWM;
        can_msg.extd = 0;
        can_msg.data_length_code = NUM_MOTORS;

        for (int i = 0; i < NUM_MOTORS; i++) {
            can_msg.data[i] = (uint8_t)last_motor_pwm[i];
        }

        if (twai_transmit(&can_msg, pdMS_TO_TICKS(10)) == ESP_OK) {
            Serial.print("Sent CAN: ");
            for (int i = 0; i < NUM_MOTORS; i++) {
                Serial.print(can_msg.data[i]);
                Serial.print(" ");
            }
            Serial.println();
        } else {
            Serial.println("Failed to send CAN message");
        }

        new_pwm_received = false;
    }

    delay(1);
}
