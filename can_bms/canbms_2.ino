#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "driver/twai.h"

#define SDA_PIN 8
#define SCL_PIN 9

#define RX_PIN 25
#define TX_PIN 26

#define TRANSMIT_RATE_MS 100

#define diagnostic_identifier 0x51
#define normal_identifier 0x50

Adafruit_ADS1115 ads1;
Adafruit_ADS1115 ads2;
Adafruit_ADS1115 ads3;

bool ads1_ok = false, ads2_ok = false, ads3_ok = false, can_ok = false;

unsigned long previousMillis = 0, currentMillis = 0;

typedef struct {
  float ads1_volts[4];
  float ads2_volts[4];
  float ads3_volts[4];
} feedback_t;

feedback_t feedback = {0};
float voltages[12] = {0.0f};
const char *labels[] = {
  "B1C1", "B1C2", "B1C3", "B1C4", "B1C5", "B1C6",
  "B2C1", "B2C2", "B2C3", "B2C4", "B2C5", "B2C6"
};

void setup() {
  Serial.begin(115200);       // ✅ Serial debug at 115200 baud
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);

  can_ok = CAN_Init();

  ads1_ok = ads1.begin(0x48, &Wire);
  if (ads1_ok) ads1.setGain(GAIN_ONE);
  ads2_ok = ads2.begin(0x49, &Wire);
  if (ads2_ok) ads2.setGain(GAIN_ONE);
  ads3_ok = ads3.begin(0x4A, &Wire);
  if (ads3_ok) ads3.setGain(GAIN_ONE);

  if (ads1_ok) { send_status_message("I:48"); Serial.println("ADS1115 0x48 OK"); }
  else { send_status_message("E:48"); Serial.println("ADS1115 0x48 ERROR"); }

  if (ads2_ok) { send_status_message("I:49"); Serial.println("ADS1115 0x49 OK"); }
  else { send_status_message("E:49"); Serial.println("ADS1115 0x49 ERROR"); }

  if (ads3_ok) { send_status_message("I:4A"); Serial.println("ADS1115 0x4A OK"); }
  else { send_status_message("E:4A"); Serial.println("ADS1115 0x4A ERROR"); }

  if (can_ok) { send_status_message("I:CAN"); Serial.println("CAN bus OK"); }
  else { send_status_message("E:CAN"); Serial.println("CAN bus ERROR"); }
}

void loop() {
  currentMillis = millis();

  if (!can_ok) {
    delay(1000);
    return;
  }

  read_ADS();
  changeCumulativeToDifference();

  if (currentMillis - previousMillis >= TRANSMIT_RATE_MS) {
    previousMillis = currentMillis;
    send_feedback();
  }

  delay(10);
}

void read_ADS() {
  if (ads1_ok) {
    for (int i = 0; i < 4; i++) {
      int16_t adc = ads1.readADC_SingleEnded(i);
      feedback.ads1_volts[i] = adc * 4.096 * 11.0 / 32768.0;
    }
  }

  if (ads2_ok) {
    for (int i = 0; i < 4; i++) {
      int16_t adc = ads2.readADC_SingleEnded(i);
      feedback.ads2_volts[i] = adc * 4.096 * 11.0 / 32768.0;
    }
  }

  if (ads3_ok) {
    for (int i = 0; i < 4; i++) {
      int16_t adc = ads3.readADC_SingleEnded(i);
      feedback.ads3_volts[i] = adc * 4.096 * 11.0 / 32768.0 ;
    }
  }
}

bool CAN_Init() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) return false;
  if (twai_start() != ESP_OK) return false;

  uint32_t alerts = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_TX_FAILED;
  if (twai_reconfigure_alerts(alerts, NULL) != ESP_OK) return false;

  return true;
}

void send_status_message(const char* text) {
  if (!can_ok) return;

  twai_message_t msg = {0};
  msg.identifier = diagnostic_identifier;
  msg.data_length_code = strlen(text) > 8 ? 8 : strlen(text);
  memcpy(msg.data, text, msg.data_length_code);

  twai_transmit(&msg, pdMS_TO_TICKS(100));

  Serial.print("Status CAN Msg Sent: ");
  Serial.println(text);
}

void send_feedback() {
  if (!can_ok) return;
  for (int i=0; i<12; i++){
    twai_message_t msg = {0};
    msg.identifier = normal_identifier;
    msg.data_length_code = 8;

    memcpy(&msg.data[0], labels[i], 4);
    memcpy(&msg.data[4], &voltages[i], 4);
    
    if (twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK) {
      Serial.print("Sent: ");
      Serial.print(labels[i]);
      Serial.print(" = ");
      Serial.println(voltages[i], 3);
    }
    else {
      Serial.print("CAN TX Fail: ");
      Serial.println(labels[i]);
    }
  }
}

void changeCumulativeToDifference(){
  // Battery 1
  voltages[0]   = feedback.ads1_volts[0];                             // B1 C1
  voltages[1]   = feedback.ads1_volts[1] - feedback.ads1_volts[0];    // B1 C2
  voltages[2]   = feedback.ads1_volts[2] - feedback.ads1_volts[1];    // B1 C3
  voltages[3]   = feedback.ads1_volts[3] - feedback.ads1_volts[2];    // B1 C4
  voltages[4]   = feedback.ads2_volts[0] - feedback.ads1_volts[3];    // B1 C5
  voltages[5]   = feedback.ads2_volts[1] - feedback.ads2_volts[0];    // B1 C6

  // Battery 2
  voltages[6]   = feedback.ads2_volts[2];                             // B2 C1
  voltages[7]   = feedback.ads2_volts[3] - feedback.ads2_volts[2];    // B2 C2
  voltages[8]   = feedback.ads3_volts[0] - feedback.ads2_volts[3];    // B2 C3
  voltages[9]   = feedback.ads3_volts[1] - feedback.ads3_volts[0];    // B2 C4
  voltages[10]  = feedback.ads3_volts[2] - feedback.ads3_volts[1];    // B2 C5
  voltages[11]  = feedback.ads3_volts[3] - feedback.ads3_volts[2];    // B2 C6
}
