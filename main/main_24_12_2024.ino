/*************************************************
* Automated Irrigation Alpha Version
*
* By: Rafli Triwijaya
* Lab. Alat dan Mesin Pertanian
* 23/12/2024

* Change Log:
* 24-12-2024
* 1. Function added
* 2. Variable revised
* 3. Use FreeRTOS
**************************************************/

// Libraries
#include "FS.h"
#include "SPIFFS.h"
#include <WiFi.h>
#include "time.h"

// Pins
#define FLOWMETER_PIN_1 27
#define FLOWMETER_PIN_2 33
#define ECHO_PIN 18
#define TRIG_PIN 5
#define RELAY_PIN_1 26
#define RELAY_PIN_2 25

// Activate SPIFFS
#define FORMAT_SPIFFS_IF_FAILED true

// Variables for WiFi connection
const char* ssid = "Virus";
const char* password = "1234567890";

// Variables for time settings
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600 * 7; // GMT+7 WIB
const int daylightOffset_sec = 0;
struct tm timeinfo; // Real-time storage

// Variables for flowrate and volume
volatile int flow_frequency_1 = 0; // Flow sensor 1 pulse count
volatile int flow_frequency_2 = 0; // Flow sensor 2 pulse count
float vol_1 = 0.0, vol_2 = 0.0, l_minute_1 = 0.0, l_minute_2 = 0.0;
float volume_per_pulse_1 = 0.004;
float volume_per_pulse_2 = 0.004;

// Relay variables
String relayStatus = "";

// Distance measurement variables
unsigned long duration; // Pulse duration
float distance; // Calculated distance

// Data processing variables
String dataBuffer = "";
unsigned long cloopTime_flow = 0;
unsigned long milsData; // Pulse duration

// Interrupt functions for flow sensors
void IRAM_ATTR flow_1() {
  flow_frequency_1++;
}

void IRAM_ATTR flow_2() {
  flow_frequency_2++;
}

// Function prototypes
void connectWifi(); // Connect to WiFi
void configureTime(); // Configure and retrieve real time
void calculateFlow(); // Calculate flowrate and volume
void saveData(); // Save measurement data to SPIFFS
void readDistance(); // Measure distance with ultrasonic sensor

// FreeRTOS tasks
void flowTask(void *param); // Task for flowrate calculation

void setup() {
  Serial.begin(115200);

  connectWifi(); // Connect to WiFi
  configureTime(); // Configure time

  if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
    Serial.println("SPIFFS Mount Failed");
    return;
  }

  // Set pin modes
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT);

  // Attach interrupts for flowmeter
  attachInterrupt(digitalPinToInterrupt(FLOWMETER_PIN_1), flow_1, RISING);
  attachInterrupt(digitalPinToInterrupt(FLOWMETER_PIN_2), flow_2, RISING);

  // Create FreeRTOS tasks
  xTaskCreate(flowTask, "Flow Task", 4096, NULL, 1, NULL);
}

void loop() {

  configureTime(); // Configure time
  int currentHour = timeinfo.tm_hour;   // Check current hour

  // Check if current hour is between 7 PM (19:00) and 7 AM (7:00)
  if (currentHour >= 19 || currentHour < 7) {
    Serial.println("Going to deep sleep.");
    esp_sleep_enable_timer_wakeup(12 * 60 * 60 * 1000000); // 12 hours in microseconds
    esp_deep_sleep_start();
  } else {
    Serial.println("Remaining awake.");
  }
}

  readDistance();

  if (distance >= 30) {
    setRelay(RELAY_PIN_1, 1);
    setRelay(RELAY_PIN_2, 1);
  } else {
    setRelay(RELAY_PIN_1, 0);
    setRelay(RELAY_PIN_2, 0);
  }

  if ((millis()-milsData) > 60000) {
    milsData = millis();

    saveData();
  }

  delay(500);
}

// FreeRTOS task for flowrate calculation
void flowTask(void *param) {
  while (true) {
    calculateFlow();
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay 1 second
  }
}

// FreeRTOS task for data saving
void spiffsTask(void *param) {
  while (true) {
    saveData();
    vTaskDelay(60000 / portTICK_PERIOD_MS); // Delay 60 seconds
  }
}

// FreeRTOS task for distance measurement
void distanceTask(void *param) {
  while (true) {
    readDistance();
    vTaskDelay(500 / portTICK_PERIOD_MS); // Delay 500 ms
  }
}

// Calculate flowrate and volume
void calculateFlow() {
  unsigned long currentTime_flow = millis();
  if (currentTime_flow >= (cloopTime_flow + 1000)) {
    cloopTime_flow = currentTime_flow; // Update cloopTime
    l_minute_1 = 0;
    // Flow Sensor 1
    if (flow_frequency_1 != 0) {
      l_minute_1 = (flow_frequency_1 * volume_per_pulse_1 * 60);
      vol_1 += l_minute_1 / 60;
      flow_frequency_1 = 0; // Reset counter

      Serial.print("FlowRate Flowmeter 1: ");
      Serial.print(l_minute_1);
      Serial.println(" L/M");
      Serial.print("Volume Flowmeter 1: ");
      Serial.print(vol_1);
      Serial.println(" L");
    }

    l_minute_2 = 0;
    // Flow Sensor 2
    if (flow_frequency_2 != 0) {
      l_minute_2 = (flow_frequency_2 * volume_per_pulse_2 * 60);
      vol_2 += l_minute_2 / 60;
      flow_frequency_2 = 0; // Reset counter

      Serial.print("Flow Rate Flowmeter 2: ");
      Serial.print(l_minute_2);
      Serial.println(" L/M");
      Serial.print("Volume Flowmeter 2: ");
      Serial.print(vol_2);
      Serial.println(" L");
    }
  }
}

// Save data to SPIFFS
void saveData() {

  char time[20];
  strftime(time, sizeof(time), "%H:%M:%S", &timeinfo);

  String data = String(time) + ";" + String(distance) + ";" + String(l_minute_1) + ";" + String(vol_1) + ";" + String(l_minute_2) + ";" + String(vol_2) + ";" + String(relayStatus) + "\n";
  dataBuffer += data;


  File file = SPIFFS.open("/log.csv", FILE_APPEND);
  if (file) {
    file.print(dataBuffer);
    Serial.print("Data saved to SPIFFS: "); Serial.println(dataBuffer);
    file.close();
    dataBuffer = ""; // Clear buffer
  } else {
    Serial.println("Failed to open file for appending.");
  }
  
  
}

// Connect to WiFi
void connectWifi() {
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" CONNECTED");
}

// Configure time
void configureTime() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  }
}

// Measure distance using ultrasonic sensor
void readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH, 38000); // Timeout ~13m
  if (duration == 0) {
    Serial.println("No echo received (timeout).");
    distance = -1; // Invalid distance
  } else {
    distance = duration * 0.0344 / 2; // Calculate distance in cm
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

  }
}

// Setting relay position
void setRelay(byte pinNo,byte kode) {
  if (kode==1) {
    // relay ON
    digitalWrite(pinNo, HIGH);
    relayStatus = "ON";
  }
  else {
    // kode=0
    // relay OFF
    digitalWrite(pinNo, LOW);
    relayStatus = "OFF";
  }
}

