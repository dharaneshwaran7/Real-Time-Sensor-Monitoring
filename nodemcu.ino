#include <Wire.h>
#include <ESP8266WiFi.h>
#include <math.h>

// Wi-Fi Credentials
const char* ssid = "YourWiFiSSID";  
const char* password = "YourWiFiPassword";  

// ADXL345 Definitions
#define ADXL345_ADDRESS 0x53
#define POWER_CTL 0x2D
#define DATA_FORMAT 0x31
#define DATAX0 0x32

const int SAMPLE_COUNT = 100;
float x_offset = 0.0, y_offset = 0.0, z_offset = 0.0;

// Ultrasonic Sensor Pins
#define TRIG1 D5
#define ECHO1 D6
#define TRIG2 D7
#define ECHO2 D8

// Piezoelectric Sensor Pin
#define PIEZO_PIN A0

// Variables for vibration calculations
float prev_magnitude = 0.0;
unsigned long prev_time = 0;

// Wi-Fi Server
WiFiServer server(80);

void setup() {
  Serial.begin(115200);
  Wire.begin(D2, D1); // SDA (D2), SCL (D1)

  // Initialize ADXL345
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(POWER_CTL);
  Wire.write(0x08); // Measurement mode
  Wire.endTransmission();

  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(DATA_FORMAT);
  Wire.write(0x0B); // Full resolution, +/-16g
  Wire.endTransmission();

  // Initialize Ultrasonic Sensors
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);

  // Start Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.begin();

  Serial.println("Calibrating...");
  calibrateSensor();
  Serial.println("Calibration complete.");
}

void loop() {
  // Handle client connection
  WiFiClient client = server.available();
  if (client) {
    Serial.println("Client Connected.");
    String request = client.readStringUntil('\r');
    client.flush();

    // Read sensor data
    float distance1 = getUltrasonicDistance(TRIG1, ECHO1);
    float distance2 = getUltrasonicDistance(TRIG2, ECHO2);
    int piezo_raw = analogRead(PIEZO_PIN);
    float piezo_voltage = piezo_raw * (3.3 / 1023.0);
    float vibration_speed = getVibrationSpeed();
    float vibration_intensity = vibration_speed > 1.0 ? vibration_speed : 0.0;

    // Generate HTML page
    String html = "<!DOCTYPE html><html><head><title>Sensor Data</title>";
    html += "<script>";
    html += "function refreshPage() { location.reload(); }";
    html += "setTimeout(refreshPage, 1000);"; // Auto-refresh every 1 second
    html += "</script>";
    html += "</head><body>";
    html += "<h1>Real-time Sensor Data</h1>";
    html += "<p><strong>Distance 1 (D1):</strong> " + String(distance1, 2) + " cm</p>";
    html += "<p><strong>Distance 2 (D2):</strong> " + String(distance2, 2) + " cm</p>";
    html += "<p><strong>Piezo Voltage:</strong> " + String(piezo_voltage, 2) + " V</p>";
    html += "<p><strong>Vibration Speed:</strong> " + String(vibration_speed, 2) + "</p>";
    html += "<p><strong>Vibration Intensity:</strong> " + String(vibration_intensity, 2) + "</p>";
    html += "</body></html>";


    // Send HTML to client
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();
    client.println(html);
    client.stop();
    Serial.println("Client Disconnected.");
  }

  delay(500); // Update interval
}

float getUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance = (duration * 0.034) / 2.0;
  return distance;
}

float getVibrationSpeed() {
  int16_t x_raw, y_raw, z_raw;

  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(DATAX0);
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345_ADDRESS, 6, true);

  if (Wire.available() == 6) {
    x_raw = (int16_t)(Wire.read() | (Wire.read() << 8));
    y_raw = (int16_t)(Wire.read() | (Wire.read() << 8));
    z_raw = (int16_t)(Wire.read() | (Wire.read() << 8));
  }

  float scale = 0.0039;
  float x = (x_raw * scale) - x_offset;
  float y = (y_raw * scale) - y_offset;
  float z = (z_raw * scale) - z_offset;

  float magnitude = sqrt(x * x + y * y + z * z);

  unsigned long current_time = millis();
  float vibration_speed = (magnitude - prev_magnitude) / ((current_time - prev_time) / 1000.0);

  prev_magnitude = magnitude;
  prev_time = current_time;

  return abs(vibration_speed);
}

void calibrateSensor() {
  int samples = 100;
  int16_t x_raw, y_raw, z_raw;
  long x_sum = 0, y_sum = 0, z_sum = 0;

  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(ADXL345_ADDRESS);
    Wire.write(DATAX0);
    Wire.endTransmission(false);
    Wire.requestFrom(ADXL345_ADDRESS, 6, true);

    if (Wire.available() == 6) {
      x_raw = (int16_t)(Wire.read() | (Wire.read() << 8));
      y_raw = (int16_t)(Wire.read() | (Wire.read() << 8));
      z_raw = (int16_t)(Wire.read() | (Wire.read() << 8));
    }

    x_sum += x_raw;
    y_sum += y_raw;
    z_sum += z_raw;

    delay(10);
  }

  float scale = 0.0039;
  x_offset = (x_sum / samples) * scale;
  y_offset = (y_sum / samples) * scale;
  z_offset = ((z_sum / samples) * scale) - 1.0;
}
