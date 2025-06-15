#include <WiFi.h>
#include <HTTPClient.h>

// WiFi credentials
const char* ssid = "POCO M6 PRO 5G";
const char* password = "123456789";

// Server URL
const char* serverUrl = "http://192.168.1.100:5000/get_text";

// Motor pins (example, adjust to your setup)
const int motorPin1 = 5; // GPIO for motor control
const int motorPin2 = 18;

void setup() {
  Serial.begin(115200);
  
  // Setup motor pins
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverUrl);
    
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      Serial.println("Received: " + payload);
      
      // Process commands
      if (payload == "move forward") {
        digitalWrite(motorPin1, HIGH);
        digitalWrite(motorPin2, LOW);
        delay(1000); // Move for 1 second
        digitalWrite(motorPin1, LOW); // Stop
      } else if (payload == "move backward") {
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, HIGH);
        delay(1000);
        digitalWrite(motorPin2, LOW);
      } else if (payload == "say hello") {
        Serial.println("TARS says: Hello!"); // Replace with speaker output
      }
    } else {
      Serial.println("HTTP GET failed, error: " + String(httpCode));
    }
    
    http.end();
  } else {
    Serial.println("WiFi disconnected");
  }
  
  delay(2000); // Poll every 2 seconds
}
