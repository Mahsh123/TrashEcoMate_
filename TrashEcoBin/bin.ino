#include <WiFi.h>
#include <Firebase_ESP_Client.h>

// Wi-Fi credentials
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// Firebase credentials
#define FIREBASE_HOST "https://your-project-id.firebaseio.com/" // Replace with your Firebase Database URL
#define FIREBASE_AUTH "YOUR_DATABASE_SECRET_OR_API_KEY"         // Replace with your Database Secret or API Key

// Ultrasonic Sensor Pins
#define TRIG_PIN 5  // GPIO 5 for Trigger
#define ECHO_PIN 18 // GPIO 18 for Echo

// Bin dimensions (in cm)
const float BIN_DEPTH = 30.0; // Distance from sensor to bottom
const float TOTAL_HEIGHT = 35.0; // Total height including sensor offset

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

void setup() {
  Serial.begin(115200);

  // Ultrasonic sensor pin setup
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Connect to Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.println("Connected to Wi-Fi");

  // Configure Firebase
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH; // Use legacy token (Database Secret)
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void loop() {
  // Measure distance
  float distance = measureDistance();

  // Calculate waste level and status
  String status = calculateWasteLevel(distance);

  // Send data to Firebase
  sendToFirebase(distance, status);

  delay(5000); // Update every 5 seconds
}

float measureDistance() {
  // Trigger the ultrasonic sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure the echo pulse duration
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate distance in cm (speed of sound = 343 m/s, or 0.0343 cm/us)
  float distance = (duration * 0.0343) / 2;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  return distance;
}

String calculateWasteLevel(float distance) {
  String status;
  float levelPercentage;

  // Adjust distance for bin depth
  if (distance >= 35.0 && distance <= 40.0) {
    levelPercentage = 0.0; // Bin is empty
    status = "Empty";
  } 
  else if (distance > BIN_DEPTH && distance < 35.0) {
    levelPercentage = 0.0; // Still considered empty
    status = "Empty";
  } 
  else if (distance <= BIN_DEPTH && distance > 5.0) {
    // Calculate percentage: 100% at 5 cm, 0% at 30 cm
    levelPercentage = ((BIN_DEPTH - distance) / (BIN_DEPTH - 5.0)) * 100.0;
    status = "Normal";
  } 
  else if (distance <= 5.0 && distance > 0.0) {
    levelPercentage = 100.0; // Full
    status = "Overflow Warning";
  } 
  else if (distance <= 0.0 || distance > 40.0) {
    levelPercentage = -1.0; // Invalid measurement
    status = "Obstructed Sensor";
  } 
  else {
    levelPercentage = -1.0; // Error case
    status = "Error";
  }

  Serial.print("Level: ");
  Serial.print(levelPercentage);
  Serial.print("%, Status: ");
  Serial.println(status);
  return status;
}

void sendToFirebase(float distance, String status) {
  if (Firebase.ready()) {
    // Send distance
    Firebase.RTDB.setFloat(&fbdo, "/wastebin/distance", distance);
    // Send status
    Firebase.RTDB.setString(&fbdo, "/wastebin/status", status);

    Serial.println("Data sent to Firebase");
  } else {
    Serial.println("Firebase not ready");
    Serial.println(fbdo.errorReason());
  }
}