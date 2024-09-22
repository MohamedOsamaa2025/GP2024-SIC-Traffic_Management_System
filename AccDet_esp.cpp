#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <NimBLEDevice.h>

// Define the UUIDs for the service and characteristic
#define SERVICE_UUID           "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID    "12345678-1234-5678-1234-56789abcdef1"

// Create a BLE server and characteristic
NimBLEServer* pServer = nullptr;
NimBLEService* pService = nullptr;
NimBLECharacteristic* pCharacteristic = nullptr;

Adafruit_MPU6050 mpu;

float lastGyroZ = 0.0; // Variable to hold the last Z gyro value
bool significantChangeReported = false; // Flag to track if a significant change has been reported

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}

void startBLEServer() {
  // Initialize BLE
  NimBLEDevice::init("ESP32_BLE_Server");

  // Create BLE Server
  pServer = NimBLEDevice::createServer();

  // Create BLE Service
  pService = pServer->createService(SERVICE_UUID);

  // Create BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE
  );

  // Set initial value for the characteristic
  pCharacteristic->setValue("Initial value");

  // Start the service
  pService->start();

  // Start advertising
  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

  Serial.println("BLE Server is running...");
}

void stopBLEServer() {
  if (pServer) {
    NimBLEDevice::deinit(); // Stop the BLE server and deinitialize
    pServer = nullptr;
    pService = nullptr;
    pCharacteristic = nullptr;
    Serial.println("BLE Server stopped.");
  }
}

void loop() {
  // Get new sensor events with the readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Check for significant change in Z gyro
  float currentGyroZ = g.gyro.z;
  Serial.println(currentGyroZ);

  if (abs(currentGyroZ - lastGyroZ) > 0.35) {
    if (!significantChangeReported) {
      Serial.println("Significant change detected in Z gyro.");
      significantChangeReported = true;

      // Start BLE server only if it's not already running
      if (!pServer) {
        startBLEServer();
      }

      // Update the BLE characteristic with a message indicating an accident
      pCharacteristic->setValue("Accident detected!");
      pCharacteristic->notify(); // Notify connected devices
    }
  } else {
    if (significantChangeReported) {
      // Stop the BLE server if the accident state has cleared
      stopBLEServer();
      delay(1000); // Wait for 1 second before allowing the server to start again
      significantChangeReported = false; // Reset the flag if no significant change
    }
  }

  // Update the last Z gyro value
  lastGyroZ = currentGyroZ;

  delay(500); // Adjust delay as needed
}