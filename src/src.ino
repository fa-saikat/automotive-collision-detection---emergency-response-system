#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>
#include <Wire.h>

#define BUZZER_PIN 3        // Buzzer Pin

#define TRIG_PIN 12         // Sonar Trig Pin
#define ECHO_PIN 13         // Sonar Echo Pin

#define SIM808_TX 7         // TX for SoftwareSerial
#define SIM808_RX 8         // RX for SoftwareSerial

#define FLAME_PIN 2         // Flame Sensor Pin

#define MOTOR_ENABLE_PIN 9  // ENA on L298N
#define MOTOR_INPUT1_PIN 10 // IN1 on L298N
#define MOTOR_INPUT2_PIN 11 // IN2 on L298N

#define PHONE_NUMBER "01618728178"  // To Send SMS

// Message content
#define MESSAGE                                                                \
  "Fire Detected!!\nHelp "                                                     \
  "Me\nLatitude:23.872526592985242\nLongitude:90.30914854319799\nLocation: "   \
  "https://maps.app.goo.gl/JbfBAPVdPbDX2VfS6"

Adafruit_MPU6050 mpu;
SoftwareSerial sim808Serial(SIM808_TX, SIM808_RX);

float duration, distance;   // Duration & Distance using Sonar

void setup() {
    // Buzzer
    pinMode(BUZZER_PIN, OUTPUT);

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    pinMode(FLAME_PIN, INPUT);

    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    pinMode(MOTOR_INPUT1_PIN, OUTPUT);
    pinMode(MOTOR_INPUT2_PIN, OUTPUT);

    Serial.begin(9600);
    sim808Serial.begin(9600);
    while (!Serial) {
        delay(10);
    }

    analogWrite(MOTOR_ENABLE_PIN, 255);   // Set the motor to always ON initially

    Serial.println("Adafruit MPU6050 test!");
    Serial.println("Initializing SIM808...");

    // Initialize MPU
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
        }
    }
    Serial.println("MPU6050 Found!");

    // Initialize SIM808 module
    if (initSIM808()) {
        Serial.println("SIM808 initialization successful");
    } else {
        Serial.println(
            "SIM808 initialization failed. Check connections and try again.");
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    Serial.print("Accelerometer range set to: ");
    switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
        Serial.println("+-2G");
        break;
    case MPU6050_RANGE_4_G:
        Serial.println("+-4G");
        break;
    case MPU6050_RANGE_8_G:
        Serial.println("+-8G");
        break;
    case MPU6050_RANGE_16_G:
        Serial.println("+-16G");
        break;
    }

    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    Serial.print("Gyro range set to: ");
    switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
        Serial.println("+- 250 deg/s");
        break;
    case MPU6050_RANGE_500_DEG:
        Serial.println("+- 500 deg/s");
        break;
    case MPU6050_RANGE_1000_DEG:
        Serial.println("+- 1000 deg/s");
        break;
    case MPU6050_RANGE_2000_DEG:
        Serial.println("+- 2000 deg/s");
        break;
    }

    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    Serial.print("Filter bandwidth set to: ");
    switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
        Serial.println("260 Hz");
        break;
    case MPU6050_BAND_184_HZ:
        Serial.println("184 Hz");
        break;
    case MPU6050_BAND_94_HZ:
        Serial.println("94 Hz");
        break;
    case MPU6050_BAND_44_HZ:
        Serial.println("44 Hz");
        break;
    case MPU6050_BAND_21_HZ:
        Serial.println("21 Hz");
        break;
    case MPU6050_BAND_10_HZ:
        Serial.println("10 Hz");
        break;
    case MPU6050_BAND_5_HZ:
        Serial.println("5 Hz");
        break;
    }

    // Serial.println("");
    delay(1000);
}

void loop() {
    sensors_event_t a, g, temp;   // Get sensor events with the readings with MPU
    mpu.getEvent(&a, &g, &temp);

    digitalWrite(TRIG_PIN, LOW);
    delay(2);
    digitalWrite(TRIG_PIN, HIGH);
    delay(10);
    digitalWrite(TRIG_PIN, LOW);

    duration = pulseIn(ECHO_PIN, HIGH);
    distance = (duration * .0343) / 2;

    /* Print out the values */
    Serial.print("Distance: ");
    Serial.println(distance);

    Serial.print("Acceleration: ");
    Serial.print(a.acceleration.x);
    Serial.print(", ");
    Serial.print(a.acceleration.y);
    Serial.print(", ");
    Serial.print(a.acceleration.z);
    Serial.println(" m/s^2");

    Serial.print("Rotation: ");
    Serial.print(g.gyro.x);
    Serial.print(", ");
    Serial.print(g.gyro.y);
    Serial.print(", ");
    Serial.print(g.gyro.z);
    Serial.println(" rad/s");

    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    Serial.println(" degC");

    int flameValue = digitalRead(FLAME_PIN);

    if (flameValue == LOW) {
        beepAndFlash(2000);

        digitalWrite(MOTOR_INPUT1_PIN, LOW);    // Turn off motor
        digitalWrite(MOTOR_INPUT2_PIN, HIGH);
        Serial.println("Flame detected! Motor turned off.");

        sendSMS(PHONE_NUMBER, MESSAGE); // Send SMS
        delay(30000);   // 30 seconds interval before sending another SMS
    } else if ((temp.temperature) > 30) {
        beepAndFlash(1000);
        Serial.println("Temperature High!");
    } else if (distance < 15) {
        beepAndFlash(300);
        Serial.println("Obstacle Ahead!");
    } else if (g.gyro.x > 0.5) {
        beepAndFlash(3000);

        digitalWrite(MOTOR_INPUT1_PIN, LOW);    // Turn off motor
        digitalWrite(MOTOR_INPUT2_PIN, HIGH);
        Serial.println("Car Flipped!! Motor turned off.");

        sendSMS(PHONE_NUMBER, MESSAGE); // Send SMS
        delay(30000);   // 30 seconds interval before sending another SMS
    }
    else {
        // No flame detected
        digitalWrite(MOTOR_INPUT1_PIN, HIGH);
        digitalWrite(MOTOR_INPUT2_PIN, LOW);
        Serial.println("No flame detected");
        Serial.println("");
    }

    analogWrite(MOTOR_ENABLE_PIN, 255); // Enable the motor

    delay(1000); // Add a delay to avoid continuous readings
}

bool initSIM808() {
    sim808Serial.println("AT");
    delay(1000);

    // Check if "OK" is received from SIM808
    if (sim808Serial.find("OK")) {
        return true;
    } else {
        return false;
    }
}

void sendSMS(const char *phoneNumber, const char *message) {
    // Set SMS text mode
    sim808Serial.println("AT+CMGF=1");
    delay(1000);

    // Set destination phone number
    sim808Serial.print("AT+CMGS=\"");
    sim808Serial.print(phoneNumber);
    sim808Serial.println("\"");
    delay(1000);

    // Send SMS content
    sim808Serial.print(message);
    delay(100);
    sim808Serial.write(26); // End the SMS with Ctrl+Z
    delay(1000);
}

void beepAndFlash(float miliseconds) {
    digitalWrite(BUZZER_PIN, HIGH); // Turn on the buzzer and LED
    digitalWrite(LED_BUILTIN, HIGH);

    delay(miliseconds); // Buzz and flash for 1 seconds

    digitalWrite(BUZZER_PIN, LOW);  // Turn off the buzzer and LED
    digitalWrite(LED_BUILTIN, LOW);
}
