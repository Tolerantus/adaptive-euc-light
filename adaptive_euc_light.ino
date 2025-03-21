// #include "GyverPID.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <ESP32Servo.h>
// #include <Servo.h>

#include <GyverHub.h>
#include <PairsFile.h>
#include <WiFi.h>



const char* ssid = "euc_adaptive_light";
const char* password = "12345678";
const float ratio = 60 / 18;

#define ISR 16
#define SERVO 5


Servo servo;

volatile bool mpuFlag = false;  // флаг прерывания готовности
uint8_t fifoBuffer[45];         // буфер

MPU6050 mpu;
PairsFile data(&LittleFS, "/data.dat", 3000);
GyverHub hub("MyDevices", "Adaptive EUC light", "");  // имя сети, имя устройства, иконка


const bool debug = true;

struct Angles {
  float x;
  float y;
  float z;
};

long correction = 0;
int target = 0;
void build(gh::Builder& b) {

  {
    gh::Row r(b);
    // наклон
    b.Slider_("targetAngle", &data).label("Target angle").range(-20, 20, 1);
  }
}

void setup() {
  if (debug) {
    Serial.begin(9600);
  }
  servo.attach(SERVO, 600, 2400);  // 600 и 2400 - длины импульсов, при которых (esp32)


  Wire.begin();

  // put your setup code here, to run once:
  prepareMpu();
  target = data["targetAngle"];
  // target = 0;


  provideAPWifi();

  hub.onBuild(build);  // подключаем билдер
  hub.begin();         // запускаем систему
  data.begin();
}

void provideAPWifi() {
  Serial.println("Start AP");
  if (!WiFi.softAP(ssid, password)) {
    Serial.print("Soft AP creation failed.");
    while (1)
      ;
  }
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
}

void prepareMpu() {
  Wire.begin();
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  attachInterrupt(ISR, dmpReady, RISING);
  // состояние соединения
  Serial.println(mpu.testConnection() ? "MPU6050 OK" : "MPU6050 FAIL");
}

// прерывание готовности. Поднимаем флаг
void dmpReady() {
  mpuFlag = true;
}

void loop() {

  hub.tick();  // тикаем тут
  data.tick();
  // по флагу прерывания и готовности DMP
  static uint32_t lastMeasure;
  if (lastMeasure - millis() > 500 && mpuFlag && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    lastMeasure = millis();
    Angles angles = getAngles();
    target = data["targetAngle"];
    float angle = angles.y;
    // Serial.print(angle);
    // Serial.print(" ");
    if (angle > HALF_PI) {
      correction = (target - degrees(PI - angle)) * ratio + 90;
      // Serial.print("PI - angle=");
      // Serial.println(PI - angle);
    } else if (angle < -HALF_PI) {
      correction = (target - degrees(-PI - angle)) * ratio + 90;
      // Serial.print("PI + angle=");
      // Serial.println(-PI - angle);
    } else {
      correction = (target - degrees(angle)) * ratio + 90;
      // Serial.print("angle=");
      // Serial.println(angle);
    }
    // int target = 0;
    // Serial.print(angles.x);
    // Serial.print(" ");
    // Serial.print(angles.y);
    // Serial.print(" ");
    // Serial.println(correction);
    servo.write(correction);  // и отправляем на серво
  }
}


Angles getAngles() {
  Quaternion q;
  VectorFloat gravity;
  float ypr[3];
  // расчёты
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  return (Angles){ ypr[2], ypr[1], ypr[0] };
}
