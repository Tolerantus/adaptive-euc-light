// #include "GyverPID.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <ESP32Servo.h>

#include <GyverHub.h>
#include <PairsFile.h>
#include <WiFi.h>
// #include <ArduinoOTA.h>



const char* ssid = "euc_adaptive_light";
const char* password = "12345678";

#define ISR 16
#define SERVO 5


Servo servo;

volatile bool mpuFlag = false;  // флаг прерывания готовности
uint8_t fifoBuffer[45];         // буфер

MPU6050 mpu;
PairsFile data(&LittleFS, "/data.dat", 3000);
GyverHub hub("MyDevices", "Adaptive EUC light", "");  // имя сети, имя устройства, иконка


const bool debug = false;
Quaternion q;         // [w, x, y, z]         Quaternion container
VectorInt16 aa;       // [x, y, z]            Accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            Gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            Gravity vector

struct Angles {
  float x;
  float y;
  float z;
};

long correction = 0;
int target = 0;
float ratio = 80 / 16;

void build(gh::Builder& b) {
  {
    gh::Row r(b);
    // наклон
    b.Switch_("enabled", &data).label("Enabled");
  }

  {
    gh::Row r(b);
    // наклон
    b.Slider_("targetAngle", &data).label("Target angle").range(-20, 20, 1);
  }
  {
    gh::Row r(b);
    // наклон
    b.Slider_("ratio", &data).label("Ratio").range(3, 7, 0.1);
  }
}

VectorInt16 g;
int32_t gLength;
VectorInt16 gMeasurements[32];
int measurements = 32;
int calibrationAttempt = 0;
boolean calibrated = false;

void setup() {
  if (debug) {
    Serial.begin(9600);
  }
  servo.attach(SERVO, 600, 2400);  // 600 и 2400 - длины импульсов, при которых (esp32)


  Wire.begin();

  // put your setup code here, to run once:
  prepareMpu();
  // target = data["targetAngle"];
  // ratio = data["ratio"];


  provideAPWifi();

  hub.onBuild(build);  // подключаем билдер
  hub.begin();         // запускаем систему
  data.begin();

  mpu.setXAccelOffset(-2911);
  mpu.setYAccelOffset(-993);
  mpu.setZAccelOffset(524);
  mpu.setXGyroOffset(119);
  mpu.setYGyroOffset(-79);
  mpu.setZGyroOffset(16);
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

// void prepareOTA() {
//   ArduinoOTA
//     .onStart([]() {
//       String type;
//       if (ArduinoOTA.getCommand() == U_FLASH)
//         type = "sketch";
//       else  // U_SPIFFS
//         type = "filesystem";

//       // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
//       Serial.println("Start updating " + type);
//     })
//     .onEnd([]() {
//       Serial.println("\nEnd");
//     })
//     .onProgress([](unsigned int progress, unsigned int total) {
//       Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
//     })
//     .onError([](ota_error_t error) {
//       Serial.printf("Error[%u]: ", error);
//       if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
//       else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
//       else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
//       else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
//       else if (error == OTA_END_ERROR) Serial.println("End Failed");
//     });

//   ArduinoOTA.begin();

//   Serial.println("Ready");
//   Serial.print("IP address: ");
//   Serial.println(WiFi.localIP());
// }

// прерывание готовности. Поднимаем флаг
void dmpReady() {
  mpuFlag = true;
}

void loop() {

  // ArduinoOTA.handle();
  hub.tick();  // тикаем тут
  data.tick();
  // по флагу прерывания и готовности DMP
  // static uint32_t lastMeasure;
  if (mpuFlag && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    // lastMeasure = millis();
    int target = data["targetAngle"];
    // int target = 0;

    float ratio = data["ratio"];
    // float ratio = 5;
    float angle = getAngles().y;
    int correction = 90;
    Serial.print(angle);
    Serial.print("\t");
    if (data["enabled"] == 1) {
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
    }
    // int target = 0;
    // Serial.print(angles.x);
    // Serial.print(" ");
    // Serial.print(angles.y);
    // Serial.print(" ");
    Serial.println(correction);
    servo.write(correction);  // и отправляем на серво
  }
}

// int getAngle() {
//   mpu_loop();
//   // Serial.print("aa\t");
//   // Serial.print(aa.x);
//   // Serial.print("\t");
//   // Serial.println(aa.y);
//   int32_t aLength = sqrt(aa.x * aa.x + aa.y * aa.y + aa.z * aa.z);

//   Serial.print("\taLength=\t");
//   Serial.print(aLength);
//   Serial.print("\tgLength=\t");
//   Serial.print(gLength);


//   double cos = gLength * 1.0 / aLength;

//   Serial.print("\tcos=\t");
//   Serial.print(cos);
//   Serial.print("\tacos=\t");
//   Serial.print(acos(cos));
//   return degrees(acos(cos));
// }

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
