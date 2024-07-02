#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <MPU6050.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <RH_RF95.h>

// 创建LoRa对象，使用SoftwareSerial通信
SoftwareSerial loraSerial(4, 5); // RX, TX for LoRa module
RH_RF95 rf95;

// 创建传感器和伺服电机对象
Adafruit_BMP085 bmp;
MPU6050 mpu;
Servo servoX;
Servo servoY;
TinyGPSPlus gps;

// 伺服电机引脚
const int servoXPin = 9;
const int servoYPin = 10;

// PID控制参数
double Kp = 1.0; // 比例系数
double Ki = 0.0; // 积分系数
double Kd = 0.0; // 微分系数

double pidInputX, pidOutputX, pidSetpointX; // X轴PID控制变量
double pidInputY, pidOutputY, pidSetpointY; // Y轴PID控制变量

unsigned long previousTime = 0;
double elapsedTime;
double errorX, lastErrorX, outputX;
double errorY, lastErrorY, outputY;

void setup() {
    Serial.begin(9600);
    loraSerial.begin(9600); // LoRa模块波特率

    // 初始化LoRa模块
    if (!rf95.init()) {
        Serial.println("LoRa init failed");
        while (1);
    }
    rf95.setFrequency(915.0); // 根据实际频段设置
    rf95.setTxPower(23, false); // 根据实际模块参数设置

    // 初始化气压传感器
    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP085 sensor, check wiring!"); 
        while (1) {}
    }

    // 初始化MPU6050
    Wire.begin();
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while (1) {}
    }

    // 初始化伺服电机
    servoX.attach(servoXPin);
    servoY.attach(servoYPin);

    // 设置PID初始参数和目标
    pidSetpointX = 0.0; // 设置X轴目标角度
    pidSetpointY = 0.0; // 设置Y轴目标角度

    // 初始化时间
    previousTime = millis();
}

void loop() {
    // 更新GPS数据
    while (loraSerial.available() > 0) {
        if (gps.encode(loraSerial.read())) {
            if (gps.location.isValid()) {
                Serial.print("Latitude: ");
                Serial.print(gps.location.lat(), 6);
                Serial.print(" Longitude: ");
                Serial.println(gps.location.lng(), 6);
            }
        }
    }

    // 获取气压和姿态数据
    Serial.print("Temperature = "); 
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
    Serial.print("Pressure = "); 
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");
    Serial.print("Real altitude = "); 
    Serial.print(bmp.readAltitude(101500)); 
    Serial.println(" meters");
    Serial.println();
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float pitch = atan2(ay, az) * 180 / PI;
    float roll = atan2(ax, az) * 180 / PI;
    Serial.print("Pitch = ");
    Serial.print(pitch);
    Serial.println(" degrees");
    Serial.print("Roll = ");
    Serial.print(roll);
    Serial.println(" degrees");
    computePID(servoX, pitch, pidSetpointX, pidOutputX, pidInputX, lastErrorX, outputX);
    computePID(servoY, roll, pidSetpointY, pidOutputY, pidInputY, lastErrorY, outputY);

    // 发送数据到地面站
    sendDataToGroundStation(pitch, roll);

    delay(500);
}

// PID计算函数
void computePID(Servo &servo, double input, double setpoint, double &output, double &pidInput, double &lastError, double &outputValue) {
    unsigned long currentTime = millis();
    elapsedTime = (double)(currentTime - previousTime) / 1000;

    double error = setpoint - input;
    pidInput += error * elapsedTime;

    output = Kp * error + Ki * pidInput + Kd * (error - lastError) / elapsedTime;

    lastError = error;
    
    int newAngle = servo.read() + output;
    servo.write(constrain(newAngle, 0, 180));

    previousTime = currentTime;
}

// 发送数据到地面站，包括GPS数据
void sendDataToGroundStation(float pitch, float roll) {
    String data = "pitch=" + String(pitch) + "&roll=" + String(roll);
    if (gps.location.isValid()) {
        data += "&latitude=" + String(gps.location.lat(), 6) +
                "&longitude=" + String(gps.location.lng(), 6);
    }

    // 将数据转换为字符数组
    char dataToSend[data.length() + 1];
    data.toCharArray(dataToSend, data.length() + 1);

    // 发送数据
    rf95.send((uint8_t *)dataToSend, sizeof(dataToSend));
    rf95.waitPacketSent();
}

