#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <MPU6050.h>
#include <Servo.h>
#include <WiFi.h>
#include <WiFiClient.h>

// WiFi连接信息
const char* ssid = "mainly";
const char* password = "";

// 服务器信息
const char* server = "data.awaland.xyz";
const int port = 443; // 使用HTTPS，默认端口443

// 创建传感器和伺服电机对象
Adafruit_BMP085 bmp;
MPU6050 mpu;
Servo servoX;
Servo servoY;

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

    // 初始化WiFi连接
    connectWiFi();

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
    // 读取气压和温度数据
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

    // 读取姿态数据
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // 转换为姿态角度（简单示例，实际应用需要更多处理）
    float pitch = atan2(ay, az) * 180 / PI;
    float roll = atan2(ax, az) * 180 / PI;

    // 输出姿态数据
    Serial.print("Pitch = ");
    Serial.print(pitch);
    Serial.println(" degrees");
    Serial.print("Roll = ");
    Serial.print(roll);
    Serial.println(" degrees");

    // 使用PID控制调整伺服电机角度
    computePID(servoX, pitch, pidSetpointX, pidOutputX, pidInputX, lastErrorX, outputX);
    computePID(servoY, roll, pidSetpointY, pidOutputY, pidInputY, lastErrorY, outputY);

    // 发送数据到服务器
    sendDataToServer(pitch, roll);

    delay(500);
}

// 连接WiFi函数
void connectWiFi() {
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
}

// PID计算函数
void computePID(Servo &servo, double input, double setpoint, double &output, double &pidInput, double &lastError, double &outputValue) {
    unsigned long currentTime = millis();
    elapsedTime = (double)(currentTime - previousTime) / 1000;

    error = setpoint - input;
    pidInput += error * elapsedTime;

    output = Kp * error + Ki * pidInput + Kd * (error - lastError) / elapsedTime;

    lastError = error;
    
    int newAngle = servo.read() + output;
    servo.write(constrain(newAngle, 0, 180));

    previousTime = currentTime;
}

// 发送数据到服务器
void sendDataToServer(float pitch, float roll) {
    WiFiClient client;

    if (!client.connect(server, port)) {
        Serial.println("Connection failed");
        return;
    }

    String url = "/"; // 在这里添加你的具体路径

    String postData = "pitch=" + String(pitch) + "&roll=" + String(roll);

    client.print(String("POST ") + url + " HTTP/1.1\r\n" +
                    "Host: " + server + "\r\n" +
                    "Content-Length: " + postData.length() + "\r\n" +
                    "Content-Type: application/x-www-form-urlencoded\r\n" +
                    "\r\n" + postData);

    Serial.println("Data sent to server");

    client.stop();
}
