# Mayoiuta
Cry of the Lost star | satellite flight control system

## 硬件连接
首先，将ESP8266模块连接到Arduino UNO：

- ESP8266 VCC 连接到 Arduino 3.3V
- ESP8266 GND 连接到 Arduino GND
- ESP8266 TX 连接到 Arduino RX (Pin 0)
- ESP8266 RX 通过电压分压器（或电阻分压器）连接到 Arduino TX (Pin 1)
- ESP8266 CH_PD 连接到 Arduino 3.3V
- 将舵机的信号线连接到Arduino UNO的数字引脚，比如Pin 9或Pin 10。
- 将舵机的电源（通常是+5V）连接到Arduino UNO的5V引脚。
- 将舵机的地线连接到Arduino UNO的GND引脚。
- GPS模块通常使用串口通信，因此可以直接连接到Arduino UNO的硬件串口（RX和TX引脚）。
- 如果使用的是软件串口，确保将GPS模块的TX连接到Arduino的软件串口RX引脚，而GPS模块的RX连接到Arduino的软件串口TX引脚。
- 烧录 Mayoiuta

Enjoy your fly!

**特殊电源要求**：ESP8266需要3.3V电源，不能直接使用5V，否则会损坏模块。Arduino UNO的3.3V供电有限，建议使用独立的3.3V稳压电源模块。

