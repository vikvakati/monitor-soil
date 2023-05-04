# ESP32 Sensor Network

This project is an example of how to create a wireless sensor network using ESP32 microcontrollers. The network consists of a transmitter and a receiver that communicate using the ESP-NOW protocol.

The transmitter is equipped with several sensors to measure environmental parameters such as temperature, humidity, moisture, pH, and nutrient levels. It sends this data wirelessly to the receiver which collects and logs the information.

A web-based dashboard is also included in this project. The dashboard allows users to view the sensor data in real-time and provides historical data trends for each environmental parameter.

## Hardware Components Required

- 2 ESP32 microcontrollers
- 1 LM35 temperature sensor
- 1 DHT11 humidity and temperature sensor
- 1 Gravity Analog pH Sensor
- 1 Gravity Analog Moisture Sensor
- 1 Gravity Analog NPK Sensor
- 1 USB cable
- Jumper wires

## Software Components Required

- Arduino IDE
- ph_grav.h library
- ESP-NOW and ESP-WIFI libraries

## Usage Instructions

1. Connect the sensors to the transmitter board according to the pin configurations defined in the code.
2. Flash the transmitter code to the ESP32 board.
3. Connect the ESP32 board to your computer using a USB cable and open the Arduino IDE.
4. Open the Serial Monitor and ensure that the board is connected and the sensors are working properly.
5. Connect the receiver board to your computer using a USB cable and flash the receiver code to the board.
6. Open the Serial Monitor and ensure that the receiver is connected and ready to receive data.
7. Power on the transmitter board and check that the data is being transmitted successfully to the receiver board.
