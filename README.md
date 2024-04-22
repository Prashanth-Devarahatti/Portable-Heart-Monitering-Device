# ESP32 ECG Heart Rate and SpO2 Monitoring with ThingSpeak Integration

This project utilizes an ESP32 microcontroller along with a MAX30102 sensor to monitor heart rate and blood oxygen saturation (SpO2) levels. The collected data is then sent to ThingSpeak, an IoT platform, for storage and visualization.

## Dependencies:

- Arduino IDE
- Wire library (`Wire.h`)
- MAX30105 library (`MAX30105.h`)
- heartRate library (`heartRate.h`)
- spo2_algorithm library (`spo2_algorithm.h`)
- WiFi library (`WiFi.h`)
- ThingSpeak library (`ThingSpeak.h`)

Ensure that you have installed the required libraries in your Arduino IDE before proceeding.

## Hardware Setup:

1. Connect the MAX30102 and AD8232 sensors to the ESP32 microcontroller according to the wiring diagram.
2. Power up the ESP32 microcontroller.

## Software Setup:

1. Configure your WiFi network credentials by replacing `"your_wifi_ssid"` and `"your_wifi_password"` with your network's SSID and password, respectively.
2. Obtain your ThingSpeak channel number and write API key from the ThingSpeak platform and replace `"YOUR_CHANNEL_NUMBER"` and `"YOUR_WRITE_API_KEY"` with the corresponding values.
3. Upload the provided code to your ESP32 microcontroller using the Arduino IDE.

## Usage:

1. Once the ESP32 is powered up and connected to WiFi, it will start monitoring heart rate and SpO2 levels.
2. Place your index finger on the sensor with steady pressure.
3. The sensor will start reading heart rate and SpO2 levels.
4. The data will be sent to ThingSpeak and can be accessed through your ThingSpeak channel.

## Code Structure:

- **Setup Function (`setup()`)**: Initializes the necessary components, including the serial communication, MAX30102 sensor, WiFi connection, and ThingSpeak integration.

- **Main Loop (`loop()`)**: Continuously reads sensor data, computes heart rate and SpO2 levels, sends the data to ThingSpeak, and delays for 20 seconds before repeating the process.

## Notes:

- Ensure that you have correctly wired the MAX30102 sensor to the ESP32 microcontroller.
- Make sure that you have a stable internet connection for the ESP32 to connect to WiFi and send data to ThingSpeak.
- You may need to adjust sensor calibration and thresholds for optimal performance.
- Customize the delay period in the loop function according to your data transmission frequency requirements.

## Contributors:

- Prashanth Devarahatti,Naman Thakur



---
Feel free to customize the README further with additional sections or details as needed!
