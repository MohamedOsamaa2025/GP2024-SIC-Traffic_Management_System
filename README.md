# GP2024-SIC-Traffic_Management_System

The Smart Traffic Management System is an integrated solution designed to optimize 
urban mobility through real-time data and intelligent automation. It features an Adaptive 
Traffic Light System that leverages computer vision and the YOLOv8 machine learning 
model to detect vehicle density and dynamically adjust traffic signals for optimal flow. To 
support emergency response, the system incorporates an Emergency Vehicle Response 
System, which uses Pushbullet notifications to detect emergency vehicle presence and 
reconfigure traffic lights to clear their path. Additionally, an Accident Detection System 
employs an IMU sensor for crash detection and RSSI-based localization, enabling the 
system to prioritize traffic flow around the incident and immediately send alerts via 
Pushbullet. All components communicate seamlessly using MQTT, are orchestrated 
through Node-RED, and monitored in real time via the Node-RED dashboard, ensuring 
centralized control and enhanced traffic safety.

![image](https://github.com/user-attachments/assets/512e5194-4414-4479-9a9f-a144538c82bd)
