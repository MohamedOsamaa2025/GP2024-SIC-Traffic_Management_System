# GradProj_SIC

1. Adaptive Traffic Light System: Utilizes computer vision powered by the YOLOv8 machine learning model to detect vehicle density and adjust traffic lights accordingly. This helps manage traffic flow efficiently based on real-time conditions.
2. Emergency Vehicle Response System: Sends notifications via the Pushbullet app to prioritize emergency vehicles. Upon receiving a message, the system adjusts the traffic lights to ensure a clear route for the vehicle.
3. Accident Detection System: Uses an IMU sensor to detect accidents and determines the location based on RSSI (signal strength). The system then prioritizes the adaptive traffic lights for the accident area and sends an emergency alert to the Pushbullet app.

All systems communicate using MQTT, are managed through Node-RED, and visualized on the Node-RED dashboard.
