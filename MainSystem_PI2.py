import time
import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
from picamera2 import Picamera2
from ultralytics import YOLO
import cv2

# MQTT Broker details
mqtt_server = "7ce86da1d434455a898586cc107e7e46.s1.eu.hivemq.cloud"
mqtt_topic1 = "TrafficDensity2"
mqtt_topic2 = "AdaptiveLights2"
mqtt_topic3 = "em"  # Control loop topic
mqtt_user = "MohamedBakr"
mqtt_password = "IOTEng2025#"

# Setup GPIO pins for LEDs
RED_LED = 14
YELLOW_LED = 15
GREEN_LED = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(RED_LED, GPIO.OUT)
GPIO.setup(YELLOW_LED, GPIO.OUT)
GPIO.setup(GREEN_LED, GPIO.OUT)

# Initialize PiCamera2
camera = Picamera2()
camera.configure(camera.create_still_configuration())

# Load YOLOv8 model (pre-trained on COCO dataset)
model = YOLO('yolov8n.pt')

# Global variable to control loop execution
active_loop = "main"

def detect_cars(image_path):
    img = cv2.imread(image_path)
    results = model(img)
    num_cars = sum(1 for result in results for obj in result.boxes.data if int(obj[5]) == 2)

    if num_cars > 0:
        for result in results:
            for obj in result.boxes.data:
                if int(obj[5]) == 2:
                    x1, y1, x2, y2 = map(int, obj[:4])
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        cv2.imwrite("./traffic_image_with_cars.jpg", img)

    print(f"Number of cars detected: {num_cars}")
    return num_cars

def get_traffic_density(num_cars):
    return "low" if num_cars < 3 else "medium" if num_cars < 6 else "high"

def on_message(client, userdata, msg):
    global active_loop
    if msg.topic == mqtt_topic2:
        handle_lights(msg.payload.decode())
    elif msg.topic == mqtt_topic3:
        handle_loop_switch(msg.payload.decode())

def handle_lights(command):
    print(f"Light Control Command: {command}")
    GPIO.output(RED_LED, command == "red")
    GPIO.output(YELLOW_LED, command == "yellow")
    GPIO.output(GREEN_LED, command == "green")

def handle_loop_switch(command):
    global active_loop
    if command in ["r1", "r2"]:
        active_loop = "secondary"
        print(f"Switching to secondary loop: {command}")
    elif command == "done":
        active_loop = "main"
        print("Returning to main loop")

def publish_traffic_density(client, num_cars):
    traffic_density = get_traffic_density(num_cars)
    print(f"Traffic Density: {traffic_density}")
    client.publish(mqtt_topic1, traffic_density)

def capture_image():
    camera.start()
    time.sleep(1)  # Reduced wait time
    image_path = "./traffic_image.jpg"
    camera.capture_file(image_path)
    camera.stop()
    print("Image captured")
    return image_path

def setup_mqtt():
    client = mqtt.Client()
    client.tls_set()
    client.username_pw_set(mqtt_user, mqtt_password)
    client.on_message = on_message
    client.connect(mqtt_server, 8883)
    client.subscribe([(mqtt_topic2, 0), (mqtt_topic3, 0)])  # Subscribing multiple topics at once
    client.loop_start()
    return client

def main_loop(client):
    last_publish_time = time.time()
    while active_loop == "main":
        if time.time() - last_publish_time >= 5:  # Publish every 5 seconds
            num_cars = detect_cars(capture_image())
            publish_traffic_density(client, num_cars)
            last_publish_time = time.time()
        time.sleep(0.5)  # Reduced sleep time

def secondary_loop():
    while active_loop == "secondary":
        print("Secondary loop running...")
        GPIO.output(GREEN_LED, GPIO.HIGH)  # Green light active during secondary loop
        time.sleep(0.5)

def main():
    client = setup_mqtt()
    try:
        while True:
            if active_loop == "main":
                main_loop(client)
            elif active_loop == "secondary":
                secondary_loop()
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        GPIO.cleanup()
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    main()
