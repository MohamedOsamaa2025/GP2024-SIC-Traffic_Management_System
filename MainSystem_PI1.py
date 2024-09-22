
import time
import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
from picamera2 import Picamera2
from ultralytics import YOLO  # Import YOLOv8
import cv2

# MQTT Broker details
mqtt_server = "7ce86da1d434455a898586cc107e7e46.s1.eu.hivemq.cloud"
mqtt_topic1 = "TrafficDensity1"
mqtt_topic2 = "AdaptiveLights1"
mqtt_topic3 = "em"  # New topic for controlling the loops
mqtt_user = "MohamedBakr"
mqtt_password = "IOTEng2025#"

# Setup GPIO pins for the LEDs
RED_LED = 14
YELLOW_LED = 15
GREEN_LED = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(RED_LED, GPIO.OUT)
GPIO.setup(YELLOW_LED, GPIO.OUT)
GPIO.setup(GREEN_LED, GPIO.OUT)

# Initialize the PiCamera2
camera = Picamera2()

# Configure the camera
camera_config = camera.create_still_configuration()
camera.configure(camera_config)

# Load YOLOv8 model (pre-trained on COCO dataset)
model = YOLO('yolov8n.pt')

# Global variables to control loops
active_loop = "main"
em_command_value = None  # To store the value of "r1" or "r2"

# Function to detect cars in the captured image using YOLOv8
def detect_cars(image_path):
    img = cv2.imread(image_path)
    results = model(img)
    num_cars = 0

    for result in results:
        for obj in result.boxes.data:
            class_id = int(obj[5])
            if class_id == 2:
                num_cars += 1
                x1, y1, x2, y2 = map(int, obj[:4])
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

    print(f"Number of cars detected: {num_cars}")
    cv2.imwrite("./traffic_image_with_cars.jpg", img)
    return num_cars

def get_traffic_density(num_cars):
    if num_cars < 3:
        return "low"
    elif 3 <= num_cars < 6:
        return "medium"
    else:
        return "high"

# Callback function to handle received MQTT messages
def on_message(client, userdata, msg):
    global active_loop, em_command_value

    if msg.topic == mqtt_topic2:
        light_control = msg.payload.decode()
        print(f"Light Control Command: {light_control}")
        if light_control == "red":
            GPIO.output(RED_LED, GPIO.HIGH)
            GPIO.output(YELLOW_LED, GPIO.LOW)
            GPIO.output(GREEN_LED, GPIO.LOW)
        elif light_control == "yellow":
            GPIO.output(RED_LED, GPIO.LOW)
            GPIO.output(YELLOW_LED, GPIO.HIGH)
            GPIO.output(GREEN_LED, GPIO.LOW)
        elif light_control == "green":
            GPIO.output(RED_LED, GPIO.LOW)
            GPIO.output(YELLOW_LED, GPIO.LOW)
            GPIO.output(GREEN_LED, GPIO.HIGH)
    
    elif msg.topic == mqtt_topic3:
        # Control switching between loops based on message
        em_command = msg.payload.decode()
        if em_command in ["r1", "r2"]:
            active_loop = "secondary"
            em_command_value = em_command  # Store the received value
            print(f"Switching to secondary loop: {em_command}.")
        elif em_command == "done":
            active_loop = "main"
            em_command_value = None  # Reset the command
            print("Returning to main loop.")

# Function to publish traffic density
def publish_traffic_density(client, num_cars):
    traffic_density = get_traffic_density(num_cars)
    print(f"Traffic Density: {traffic_density}")
    client.publish(mqtt_topic1, traffic_density)

# Function to capture an image
def capture_image():
    camera.start()
    time.sleep(2)
    image_path = "./traffic_image.jpg"
    camera.capture_file(image_path)
    camera.stop()
    print("Image captured")
    return image_path

# Setup MQTT client and connection
def setup_mqtt():
    client = mqtt.Client()
    client.tls_set()
    client.username_pw_set(mqtt_user, mqtt_password)
    client.on_message = on_message
    client.connect(mqtt_server, 8883)
    client.subscribe(mqtt_topic2)
    client.subscribe(mqtt_topic3)  # Subscribe to the control topic
    client.loop_start()
    return client

# Main loop
def main_loop(client):
    last_publish_time = time.time()
    while active_loop == "main":
        if time.time() - last_publish_time > 5:
            image_path = capture_image()
            num_cars = detect_cars(image_path)
            publish_traffic_density(client, num_cars)
            last_publish_time = time.time()
        time.sleep(1)

# Secondary loop (to run when 'r1' or 'r2' message is received)
def secondary_loop():
    while active_loop == "secondary":
        if em_command_value == "r1":
            print("Secondary loop running for r1...")
            # Perform tasks specific to "r1"
            GPIO.output(GREEN_LED, GPIO.HIGH)
            GPIO.output(RED_LED, GPIO.LOW)
            GPIO.output(YELLOW_LED, GPIO.LOW)
        elif em_command_value == "r2":
            print("Secondary loop running for r2...")
            # Perform tasks specific to "r2"
            GPIO.output(RED_LED, GPIO.HIGH)
            GPIO.output(GREEN_LED, GPIO.LOW)
            GPIO.output(YELLOW_LED, GPIO.LOW)
        time.sleep(1)

# Main program
def main():
    try:
        client = setup_mqtt()
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
