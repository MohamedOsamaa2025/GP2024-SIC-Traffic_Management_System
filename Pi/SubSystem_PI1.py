import asyncio
from bleak import BleakScanner
import paho.mqtt.client as mqtt

# MQTT Broker details
mqtt_server = "7ce86da1d434455a898586cc107e7e46.s1.eu.hivemq.cloud"
mqtt_topic_road_1 = "road_1"
mqtt_user = "MohamedBakr"
mqtt_password = "IOTEng2025#"
mqtt_port = 8883

# MAC address to monitor
mac_address = "C0:49:EF:D1:57:62"

# Initialize the MQTT client
client = mqtt.Client()

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
    else:
        print(f"Failed to connect, return code {rc}")

def setup_mqtt():
    client.username_pw_set(mqtt_user, mqtt_password)
    client.tls_set()  # Use TLS for secure connection
    client.on_connect = on_connect
    client.connect(mqtt_server, mqtt_port, 60)
    client.loop_start()

async def get_rssi(mac_address):
    try:
        # Limiting scan to MAC address of interest
        devices = await BleakScanner.discover()
        for device in devices:
            if device.address == mac_address:
                return device.rssi
    except Exception as e:
        print(f"Error during RSSI retrieval: {e}")
    return None

async def monitor_rssi(mac_address):
    while True:
        rssi_value = await get_rssi(mac_address)
        if rssi_value is not None:
            print(f"RSSI for {mac_address}: {rssi_value}")
            # Publish the RSSI value to the MQTT topic
            client.publish(mqtt_topic_road_1, str(rssi_value))
            print(f"Published RSSI {rssi_value} to {mqtt_topic_road_1}")
        else:
            print(f"Device {mac_address} not found.")
        await asyncio.sleep(5)  # Wait for 5 seconds before next scan

async def main():
    setup_mqtt()  # Start MQTT setup and loop
    await monitor_rssi(mac_address)  # Start monitoring RSSI

try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("Exiting...")
finally:
    # Gracefully stop MQTT loop and disconnect
    client.loop_stop()
    client.disconnect()
