import asyncio
from bleak import BleakScanner
import paho.mqtt.client as mqtt

# MQTT Broker details
mqtt_server = "7ce86da1d434455a898586cc107e7e46.s1.eu.hivemq.cloud"
mqtt_topic_road_1 = "road_1"
# mqtt_topic_road_2 = "road_2"
mqtt_user = "MohamedBakr"
mqtt_password = "IOTEng2025#"
mqtt_port = 8883

# Initialize the MQTT client
client = mqtt.Client()

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
    else:
        print(f"Failed to connect, return code {rc}")

client.username_pw_set(mqtt_user, mqtt_password)
client.tls_set()  # Use TLS for secure connection
client.on_connect = on_connect
client.connect(mqtt_server, mqtt_port, 60)

# Start the MQTT loop in a non-blocking manner
client.loop_start()

async def get_rssi(mac_address):
    try:
        devices = await BleakScanner.discover()
        for device in devices:
            if device.address == mac_address:
                return device.rssi
    except Exception as e:
        print(f"Error during RSSI retrieval: {e}")
    return None

async def monitor_rssi(mac_addresses):
    while True:
        for i, mac_address in enumerate(mac_addresses):
            rssi_value = await get_rssi(mac_address)
            if rssi_value is not None:
                print(f"RSSI for {mac_address}: {rssi_value}")

                # Send RSSI to the appropriate MQTT topic
                if i == 0:
                    client.publish(mqtt_topic_road_1, str(rssi_value))
                    print(f"Published RSSI {rssi_value} to {mqtt_topic_road_1}")
                elif i == 1:
                    client.publish(mqtt_topic_road_2, str(rssi_value))
                    print(f"Published RSSI {rssi_value} to {mqtt_topic_road_2}")
            else:
                print(f"Device {mac_address} not found.")
        
        # Wait for a short interval before scanning again
        await asyncio.sleep(5)

# Replace these with the actual MAC addresses of your ESP devices
esp_mac_addresses = ["C0:49:EF:D1:57:62", "94:E6:86:38:61:AE"]

# Run the monitoring function
asyncio.run(monitor_rssi(esp_mac_addresses))

# Stop the MQTT loop when done (in case of a clean exit)
client.loop_stop()
client.disconnect()
