import time
import paho.mqtt.client as mqtt
import json
import numpy as np

broker = "10.224.64.29"
port = 1883
topic = "mpu6050/data"

message_counter = 0
latency_measurements = []

def on_connect(client, userdata, flags, rc, *args, **kwargs):
    print(f"Connected with result code {rc}")
    client.subscribe(topic)

def on_message(client, userdata, msg):
    global message_counter
    global latency_measurements
    message_counter += 1
    message = msg.payload.decode()
    message = json.loads(message)
    latency = time.time() - float(message[1])
    latency_measurements.append(latency)
    print(f"Received message nr. {message_counter}: {message[0]} with latency {latency}")
    if(message[0] == 'Done'):
        print(f'Mean latency is: {np.mean(latency_measurements)}')
        client.disconnect()

# Create an MQTT client instance
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

client.on_connect = on_connect
client.on_message = on_message

client.connect(broker, port, 60)

# Start the MQTT loop
client.loop_forever()