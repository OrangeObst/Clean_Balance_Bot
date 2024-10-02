from smbus2 import SMBus
from mpu6050 import MyMPU6050
from codetiming import Timer
import paho.mqtt.client as mqtt
import json
import time
import sys

bus = SMBus(1)
mpu = MyMPU6050(bus)

mpu.set_dlpf_cfg(2)
mpu.set_smplrt_div(4)


def on_connect(client, userdata, flags, rc, *args, **kwargs):
    print(f"Connected with result code {rc}")
    client.subscribe(topic)

def on_publish(client, userdata, mid, *args, **kwargs):
    print(f"Message {mid} published.")

@Timer(name="on_message", text="on_message: {milliseconds:.6f}ms")
def on_message(client, userdata, msg):
    global start_time
    global message_counter
    message_counter += 1
    latency = time.time() - start_time
    print(f"Received message nr. {message_counter}: {msg.payload.decode()}")
    print(f"Round-trip latency: {latency:.6f} seconds")

# MQTT setup
broker = "10.224.64.29"
port = 1883
topic = "mpu6050/data"

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_publish = on_publish
client.on_message = on_message

client.connect(broker, port, 60)
client.loop_start()

message_counter = 0
t = Timer("example", text="Time spent: {:.2f}")
try:
    timer = time.time()
    while((time.time() - timer) < 5):
        start_time = time.time()
        
        # Read data from MPU6050
        data = mpu.MPU_ReadData()
        
        # Convert data to JSON
        payload = json.dumps(data)

        # Measure data size
        data_size = sys.getsizeof(payload)
        
        # Publish data to MQTT broker
        result = client.publish(topic, payload)
        
        # Print data size
        # print(f"Data sent: {data_size} bytes")
        
        # Wait for a second before sending the next data
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Exiting...")
finally:
    client.loop_stop()
    client.disconnect()

time.sleep(1)
time.sleep(1)
time.sleep(1)