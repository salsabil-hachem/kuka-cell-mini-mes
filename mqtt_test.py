# mqtt_test.py - sends test messages of the KUKA cell to Mosquitto (MQTT 3.1.1, QoS 0)
import json, socket, time

HOST, PORT = "localhost", 1883
TOPIC = "autoparts/sousse/cellA"

def enc_len(n):
    out = bytearray()
    while True:
        b, n = n % 128, n // 128
        out.append(b | 0x80 if n else b)
        if not n:
            return bytes(out)

def enc_str(s):
    b = s.encode()
    return len(b).to_bytes(2, "big") + b

# 1) CONNECT
sock = socket.create_connection((HOST, PORT), timeout=2)
body = enc_str("MQTT") + bytes([4, 0x02, 0, 0]) + enc_str("kuka-test")
sock.sendall(b"\x10" + enc_len(len(body)) + body)
ack = sock.recv(4)
print("Connected" if ack[3] == 0 else "Refused")

# 2) PUBLISH
def publish(kind, data):
    body = enc_str(f"{TOPIC}/{kind}") + json.dumps(data).encode()
    sock.sendall(b"\x30" + enc_len(len(body)) + body)
    print("sent", kind, data)

publish("order",  {"order": "OF-TEST", "target": 2})
publish("status", {"state": "RUNNING", "text": "PICKING raw part"})
time.sleep(1)
publish("part",   {"order": "OF-TEST", "serial": "SN-0001", "diameter_mm": 20.012,
                   "result": "OK", "cycle_time_s": 32.5, "downtime_s": 0})
publish("stop",   {"order": "OF-TEST", "serial": "SN-0002", "station": "vision",
                   "reason": "Camera timeout", "duration_s": 24})
publish("status", {"state": "IDLE", "text": "ORDER FINISHED"})

# 3) DISCONNECT
time.sleep(0.5)
sock.sendall(b"\xe0\x00")
sock.close()
print("Disconnected")