# 智慧音箱連動範例
# 請參考 README_SMART_SPEAKER.md 進行設定。

import os
import sys

import grpc
import kachaka_api_pb2
import paho.mqtt.client as mqtt
from kachaka_api_pb2_grpc import KachakaApiStub

CHANNEL = "test"  # Beebotte 的頻道名稱
RESOURCE = "sample"  # Beebotte 的資源名稱
PEM = "./mqtt.beebotte.com.pem"  # Beebotte 的憑證檔案

# API 的初始化
stub = KachakaApiStub(grpc.insecure_channel(sys.argv[1]))

# 從環境變數 TOKEN 取得權杖
try:
    token = os.environ["TOKEN"]
except KeyError:
    print("TOKEN is not set.")
    sys.exit(1)

if not os.path.exists(PEM):
    print(f"{PEM} file not found.")
    sys.exit(1)


def on_connect(client, userdata, flag, rc):
    if rc == 0:
        print("Connection succeeded.")
        client.subscribe(f"{CHANNEL}/{RESOURCE}", 1)
    else:
        print("Connection failed. code=" + str(rc))


def on_message(client, data, msg):
    print("on_message: " + str(msg.payload))

    # 執行 move_shelf 指令
    req = kachaka_api_pb2.StartCommandRequest(
        command=kachaka_api_pb2.Command(
            move_shelf_command=kachaka_api_pb2.MoveShelfCommand(
                target_shelf_id="S01",  # 拿取 shelf ID S01
                destination_location_id="L01",  # 移動到 location ID L01
            )
        )
    )
    response = stub.StartCommand(req)
    if not response.result.success:
        print("Sending move shelf command failed")
        sys.exit(1)


# MQTT 客戶端的初始化
mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message

mqtt_client.username_pw_set(f"token:{token}")
mqtt_client.tls_set(PEM)
mqtt_client.connect("mqtt.beebotte.com", 8883, 60)
mqtt_client.loop_forever()
