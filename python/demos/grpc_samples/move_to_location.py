import sys

import grpc
import kachaka_api_pb2
from kachaka_api_pb2_grpc import KachakaApiStub

# gRPC 的初始化
stub = KachakaApiStub(grpc.insecure_channel(sys.argv[1]))

# 取得目前的 cursor
req = kachaka_api_pb2.GetRequest()
resp = stub.GetLastCommandResult(req)
last_cursor = resp.metadata.cursor

# 準備 move_to_location 指令
req = kachaka_api_pb2.StartCommandRequest(
    command=kachaka_api_pb2.Command(
        move_to_location_command=kachaka_api_pb2.MoveToLocationCommand(
            target_location_id="L01"
        )
    )
)

# 執行指令
resp = stub.StartCommand(req)
if not resp.result.success:
    print("Sending MoveToLocation command failed: " + resp.result.error_code)
    sys.exit(1)

print("MoveToLocation command sent")

# 等待指令執行完成
req = kachaka_api_pb2.GetRequest()
while True:
    resp = stub.GetLastCommandResult(req)
    if last_cursor != resp.metadata.cursor:
        break
    req.metadata.cursor = resp.metadata.cursor
    if not resp.result.success:
        print("MoveToLocation command failed :", resp.result.error_code)
        sys.exit(1)

print("MoveToLocation command completed")
