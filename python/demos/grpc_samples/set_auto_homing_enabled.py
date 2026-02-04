import sys

import grpc
import kachaka_api_pb2
from kachaka_api_pb2_grpc import KachakaApiStub

# gRPC 的初始化
stub = KachakaApiStub(grpc.insecure_channel(sys.argv[1]))

# 準備請求
req = kachaka_api_pb2.SetAutoHomingEnabledRequest(enable=False)

# 執行 SetAutoHomingEnabled
response = stub.SetAutoHomingEnabled(req)
if not response.result.success:
    print("Sending SetAutoHomingEnabled failed")

# 顯示執行結果
print(response)
