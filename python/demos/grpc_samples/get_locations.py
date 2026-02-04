import sys

import grpc
import kachaka_api_pb2
from kachaka_api_pb2_grpc import KachakaApiStub

# gRPC 的初始化
stub = KachakaApiStub(grpc.insecure_channel(sys.argv[1]))

# 執行 GetLocations
response = stub.GetLocations(kachaka_api_pb2.GetRequest())

# 顯示執行結果
print(response)
