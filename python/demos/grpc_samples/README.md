# 使用 Python 執行 gRPC API 的範例

## 準備
```
git clone https://github.com/pf-robotics/kachaka-api.git # 本儲存庫
python3 -m venv venv
source venv/bin/activate
cd kachaka-api/python/demos
pip install -r requirements.txt
cd grpc_samples
python -m grpc_tools.protoc -I../../../protos --python_out=. --pyi_out=. --grpc_python_out=. ../../../protos/kachaka-api.proto
```

## 執行
```
python <範例程式碼>.py <Kachaka 的 IP 位址>:26400
```


### 地圖的匯入・匯出範例

取得地圖列表
```
% ./get_map_list.py XX.XX.XX.XX:26400
metadata {
  cursor: 42676578825899
}
map_list_entries {
  id: "XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX"
  name: "Map 1"
}
map_list_entries {
  id: "XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX"
  name: "Map 2"
}
```

匯出
```
./export_map_api_client.py -s XX.XX.XX.XX:26400 -m XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX -o my-map1.kmap
```

匯入
```
./import_map_api_client.py -s XX.XX.XX.XX:26400 -i my-map1.kmap
```
