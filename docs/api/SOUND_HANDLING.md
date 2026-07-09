# 音效的管理與播放

自 **3.17** 版起，Kachaka API 新增了一組音效（Sound）API，可用來管理儲存在機器人上的音效檔案，並讓機器人播放這些音效。與朗讀文字的 `speak`（語音合成）不同，本組 API 是直接播放預先上傳的音效檔案。

## 基本 API

音效 API 共有以下 5 個。

|        API        |                 角色                  |
|        ---        |                 ---                   |
| `GetSoundList`    | 取得機器人上已註冊的音效清單          |
| `AddSound`        | 上傳並註冊一個新的音效檔案            |
| `DeleteSound`     | 刪除指定的音效                        |
| `PlaySound`       | 播放指定的音效（可選擇是否循環播放）  |
| `StopSound`       | 停止目前正在播放的音效                |

> 註：在目前版本中，音效 API 尚未整合進高階的 `KachakaApiClient`，需透過原始的 gRPC stub（`KachakaApiStub`）呼叫。用法請參考下方[使用範例](#使用範例)。

## API 詳細說明

### GetSoundList
* 取得機器人上目前已註冊的所有音效。
* 回應中的每個 `Sound` 都包含 `id`（音效 ID）與 `name`（音效名稱）。播放或刪除音效時需使用此 `id`。

```protobuf
message Sound {
  string id = 1;    // 音效 ID（播放、刪除時使用）
  string name = 2;  // 音效名稱
}

message GetSoundListResponse {
  Metadata metadata = 1;
  repeated Sound sounds = 2;  // 已註冊的音效清單
}
```

### AddSound
* 上傳一個音效檔案並註冊到機器人上。
* `name` 為音效的名稱，`data` 為音效檔案的二進位內容（以 binary 模式讀取檔案後直接放入）。
* 註冊成功後，回應的 `sound_id` 即為此音效的 ID，可用於後續播放或刪除。

```protobuf
message AddSoundRequest {
  string name = 1;  // 音效名稱
  bytes data = 2;   // 音效檔案的二進位資料
}

message AddSoundResponse {
  Result result = 1;    // 執行結果（success / error_code）
  string sound_id = 2;  // 新註冊音效的 ID
}
```

### DeleteSound
* 刪除指定 ID 的音效。
* `sound_id` 請帶入 `GetSoundList` 或 `AddSound` 取得的音效 ID。

```protobuf
message DeleteSoundRequest {
  string sound_id = 1;  // 要刪除的音效 ID
}

message DeleteSoundResponse {
  Result result = 1;
}
```

### PlaySound
* 播放指定 ID 的音效。
* 將 `loop` 設為 `true` 時會循環播放，直到呼叫 `StopSound` 為止；設為 `false`（預設）則播放一次後結束。

```protobuf
message PlaySoundRequest {
  string sound_id = 1;  // 要播放的音效 ID
  bool loop = 2;        // 是否循環播放
}

message PlaySoundResponse {
  Result result = 1;
}
```

### StopSound
* 停止目前正在播放的音效。
* 此請求不需要任何參數。

```protobuf
message StopSoundRequest {
}

message StopSoundResponse {
  Result result = 1;
}
```

> 關於 `Result` 訊息（`success`、`error_code`）的說明，請參閱[錯誤狀態的偵測與處理](./ERROR_HANDLING.md)。

## 使用範例

由於音效 API 尚未封裝進高階 client，以下以原始 gRPC stub 示範完整流程：列出音效 → 上傳 → 播放 → 停止 → 刪除。

```python
import sys

import grpc
import kachaka_api_pb2
from kachaka_api_pb2_grpc import KachakaApiStub

# gRPC 的初始化（sys.argv[1] 為機器人位址，例如 192.168.1.100:26400）
stub = KachakaApiStub(grpc.insecure_channel(sys.argv[1]))

# 1. 取得目前已註冊的音效清單
response = stub.GetSoundList(kachaka_api_pb2.GetRequest())
for sound in response.sounds:
    print(f"id={sound.id}, name={sound.name}")

# 2. 上傳一個新的音效檔案
with open("chime.wav", "rb") as f:
    data = f.read()
add_response = stub.AddSound(kachaka_api_pb2.AddSoundRequest(name="chime", data=data))
if not add_response.result.success:
    print("AddSound failed")
    sys.exit(1)
sound_id = add_response.sound_id
print(f"已註冊音效，sound_id={sound_id}")

# 3. 播放剛上傳的音效（單次播放）
stub.PlaySound(kachaka_api_pb2.PlaySoundRequest(sound_id=sound_id, loop=False))

# 4. 停止播放（循環播放時特別有用）
stub.StopSound(kachaka_api_pb2.StopSoundRequest())

# 5. 刪除音效
stub.DeleteSound(kachaka_api_pb2.DeleteSoundRequest(sound_id=sound_id))
```

> 上述 gRPC 範例的執行環境設定（`import` 路徑、機器人位址等），與 [`python/demos/grpc_samples/`](../../python/demos/grpc_samples/) 內的其他範例相同，可一併參考。
