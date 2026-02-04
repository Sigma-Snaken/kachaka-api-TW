# 錯誤狀態的偵測與處理

當 Kachaka 的某個動作失敗或設備發生故障時，錯誤資訊會透過 Kachaka API 傳達。這些傳達的錯誤都會被分配一個錯誤碼（編號）。透過參照此錯誤編號，可以得知發生了什麼錯誤以及如何應對。

![Kachaka 的錯誤狀態](./images/kachaka_errors.png)

## 錯誤發生的類型
Kachaka 發出錯誤的時機大致可分為兩種。

### 同步錯誤
同步錯誤是指使用者透過 API 對 Kachaka 發出指令時，作為回覆返回的錯誤。例如，嘗試設定音量時呼叫 SetVolume API，如果要設定的音量超出範圍，回應中就會包含該錯誤資訊。

這類同步 API 的回應中包含以下 Result 訊息。
當 `success` 為 false 時表示錯誤，`error_code` 中包含錯誤碼。

```protobuf
message Result {
  bool success = 1;
  int32 error_code = 3;
}
```

例如設定音量的 SetVolume API，回應如下。

```protobuf
message SetSpeakerVolumeResponse {
  Result result = 1;
}
```


### 非同步錯誤
非同步錯誤是指與任何 API 呼叫無關而發生的錯誤。
例如，從「正在暫停」、「因障礙物無法移動」到相機故障等設備問題，涵蓋了各種 Kachaka 的狀態。
這類非同步錯誤可透過呼叫 `GetError` API 來取得。

```protobuf
message GetErrorResponse {
  Metadata metadata = 1;
  repeated int32 error_codes = 2;
}
```

如上所示，回應中的 `error_codes` 陣列包含了表示當前錯誤狀態的所有錯誤碼。
像暫停這樣的持續性狀態，會一直包含在內直到解除。
另一方面，像「因障礙物無法移動」這樣的事件性錯誤，會在該錯誤發生的時刻被包含。

## 錯誤的種類
錯誤碼與其含義的對照表由機器人持有主資料，可透過 `GetRobotErrorCodeJson` API 取得。
原則上，編號不會改變其含義，在版本升級時也保持一致性。（同類別的錯誤可能會被進一步細分）

### 錯誤碼摘錄
部分摘錄如下。例如，暫停狀態可透過檢查 21051 是否包含在錯誤中來判斷。

<details>
<summary>錯誤碼摘錄</summary>

```json
[
  ...
  {
    "code": 14605,
    "title": "充電ドック上に家具を置くことはできません",
    "description": "",
    "title_en": "Furniture cannot be placed on the charging dock",
    "description_en": "",
    "error_type": "Error",
    "ref_url": ""
  },
  {
    "code": 14606,
    "title": "家具を載せていません",
    "description": "",
    "title_en": "Kachaka is not docked with a furniture",
    "description_en": "",
    "error_type": "Error",
    "ref_url": ""
  },
  ...
  {
    "code": 21051,
    "title": "一時停止しています",
    "description": "電源ボタンを押して解除してください。",
    "title_en": "Kachaka is paused",
    "description_en": "Please press the power button to resume from the pause state.",
    "error_type": "Warn",
    "ref_url": ""
  },
  {
    "code": 21052,
    "title": "段差を検知しました",
    "description": "段差を検知したため、一時停止しました。段差のない場所にカチャカを移動した上で、電源ボタンを押して解除してください。",
    "title_en": "Step detected",
    "description_en": "Kachaka has paused because a step was detected. Please move Kachaka to a flat surface and press the power button to resume from the pause state.",
    "error_type": "Error",
    "ref_url": ""
  },
  ...
```
</details>

### 錯誤的嚴重程度
上述取得的 JSON 中包含 "error_type" 欄位。這表示錯誤的嚴重程度，同步錯誤和非同步錯誤分別有以下種類。

#### 同步錯誤（API 呼叫時）
| 錯誤類型 | 說明 |
|------------|------|
| Ignore | 不算成功，但可忽略的程度 |
| Warn | 最好告知的程度 |
| Error | 失敗，但再次嘗試可能成功 |
| Bug | 異常情況發生（需要更新） |

#### 非同步錯誤（機器人狀態）
| 錯誤類型 | 說明 | LED |
|------------|------|------|
| Warn | 最好告知的程度資訊 |  |
| Error | 可透過非重啟方式恢復 | 黃色 |
| Recoverable | 可在一定時間內恢復 | 白色旋轉 |
| Fatal | 可透過重啟恢復 | 紅色 |
| CallForSupport | 嚴重故障（請聯繫客服支援） | 紅色閃爍 |
