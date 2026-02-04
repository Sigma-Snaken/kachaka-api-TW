# 智慧音箱連動範例

## 開始使用
* 此範例進行 Kachaka、Google Home、IFTTT、Beebotte 的服務連動。
* 需要 Google Nest Mini 等裝置，以及 IFTTT、Beebotte 服務的註冊。

## Beebotte 的設定

* 登入 Beebotte 網站：https://beebotte.com/
* 建立新的頻道/資源
  * 選擇左側選單的 Channels
    * 按下 My Channels 右側的「Create New」按鈕
    * Channel Name: test
    * Resource name: sample
    * 按下「Create Channel」按鈕
* 儲存以下位置的權杖
  * 選擇左側選單的 Channels
  * 從 My Channels 選擇 test
  * Channel Token: token_xxxxxxx

## IFTTT 的設定

* 登入 IFTTT 網站：https://ifttt.com/
* 按下「create」按鈕
* 按下「If This」，進行以下設定
  * 從 Choose a service 選擇 Google Assistant V2
  * 按下「activate scene」
     * 設定呼叫服務時對 Google Home 說的話，例如「把家具帶過來」
  * 按下「Create Trigger」
* 按下「Then That」，進行以下設定
  * 從 Choose a service 選擇 Webhooks
  * 按下「Make a web request」，進行以下設定
    * URL: https://api.beebotte.com/v1/data/publish/test/sample?token=<Beebotte 的權杖字串>
    * Method: POST
    * Content Type: application/json
    * Body: {"data": {}}
  * 按下「Create action」

## Google Home 的設定

* 請參考以下進行設定
  * https://support.google.com/googlenest/answer/7194656?hl=en&co=GENIE.Platform%3DAndroid
* 提示
  * 使用標準 IFTTT 用法時，需要說「OK Google，啟用把家具帶過來」才能執行。
  * 在 Google Assistant 應用程式的例行程序設定中，可以變更為只說「OK Google，把家具帶過來」即可執行。
    * 選擇「自動化」
    * 用「＋」新增例行程序
    * 「新增啟動條件」→「對 Google 助理說話時」→ 設定「把家具帶過來」，按下「新增條件」
    * 「新增動作」→「新增自訂動作」→ 設定「啟用把家具帶過來」，按下「完成」

## 準備

```
git clone https://github.com/pf-robotics/kachaka-api.git # 本儲存庫
python3 -m venv venv
source venv/bin/activate
cd kachaka-api/python/demos
pip install -r requirements.txt

python -m grpc_tools.protoc -I../../protos --python_out=. --pyi_out=. --grpc_python_out=. ../../protos/kachaka-api.proto

wget https://beebotte.com/certs/mqtt.beebotte.com.pem
```

## 執行

```
export TOKEN=<Beebotte 的權杖字串>
python smart_speaker.py <Kachaka 的 IP 位址>:26400
```

對 Google Home 說「OK Google，把家具帶過來」，Kachaka 就會搬運家具。

