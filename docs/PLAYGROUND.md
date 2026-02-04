# 在 Kachaka 內部（Playground）執行自製程式

可以使用 Kachaka API 開發的程式，在 Kachaka 本體的部分資源（Playground）上執行。
Playground 是在 Kachaka 內部運行的 Docker 容器，以 Ubuntu 22.04 LTS 為基礎。
使用 Playground，可以開發僅靠 Kachaka 就能完成的系統。


## 目錄
- [Playground 的規格](#playground-的規格)
  - [連接埠](#連接埠)
  - [從 Playground 存取 Kachaka API](#從-playground-存取-kachaka-api)
  - [Playground 的資源限制](#playground-的資源限制)
- [透過 ssh 登入 Playground](#透過-ssh-登入-playground)
- [在 Playground 中執行範例程式](#在-playground-中執行範例程式)
- [自製軟體的自動啟動](#自製軟體的自動啟動)
  - [範例）自動啟動報時範例](#範例-自動啟動報時範例)

## Playground 的規格
### 連接埠
* Kachaka 公開的連接埠中，與 Playground 相關的如下。

| 連接埠號 | 用途 |
| --- | --- |
| 26400 | KachakaAPI 伺服器 (gRPC) |
| 26500 | Playground 的 ssh |
| 26501 | Playground 的 JupyterLab |
| 26502~26509 | 未分配（可自由使用） |

### 從 Playground 存取 Kachaka API
* 從 Playground 內部使用 Kachaka API 時，伺服器位址為 `100.94.1.1:26400`。
  * Python 的 kachaka_api 套件中，`KachakaApiClient` 的預設值即為此位址。

### Playground 的資源限制

* 儲存空間合計（/home, tmp）3GB
* 記憶體 512MB


## 透過 ssh 登入 Playground

* 使用 JupyterLab 的終端機或以下任一 notebook 進行公鑰設定
    * utils/set_authorized_keys.ipynb
    * utils/set_authorized_keys_from_github.ipynb
        * 如果想使用在 GitHub 上註冊的金鑰，此腳本很方便
* 使用 utils/set_authorized_keys.ipynb 的設定方法
    * 選擇畫面左上角的 File Browser。
    * 在畫面左側的檔案列表中雙擊 utils → set_authorized_keys.ipynb。
    * 在畫面中央的 public_keys 中貼上公鑰文字。
    * 按下上方選單的「▶▶」按鈕。

<img src="playground/images/set_authorized_kyes.png" alt="set-authorized-keys" width="600">

* 使用 utils/set_authorized_keys_from_github.ipynb 的設定方法
    * 選擇畫面左上角的 File Browser。
    * 在畫面左側的檔案列表中雙擊 utils → set_authorized_keys_from_github.ipynb。
    * 在畫面中央的 user 中輸入 GitHub 的使用者名稱。
    * 按下上方選單的「▶▶」按鈕。

<img src="playground/images/set_authorized_keys_from_github.png" alt="set-authorized-keys-from-github" width="600">

執行以下命令登入 Playground

```bash
ssh -p 26500 -i <對應已註冊公鑰的私鑰> kachaka@<Kachaka 的 IP 位址>
```

## 在 Playground 中執行範例程式

* 透過 ssh 登入 Kachaka。
* 執行以下命令後，Kachaka 會每隔 1 分鐘播報時間。

```bash
cd ~
git clone https://github.com/pf-robotics/kachaka-api.git
pip install -r /home/kachaka/kachaka-api/python/demos/requirements.txt
python3 /home/kachaka/kachaka-api/python/demos/time_signal.py 100.94.1.1:26400
```

## 自製軟體的自動啟動

* 如果希望 Kachaka 重新啟動時自動執行程式，可以使用自動啟動功能。
* 在 Playground 的 `/home/kachaka/kachaka_startup.sh` 中寫入要自動啟動的處理，Kachaka 啟動時會自動執行。
* 日誌會記錄在 `/tmp/kachaka_startup.log`
    * 自動啟動 python3 時建議加上 `-u` 選項。否則標準輸出會被緩衝，可能無法確認日誌。

### 範例）自動啟動報時範例

* 以下以自動啟動 Kachaka 報時範例為例進行說明。
* 將 `/home/kachaka/kachaka_startup.sh` 編輯如下。

```bash
#!/bin/bash

jupyter-lab --port=26501 --ip='0.0.0.0' &

# 新增以下行
python3 -u /home/kachaka/kachaka-api/python/demos/time_signal.py 100.94.1.1:26400 &
```

* 儲存後，重新啟動 Kachaka，稍等片刻即會每隔 1 分鐘播報當前時間。
