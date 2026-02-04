<div align="center" style="margin-bottom: 200px;">

  <img src="docs/images/kachaka_api.webp" width="500">

  <img src="docs/images/kachaka_api_logo.png" width="300">

</div>

> **Note**
> 本專案 fork 自 [pf-robotics/kachaka-api](https://github.com/pf-robotics/kachaka-api)，所有日文註解與說明已由 **Claude Opus 4.5** 於 **2026 年 2 月**自動翻譯為**繁體中文**。
> 程式碼邏輯未做任何修改，僅翻譯文件、註解、Notebook markdown cell 及 UI 字串。

##

[智慧家具平台「Kachaka」](https://kachaka.life/) 的API提供儲存庫。

Kachaka API 提供控制 Kachaka 對接和移動、取得 Kachaka 狀態和感測器數值等功能。

* 可從區域網路內的裝置，或 Kachaka 內部的使用者環境（Playground）進行存取。
* 本官方儲存庫提供可在 Python 和 ROS 2 中輕鬆使用 Kachaka API 的 SDK。

* 有關 Kachaka API 可用功能的列表，請參閱「[Kachaka API 功能一覽](./docs/FEATURES.md)」。

### 官方提供的 SDK

* 🐍 Python 3.10+
* 🤖 ROS 2 Humble (Ubuntu 22.04 LTS)

### 其他語言
Kachaka API 以 [gRPC](https://grpc.io/) 通訊介面的形式提供。
其他語言也可以直接使用 gRPC 進行存取。

## 開始使用
### 啟用 Kachaka API
> [!IMPORTANT]
> 無論使用哪種方式，首先都需要透過智慧型手機應用程式啟用 Kachaka API。

* 連接 Kachaka 後，從 [⚙設定] 分頁選擇要連接的機器人，開啟 [Kachaka API] 頁面並將「啟用 Kachaka API」設為開啟。
* 會顯示對話框，請確認「使用條款」後，勾選「同意 Kachaka API 使用條款」並按下「設定」。

<table>
<tr>
<td><img src="./docs/images/spapp_kachaka_api_screen.png" width="150"></td>
<td><img src="./docs/images/spapp_kachaka_api_enable_dialog.png" width="150"></td>
</tr>
</table>

### 確認 Kachaka 的 IP 位址
* 此外，任何情況下都需要 Kachaka 的 IP 位址。
* 可從 [⚙設定] > [應用程式資訊] 中確認。（以下截圖已做遮蔽處理）
* 另外，支援 mDNS 名稱解析，可透過同一畫面中的「序號」組成的
    * `kachaka-<序號>.local` 主機名稱進行存取。

<table>
<tr>
<td><img src="./docs/images/spapp_kachaka_app_info.png" width="150"></td>
<td><img src="./docs/images/spapp_kachaka_app_info_screen.png" width="150"></td>
</tr>
</table>

## Kachaka API 手冊

* 📖 [Kachaka API 功能一覽](./docs/FEATURES.md)
    * 彙整了 Kachaka API 可實現的功能。
* 🚀 [快速體驗 Kachaka API (JupyterLab)](./docs/QUICKSTART.md)
    * 說明如何透過網頁瀏覽器使用 JupyterLab 來使用 Kachaka API。
    * 不受作業系統限制，廣泛適用，推薦用於 Kachaka API 的動作確認和範例程式碼的執行。
* 🐍 [使用 Python 操作 Kachaka API](./docs/PYTHON.md)
    * 說明如何使用 Python 來操作 Kachaka API。
* 🤖 [使用 ROS 2 操作 Kachaka API](./docs/ROS2.md)
    * 說明如何使用 ROS 2 來操作 Kachaka API。
* 🏠 [在 Kachaka 內部（Playground）執行自製程式](./docs/PLAYGROUND.md)
    * Kachaka 內部有一個稱為 Playground 的使用者環境。
    * 無需準備外部裝置，即可在 Kachaka 內部執行自製程式。
* 🌐 [使用 Python 和 ROS2 以外的語言操作 Kachaka API](./docs/GRPC.md)
    * 說明如何使用 Python 和 ROS2 以外的語言來操作 Kachaka API。
* 💻 [在 Web 應用程式中使用 Kachaka API](./docs/WEB.md)
    * 說明如何在 Web 應用程式中使用 Kachaka API。

## 💬 需求・錯誤回報・貢獻

* Kachaka API 以開源軟體（OSS）形式公開。歡迎提出需求和錯誤回報。請參閱[貢獻指南](./CONTRIBUTING.md)。
* 問題和需求請透過 [GitHub Discussions](https://github.com/pf-robotics/kachaka-api/discussions) 提出。

## License
Copyright 2023 Preferred Robotics, Inc.
Licensed under [the Apache License, Version 2.0](LICENSE).
