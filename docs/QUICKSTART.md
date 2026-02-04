# 快速體驗 Kachaka API (JupyterLab)

透過使用在 Kachaka 本體內運行的 JupyterLab，不受作業系統限制，僅需網頁瀏覽器即可執行 Kachaka API (Python)。推薦用於 Kachaka API 的動作確認和範例程式碼的執行。

> [!NOTE]
> 有關支援的瀏覽器，請參閱 [JupyterLab 官方文件](https://jupyterlab.readthedocs.io/en/stable/getting_started/installation.html#supported-browsers)。

## 目錄
- [開啟 JupyterLab](#開啟-JupyterLab)
- [執行範例程式碼](#執行範例程式碼)
    - [下載範例程式碼](#下載範例程式碼)
    - [安裝執行範例所需的依賴套件](#安裝執行範例所需的依賴套件)
    - [Kachaka 發話的範例程式碼](#Kachaka-發話的範例程式碼)
- [試用 Kachaka API 的方法](#試用-Kachaka-API-的方法)

## 開啟 JupyterLab

1. 確認 Kachaka 的 IP 位址。
    * 在智慧型手機應用程式的「⚙設定」>「應用程式資訊」>「IP 位址」中可以找到 Kachaka 的 IP 位址。
2. 開啟瀏覽器，存取以下 URL。
    * `http://<Kachaka 的 IP 位址>:26501/`（例如：`http://192.168.0.20:26501/`）
    <img src="./quickstart/images/jupyter_url.png" width="400">
* 會顯示登入畫面，請輸入以下密碼。
    * 密碼：kachaka

![jupyter-login](./quickstart/images/jupyter-login.png)

* 如需變更密碼，請參閱以下說明。
<details>
<summary>變更密碼的方法</summary>

* 如需變更密碼，首先從 Launcher 選擇「Terminal」。

<img src="./quickstart/images/jupyter-terminal.png" width="450">

* 在 Terminal 中輸入以下命令。

```console
$ jupyter lab password
Enter password: <新密碼>
Verify password: <新密碼>
```

* 重新啟動 Kachaka 本體後，新密碼即會生效。

</details>

## 執行範例程式碼
### 下載範例程式碼

* 在左側的檔案列表中雙擊 README.ipynb。
* 點擊上方選單的「▶▶」。

<img src="./quickstart/images/jupyter-readme.png" width="600">

* 如果顯示以下對話框，請按下「Restart」按鈕。

<img src="./quickstart/images/jupyter-restart-dialog.png" width="400">

* 範例程式碼下載完成後，會顯示如下訊息，左側的檔案列表中會建立 kachaka-api 資料夾。

<img src="./quickstart/images/jupyter-clone-sample.png" width="600">

### 安裝執行範例所需的依賴套件

安裝執行範例所需的依賴套件。
下載後只需執行一次即可。

* 在左側的檔案列表中選擇 kachaka-api/python/demos 資料夾。
* 雙擊 install_libraries.ipynb，右側會顯示原始碼。
* 按下上方選單的「▶▶」按鈕執行。

### Kachaka 發話的範例程式碼

* 在左側的檔案列表中選擇 kachaka-api/python/demos 資料夾。
* 例如雙擊 speak.ipynb 開啟。
* 右側會顯示原始碼。
* 按下上方選單的「▶▶」按鈕即可執行全部程式碼。

<img src="./quickstart/images/jupyter-sample-speak.png" width="600">

執行結果

Kachaka 會說出「カチャカです、よろしくね」。


## 試用 Kachaka API 的方法
* 接下來，開啟 `kachaka-api/docs/kachaka_api_client.ipynb`。
* 這是一份逐一說明並執行所有 API 的文件。

<img src="./quickstart/images/kachaka_api_client_docs.png" width="600">

* 這次按下上方選單的「▶」按鈕，即可逐一執行程式碼。
* 從最上方開始依序執行，逐一確認每個 API 的動作。
