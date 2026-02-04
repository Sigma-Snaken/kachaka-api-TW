# 在 Web 應用程式中使用 Kachaka API

* 在瀏覽器中運行的 Web 應用程式也可以使用 Kachaka API。
* 但由於目前 Web 應用程式無法直接使用 gRPC（HTTP/2），因此需要架設代理伺服器，透過 grpc-web 協定進行通訊。
  * 可從 JavaScript 自由處理 gRPC 通訊所需的 HTTP/2 的環境有限，因此需要使用以 HTTP/1.1 表達的 grpc-web 協定。
* 這裡介紹使用 envoy 架設代理的方法，並展示從 Web 應用程式使用 Kachaka API 的範例。

## 目錄
- [代理伺服器](#代理伺服器)
- [Web 範例（React + TypeScript）](#web-範例-react--typescript)


## 代理伺服器

* 啟動代理伺服器

```bash
$ ./tools/web_proxy/start_proxy_remote.sh <Kachaka 的 IP 位址>
```

* 代理伺服器可以在任何能被 Web 應用程式存取且具有 Kachaka 網路連線的地方啟動。
* 此腳本展示了在 `localhost:50000` 架設代理伺服器的範例。

## Web 範例（React + TypeScript）

![](web/images/web_sample_capture.png)

* 這是使用 React 與 Kachaka API 整合的 Web 應用程式範例。
* 按下加號按鈕即可新增面板，選擇面板類型後會顯示使用對應 API 的展示。
* 要啟動請執行以下命令。

```bash
$ cd web/demos/kachaka_api_web_sample
$ npm install
$ npm run dev
```

* 如果沒有 npm 環境，請自行進行安裝。
* 以下是安裝方法的範例。
  * 透過 apt 安裝的 npm 可能版本較舊。建議使用 n 安裝最新的 stable 版本。
  * 範例已在 nodejs v18.17.1、npm 9.6.7 上確認可正常運作。

```bash
$ sudo apt install nodejs npm
$ sudo npm install -g n
$ sudo n stable
```

### Kachaka API 與 React hook
* 如 [使用 Cursor 進行長輪詢](./GRPC.md#使用-cursor-進行長輪詢) 中所介紹，Kachaka API 使用稱為 cursor 的概念來實現高效的值更新。
* 在 React hook 中，利用 cursor 的更新來呼叫 Get 系列 API，每次收到回應時更新 state，可以在值變更時以最少的計算進行渲染。
* 具體處理請參閱 `web/demos/kachaka_api_web_sample/src/kachakaApi.ts`。
