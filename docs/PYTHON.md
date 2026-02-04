# 使用 Python 操作 Kachaka API

* 為了方便在 Python 中使用 Kachaka API，官方提供了名為 `kachaka_api` 的 Python 套件。
* [使用 JupyterLab 體驗 Kachaka API](./QUICKSTART.md) 中使用的就是這個套件。本文件說明如何在您自己的電腦上執行 Python 來使用 Kachaka API。

## 目錄
- [安裝方法](#安裝方法)
- [基本使用方式](#基本使用方式)
- [範例程式碼](#範例程式碼)
- [非同步套件 (aio)](#非同步套件-aio)


## 安裝方法

### pip
可使用以下命令進行安裝：

```bash
pip install kachaka-api
```

### uv
使用 uv 的情況下可使用以下命令進行安裝。

```bash
uv add kachaka-api
```

### 發行版本與版本號
kachaka_api 套件會在 Kachaka 軟體更新發佈時同步發行（可能會延遲數天至一週左右）。
版本由 4 個數字組成，Kachaka 軟體版本的 3 個數字後面接著 kachaka_api 套件的版本號。
例如在 Kachaka SW3.10.6 發行時會發佈 kachaka_api 套件 3.10.6.0，如果之後僅套件有重要變更，則會發佈第 4 個數字遞增的 3.10.6.1、3.10.6.2 等版本。

## 基本使用方式
* kachaka_api 套件中，`KachakaApiClient` 類別對應一台 Kachaka。
* 每個 API 都可以作為此類別的方法來執行。

```python
from kachaka_api import KachakaApiClient

client = KachakaApiClient(target="192.168.1.100:26400")

# 取得狀態
current_pose = client.get_robot_pose()
print(f"current pose: {current_pose}")

# 操作和指令
client.speak("カチャカです、よろしくね")
```

* 我們公開了一份可以逐一執行所有 API 並附帶說明的筆記本。
    * [kachaka_api_client.ipynb（Kachaka API 文件）](./kachaka_api_client.ipynb)

<img src="./python/images/kachaka_api_client_docs.png" width="600">


> [!CAUTION]
> 在您自己的電腦上執行 jupyter 時，請在開頭的儲存格中為 KachakaApiClient 指定 Kachaka 的 IP 位址。
> ```diff
> import kachaka_api
>
> -client = kachaka_api.KachakaApiClient()
> +client = kachaka_api.KachakaApiClient(target="<您的 Kachaka 的 IP 位址>:26400")
> ```

## 範例程式碼

* [sample_llm_speak.py](../python/demos/sample_llm_speak.py) ... 使用 ChatGPT 在指令結束時進行對話的範例
* 其他在 [python/demos/](../python/demos) 下還有許多可在 JupyterLab 中使用的 Notebook 格式範例

## 非同步套件 (aio)
> [!NOTE]
> 如果您還不太熟悉 Python，可以跳過這一節。

* Kachaka API 提供同步和非同步兩種介面。
    * 兩種介面基本上相同，只需將匯入來源切換為 aio，即可使用 async 版的介面。

```python
import asyncio

from kachaka_api.aio import KachakaApiClient

async def main() -> None:
    client = KachakaApiClient(target="192.168.1.100:26400")

    # 取得狀態
    current_pose = await client.get_robot_pose()
    print(f"current pose: {current_pose}")

    # 操作和指令
    await client.speak("カチャカです、よろしくね")

if __name__ == "__main__":
    asyncio.run(main())
```


* 此外，非同步套件除了同步套件的功能外，還支援 callback 註冊。
  * 有關 callback 功能，請參閱 [sample_llm_speak.py](../python/demos/sample_llm_speak.py)。

* 非同步版同樣也準備了逐一執行 API 的文件。
    * [kachaka_api_client_async.ipynb](./kachaka_api_client_async.ipynb)

