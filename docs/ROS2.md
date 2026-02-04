# 使用 ROS 2 操作 Kachaka API

* 說明從 ROS 2 使用 Kachaka API 的步驟。
* 官方提供 gRPC 與 ROS 2 的橋接功能，以及 URDF 等 description。

## 目錄
- [kachaka-api ROS 2 提供的內容](#kachaka-api-ros-2-提供的內容)
- [ROS 2 Humble 的設定](#ros-2-humble-的設定)
- [使用 Docker 啟動 ros2_bridge](#使用-docker-啟動-ros2_bridge)
  - [安裝 Docker](#安裝-docker)
  - [啟動橋接](#啟動橋接)
- [確認運作](#確認運作)
- [與其他 ROS 2 套件整合](#與其他-ros-2-套件整合)
  - [建置 kachaka_interfaces、kachaka_description](#建置-kachaka_interfaces-kachaka_description)
  - [使用 RViz2 進行視覺化](#使用-rviz2-進行視覺化)
- [範例程式碼](#範例程式碼)
- [自行建置 Docker 映像檔](#自行建置-docker-映像檔)

## kachaka-api ROS 2 提供的內容
| 套件名稱 | 角色 |
| --- | --- |
| kachaka_grpc_ros2_bridge | 從 ROS 2 使用 Kachaka API 的橋接 |
| kachaka_interfaces | kachaka_grpc_ros2_bridge 的介面定義 |
| kachaka_description | Kachaka 的 URDF 模型 |

* 針對 ROS2 Humble 環境，我們公開了已完成建置的 Docker 映像檔。接下來的說明將使用此 Docker 映像檔來啟動 ros2_bridge。
* 您也可以自行建置，以便在 ROS 2 Humble 以外的版本中使用。

## ROS 2 Humble 的設定
* 請參考以下連結設定 ROS 2 Humble。
    * https://docs.ros.org/en/humble/index.html

## 使用 Docker 啟動 ros2_bridge
### 安裝 Docker
* 請參考以下連結進行 Docker 的設定。
    * https://docs.docker.com/engine/install/ubuntu/

### 啟動橋接
* 執行以下腳本即可啟動橋接。
* 首次執行時會下載 Docker 映像檔。
    * ※ 映像檔的提供可能會在不另行通知的情況下停止。

```bash
cd ~/kachaka-api/tools/ros2_bridge
./start_bridge.sh <Kachaka 的 IP 位址>
```

### 確認運作

* 確認是否能從 topic 取得訊息。
* 由於橋接的容器已啟動，可以使用它來確認 topic 等資訊。

* 取得 topic 列表
```bash
docker exec -it ros2_bridge-ros2_bridge-1 /opt/kachaka/env.sh ros2 topic list
```

* 取得目的地列表

```bash
docker exec -it ros2_bridge-ros2_bridge-1 /opt/kachaka/env.sh ros2 topic echo /kachaka/layout/locations/list
```

* 如果收到以下回應則表示成功。

```yaml
locations:
- id: L01
  name: ダイニング
  type: 0
  pose:
    x: 1.33572
    y: 2.328592
    theta: 0.0
```


## 與其他 ROS 2 套件整合

### 建置 kachaka_interfaces、kachaka_description

* 由於橋接提供的 topic 使用 Kachaka 自訂介面，因此需要建置這些套件。
    * kachaka_description 請依需求自行選擇是否建置。
* 按以下步驟進行建置。

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ln -s ~/kachaka-api/ros2/kachaka_interfaces/ kachaka_interfaces
ln -s ~/kachaka-api/ros2/kachaka_description/ kachaka_description

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
```

### 使用 RViz2 進行視覺化
* 讓我們使用 RViz2 來視覺化從橋接取得的資訊。
* kachaka_description 中有範例 config，先使用這個。

```bash
cd ~/ros2_ws
source install/setup.bash
cd src/kachaka_description/config
rviz2 -d kachaka.rviz
```

## 範例程式碼

* 範例程式碼位於以下位置
    * [ros2/demos](../ros2/demos)
* 範例程式碼的執行方法
    * 請依照範例程式碼 README.md 中的步驟執行


## 自行建置 Docker 映像檔
* 預設設定使用發行的 Docker 映像檔啟動 Docker 容器並在其上運行 ros2_bridge。使用發行的 Docker 映像檔時，可跳過此步驟。
* 如需自訂 Docker 映像檔，請按以下步驟建置。
    * 以下範例中設定 `BASE_ARCH=x86_64`，這是在 x86_64 架構 CPU 上執行 ros2_bridge 的情況。
    * 如要在 arm64 架構 CPU 上執行 ros2_bridge，請設定 `BASE_ARCH=arm64`。

```bash
docker buildx build -t kachaka-api --target kachaka-grpc-ros2-bridge -f Dockerfile.ros2 . --build-arg BASE_ARCH=x86_64 --load
```

* 對 [tools/ros2_bridge/docker-compose.yaml](../tools/ros2_bridge/docker-compose.yaml) 進行以下變更。
    * 變更前：`image: "asia-northeast1-docker.pkg.dev/kachaka-api/docker/kachaka-grpc-ros2-bridge:${TAG}"`
    * 變更後：`image: kachaka-api:latest`
