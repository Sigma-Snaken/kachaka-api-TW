# 家具的移動

## 基本 API
用於移動家具的 API 有多個。基本的有以下 4 個。

|      API      |              角色                    |
|      ---      |              ---                    |
| `move_shelf`  | 將指定的家具移動到指定的位置      |
| `return_shelf`| 將指定的家具收回到家具的主位置      |
| `dock_shelf`  | 對接 Kachaka 正前方的家具  |
| `undock_shelf`| 放下正在承載的家具並脫離      |


### move_shelf
* 呼叫方式：`move_shelf(shelf_id, location_id)`
* 將家具（ID: shelf_id）移動到指定位置（ID: location_id）。

![](./images/move_shelf.png)


### return_shelf
* 呼叫方式：`return_shelf(shelf_id)`
* 將家具（ID: shelf_id）收回到家具的主位置。

* 如果要收回目前正在承載的家具，請將 shelf_id 留空，使用 `return_shelf("")`。

![](./images/return_this_shelf.png)

* 如果指定目前未承載的家具，會前往該家具所在位置進行對接並收回。

![](./images/return_shelf.png)

### dock_shelf
* 呼叫方式：`dock_shelf()`
* 對接 Kachaka 正前方的家具。

![](./images/dock_shelf.png)

### undock_shelf
* 呼叫方式：`undock_shelf()`
* 將正在承載的家具放置在該處，並從家具下方脫離。向前或向後脫離取決於空間狀況。

![](./images/undock_shelf.png)

## 進階 API
* 此外，為了對應更進階的使用案例，提供了以下 API。

|                                       |                                                   |
|                 ---                   |                        ---                        |
| `dock_any_shelf_with_registration`    | 對接在指定位置註冊的家具  |

### dock_any_shelf_with_registration

* 呼叫方式：`dock_any_shelf_with_registration(location_id)`
* 對接放置在指定位置的任意家具。
* **如果家具尚未註冊，則在對接後會進行家具註冊。**

![](./images/dock_any_shelf_with_registration.png)
