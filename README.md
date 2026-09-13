# mio-dashcam

Mio 行車記錄器的檔案整理與轉檔工具。把記憶卡上散落的三分鐘片段，還原成一趟行程一支影片：
後鏡頭疊在前鏡頭上方（PIP）、右上角畫出這趟路的 GPS 軌跡 (Overlay)、音量正規化

## Requirements

- `ffmpeg` / `ffprobe`（macOS：`brew install ffmpeg`）
- 編碼使用 `hevc_videotoolbox`，也就是需要 Apple Silicon / Intel Mac 的硬體編碼器

```sh
cargo build --release      # → target/release/mio-dashcam
```

## Usage

```sh
# 1. 整理記憶卡：Normal/ 與 Event/ 攤平成每日一個資料夾
mio-dashcam organize /Volumes/DASHCAM ~/Movies/DASHCAM
mio-dashcam organize -n /Volumes/DASHCAM        # 先看看會做什麼，不動檔案
mio-dashcam organize --copy /Volumes/DASHCAM .  # 複製而非搬移

# 2. 看看有哪些連續段落（不會動到任何檔案）
mio-dashcam list ~/Movies/DASHCAM

# 3. 轉檔
mio-dashcam render ~/Movies/DASHCAM -o ~/Movies/out
mio-dashcam render ~/Movies/DASHCAM/20260820 -o out -y --skip-existing

# 只轉其中幾段（序號就是 list 印出來的那個）
mio-dashcam render ~/Movies/DASHCAM -s 12
mio-dashcam render ~/Movies/DASHCAM -s 2,17-18
mio-dashcam render ~/Movies/DASHCAM -s 20260911        # 整天
mio-dashcam render ~/Movies/DASHCAM -s 20260905_235044 # 指名某一段
```

`-s/--select` 的每個項目：4 位數以內的數字是**序號**（`3`、`2-5`），其他都當成**名稱前綴**
（`20260911` 是整天、`20260911_0919` 是那個時段）。逗號並列，重複的只會算一次。
指到不存在的東西會直接報錯，不會默默少轉幾支。`list` 也吃同一個參數，可以先確認選到什麼；
不論有沒有 `-s`，印出來的序號都是完整清單裡的序號，不會因為過濾而跳號。

輸入來源三種都吃，因為分段只看檔名解出來的時間，不看資料夾結構：

| 來源 | 例子 |
| --- | --- |
| 未整理的記憶卡 | `Normal/F/`、`Normal/R/`、`Normal/NMEA/`、`Event/…` |
| 已整理的封存 | `20260820/FILE260820-205705F.mp4` |
| 任何含影片的資料夾 | 單一天、單一段，或整個封存 |

輸出檔名是 `20260820_205705.mp4`（該段起始時間），與既有的 `gpx/` 命名一致。

### 常用選項

`render`：

| 選項 | 說明 |
| --- | --- |
| `-s, --select <SPEC>` | 只處理指定的段落，見上 |
| `--no-gps` | 不解析 NMEA、不畫軌跡 |
| `--big` | 軌跡圖用 1920px（預設 540px） |
| `--skip-existing` | 已有同名輸出就跳過，適合中斷後續跑 |
| `--keep` | 保留串接後的 `_F` / `_R` 中間檔 |
| `-y` | 不問直接開始 |
| `--pip-height <PX>` | 後鏡頭子母畫面高度，預設 432 |
| `--gap <SEC>` | 連續判定的容許間隔，預設 5 秒 |
| `--tz <±HH:MM>` | 記錄器時區；預設自動推得 |

Ctrl-C 不會中途砍掉 ffmpeg，而是等目前這支跑完再停，輸出不會留下半截檔案。

## How it works

以 dashcam clip/NMEA 為來源、以影片 timeline 為主軸、以每 frame transparent video overlay 為輸出的 GPS renderer

**檔名即索引。** `FILE260820-205705F.mp4`、`…R.mp4`、`…F.NMEA` 三個檔靠同一個時間戳配對，
所以前後鏡頭與 GPS 記錄不管放在哪個子資料夾都能湊回同一段。事件檔 `EMER…` 在整理時
改名成 `FILE…`，讓它和一般片段一起排序。

**連續判定看時間，不看資料夾。** 下一段的開始時間若不超過「上一段結束 + 5 秒」就算同一趟。
片長是實際 probe 出來的，所以熄火時被截短的最後一段不會把下一趟黏上來。跨午夜的行程在
已整理的封存裡會被拆在兩個日期資料夾，這裡仍然併成一段。

**GPS 對時。** NMEA 裡的 RMC 是 UTC，檔名是記錄器本地時間；而且記錄器開機後要幾秒才寫出
第一筆，所以每段的第一筆並不對齊 0 秒。做法是：用「檔名時間 − 首筆 UTC」四捨五入到 15 分鐘
推出時區（可用 `--tz` 指定），再把每筆有效位置放到「前面各段片長總和 + 段內偏移」的位置上；
無定位的 RMC 時間仍保留為空洞，不能被誤當成直線中間的有效點。

**GPS 異常處理。** 這裡將「session 結構錯誤」與「單筆訊號異常」分開處理：

- 壞 checksum、格式錯誤或缺日期的 RMC 只丟掉該行。
- 某一個 `.NMEA` sidecar 讀不到或沒有有效 RMC 時印 warning，其他片段仍繼續處理；
  該片段會成為 GPS gap，影片本身不會因此失敗。
- `V`（無定位）保留時間位置但不建立 Fix；缺少 speed 或 course 時使用 `NaN`，不偽裝成 0。
- interpolation 只接受有限數值；單一 sample 只能在自己的 timestamp 使用，跨過超過 10 秒的 gap、
  或遇到反向時間軸時回傳空位置。相同 timestamp 的 interpolation 使用第一筆 sample。
- GPS 導出的橫向/縱向加速度若超過 10 g 會變成 `NaN`，但不會因此刪掉原始位置；位置跳點目前仍不做
  獨立的物理速度 outlier 過濾。

**1 Hz → 30 fps。** 航向先 unwrap 成連續角度再線性插值，所以箭頭滑順移動而不是每秒跳一格。空洞超過 10 秒
不硬接 —— 軌跡在那裡斷開、箭頭也不畫，免得畫出一條沒人開過的直線；10 秒內的才補，
補出來的點用白色箭頭、1.5 秒內有真實定位的用紅色箭頭區分。

**軌跡與運動量。** 投影是球面 Mercator，整趟先框一個 bounding box 定住視角，地圖才不會在車底下漂。
畫之前抽點（距上一點要同時滿足
≥0.2 秒與 ≥3 公尺），等紅燈原地抖動的上百筆會塌成兩點。另外由位置／速度／航向導出橫向
與縱向加速度（`gpsLatAcc`／`gpsLongAcc` 各套 1.2 秒平均），渲染前印出整趟的最高速與峰值。

**合成。** 前鏡頭 3840×2160、後鏡頭 2560×1440，兩者各自 `-c copy` 串接（只留主視訊軌，
丟掉縮圖軌與 data 軌）。後鏡頭裁掉上方 1/4（車內頂篷與反光）後縮到 432px 貼在上方置中，
軌跡貼右上角 —— 底部有記錄器燒進去的狀態列與浮水印，上方中央被 PIP 占走，右上是唯一
乾淨的角落。音訊做 loudnorm 兩段式量測，
以 `linear=true` 施一個固定增益到 −16 LUFS —— 不壓縮、不 gating，動態與細節都保留。

## Tests

```sh
cargo test
```

涵蓋檔名解析、NMEA（含 checksum、無定位紀錄、南/西半球）、連續分段（含跨午夜與被截短的
尾段）、濾波器與插值（航向繞圈、收訊中斷、單筆 sample、NaN/非遞增時間）、Mercator 投影與
抽點，以及軌跡光柵化與已走／未走的切換。
