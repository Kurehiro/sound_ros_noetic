# sound_ros_noetic 利用手順

このリポジトリは、ROS Noetic で
**音声トリガ検出 → 録音 → Whisper文字起こし → ROSトピック配信**
を行うための実行環境です。

## 0. この構成で解決している問題

ROS1 は Master 登録後にノード同士で直接通信します。
そのため、`network_mode: host` + ホストIP広告の構成では、別コンテナから到達できず Subscribe が失敗することがあります。

このリポジトリでは、`ros_net`（bridge）上で `ros_audio_container` を固定名で動かし、
`ROS_MASTER_URI=http://ros_audio_container:11311` を基準に通信することで、コンテナ間通信を安定化しています。

---

## 1. 事前準備

- Docker / Docker Compose が使えること
- Linuxホストでマイクが認識されていること
- 音声入力のため `/dev/snd` が利用できること
- GUIを使う場合は、ホスト側で X サーバーが使えること（`xhost`）

---

## 2. 初回セットアップ

```bash
cd sound_noetic
bash env_setup.sh
```

`env_setup.sh` では、以下を `.env` に保存します。

- `ROS_MASTER_URI`（デフォルト: `http://ros_audio_container:11311`）
- `ROS_HOSTNAME`（デフォルト: `ros_audio_container`）
- `DISPLAY`
- `XAUTHORITY_FILE`（デフォルト: `$HOME/.Xauthority`）

> 補足: この構成は PC 依存の PulseAudio ソケットマウントを外し、
> どの PC でも `docker compose up` で起動しやすい設定にしています。

---

## 3. コンテナの作成と起動（3ステップ）

```bash
cd sound_noetic
bash 01_build_image.sh
bash 02_create_container.sh
bash 03_start_container.sh
```

短い別名でも同じです。

```bash
bash build_image.sh
bash create_container.sh
bash start_container.sh
```

---

## 4. コンテナへ入る

```bash
cd sound_noetic
bash enter_docker.sh
```

`enter_docker.sh` は以下を実施します。

- `docker compose up -d ros_audio`
- コンテナIPを取得して `ROS_IP` として設定
- `terminator_config` を毎回 `/root/.config/terminator/config` に反映
- Terminator レイアウト `ros_8` を起動（8ペイン）
- GTK/X11 安定化のため `GDK_DISABLE_SHM=1` / `NO_AT_BRIDGE=1` / `--no-dbus` を適用
- `DISPLAY` に対応する X ソケットがなければ `/tmp/.X11-unix/X*` から自動補正
- `xhost +SI:localuser:root` も実行して、コンテナ root の X 接続を許可
- 各ペインは `ros_shell_init.sh` を経由して `devel/setup.bash` と ROS 環境変数を自動設定
- 失敗時は `bash -lc /root/workspace/ros_shell_init.sh` へフォールバック

---

## 5. コンテナ内でROSを起動

### 5-1. ワークスペースをビルド（初回または変更時）

```bash
cd /root/workspace
catkin_make
source /root/workspace/devel/setup.bash
```

### 5-2. 音声パイプライン起動

```bash
source /root/workspace/devel/setup.bash
roslaunch sound_send whisper.launch
```

必要に応じて引数を指定します。

```bash
roslaunch sound_send whisper.launch mic_id:=f vol_thresh:=0.2
```

`whisper.launch` では `sound_trigger.py` と `whisper_node.py` が起動します。

---

## 6. 別コンテナから Subscribe する場合

購読側コンテナも同じ `ros_net` に参加させてください。

例（購読側 compose）:

```yaml
networks:
  ros_net:
    external: true
    name: ros_net
```

購読側の環境変数例:

- `ROS_MASTER_URI=http://ros_audio_container:11311`
- `ROS_HOSTNAME=<購読側コンテナ名>`

---

## 7. よく使う確認コマンド

ホスト側:

```bash
docker ps -a | grep ros_audio_container
docker logs ros_audio_container --tail 100
docker network inspect ros_net
```

コンテナ内:

```bash
printenv | grep -E 'ROS_MASTER_URI|ROS_HOSTNAME|ROS_IP'
rostopic list
rostopic echo /whisper_result
```

---

## 8. 停止・削除

```bash
cd sound_noetic
docker compose stop
# 完全に消す場合
docker compose down
```