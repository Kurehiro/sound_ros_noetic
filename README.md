<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
### ポイント
- `git commit` しただけでは、**自分のPC（ローカル）に記録されるだけ**です。
- `git push` して初めて、GitHub（リモート）に反映されます。
- ブランチ名が分からない場合は `git branch --show-current` で確認できます。
=======
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
# sound_ros_noetic 利用手順

このリポジトリは、ROS Noetic で
**音声トリガ検出 → 録音 → Whisper文字起こし → ROSトピック配信**
を行うための実行環境です。

<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
=======
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
## 0. この構成で解決している問題
ROS1は Master 登録後にノード同士で直接通信します。
そのため、`network_mode: host` + ホストIP広告だと別コンテナから到達できず、Subscribeが失敗することがあります。

このリポジトリでは、`ros_net` (bridge) 上で `ros_audio_container` を固定名で動かし、
`ROS_MASTER_URI=http://ros_audio_container:11311` を基準に通信することで、コンテナ間通信を安定化しています。

<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
---

## 1. 事前準備

- Docker / Docker Compose が使えること
- Linuxホストでマイクが認識されていること
- 音声入力を使うため `/dev/snd` が利用できること
- GUIを使う場合はホスト側で X サーバーが使えること（`xhost`）

---

## 2. 初回セットアップ

```bash
cd sound_noetic
bash env_setup.sh
```

`env_setup.sh` では以下を `.env` に保存します。
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
- `ROS_MASTER_URI`
- `ROS_IP`
=======
- `ROS_MASTER_URI`（デフォルト: `http://ros_audio_container:11311`）
- `ROS_HOSTNAME`（デフォルト: `ros_audio_container`）
>>>>>>> theirs
=======
- `ROS_MASTER_URI`（デフォルト: `http://ros_audio_container:11311`）
- `ROS_HOSTNAME`（デフォルト: `ros_audio_container`）
>>>>>>> theirs
=======
- `ROS_MASTER_URI`（デフォルト: `http://ros_audio_container:11311`）
- `ROS_HOSTNAME`（デフォルト: `ros_audio_container`）
>>>>>>> theirs
=======
- `ROS_MASTER_URI`（デフォルト: `http://ros_audio_container:11311`）
- `ROS_HOSTNAME`（デフォルト: `ros_audio_container`）
>>>>>>> theirs
=======
- `ROS_MASTER_URI`（デフォルト: `http://ros_audio_container:11311`）
- `ROS_HOSTNAME`（デフォルト: `ros_audio_container`）
>>>>>>> theirs
=======
- `ROS_MASTER_URI`（デフォルト: `http://ros_audio_container:11311`）
- `ROS_HOSTNAME`（デフォルト: `ros_audio_container`）
>>>>>>> theirs
=======
- `ROS_MASTER_URI`（デフォルト: `http://ros_audio_container:11311`）
- `ROS_HOSTNAME`（デフォルト: `ros_audio_container`）
>>>>>>> theirs
=======
- `ROS_MASTER_URI`（デフォルト: `http://ros_audio_container:11311`）
- `ROS_HOSTNAME`（デフォルト: `ros_audio_container`）
>>>>>>> theirs
=======
- `ROS_MASTER_URI`（デフォルト: `http://ros_audio_container:11311`）
- `ROS_HOSTNAME`（デフォルト: `ros_audio_container`）
>>>>>>> theirs
=======
- `ROS_MASTER_URI`（デフォルト: `http://ros_audio_container:11311`）
- `ROS_HOSTNAME`（デフォルト: `ros_audio_container`）
>>>>>>> theirs
=======
- `ROS_MASTER_URI`（デフォルト: `http://ros_audio_container:11311`）
- `ROS_HOSTNAME`（デフォルト: `ros_audio_container`）
>>>>>>> theirs
- `DISPLAY`
- PulseAudio 用のパス情報

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

<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
または直接:

```bash
docker exec -it ros_audio_container bash
```
=======
`enter_docker.sh` は以下を実施します。
- `docker compose up -d ros_audio`
- コンテナIPを取得して `ROS_IP` として設定
- `ROS_MASTER_URI` / `ROS_HOSTNAME` / `ROS_IP` を付与して `docker exec`
>>>>>>> theirs
=======
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
`enter_docker.sh` は以下を実施します。
- `docker compose up -d ros_audio`
- コンテナIPを取得して `ROS_IP` として設定
- `terminator_config` を毎回 `/root/.config/terminator/config` に反映
<<<<<<< ours
- Terminatorレイアウト `ros_13` を起動（大1 + 中4 + 小8 の13ペイン）
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
- 失敗時は `bash` へフォールバック
<<<<<<< ours
>>>>>>> theirs
=======
>>>>>>> theirs
=======
- GTK/X11安定化のため `GDK_DISABLE_SHM=1` / `NO_AT_BRIDGE=1` / `--no-dbus` を適用
- 失敗時は `bash` へフォールバック
>>>>>>> theirs
=======
- GTK/X11安定化のため `GDK_DISABLE_SHM=1` / `NO_AT_BRIDGE=1` / `--no-dbus` を適用
- 失敗時は `bash` へフォールバック
>>>>>>> theirs
=======
- GTK/X11安定化のため `GDK_DISABLE_SHM=1` / `NO_AT_BRIDGE=1` / `--no-dbus` を適用
- 失敗時は `bash` へフォールバック
>>>>>>> theirs
=======
- GTK/X11安定化のため `GDK_DISABLE_SHM=1` / `NO_AT_BRIDGE=1` / `--no-dbus` を適用
- 失敗時は `bash` へフォールバック
>>>>>>> theirs
=======
- GTK/X11安定化のため `GDK_DISABLE_SHM=1` / `NO_AT_BRIDGE=1` / `--no-dbus` を適用
- 失敗時は `bash` へフォールバック
>>>>>>> theirs
=======
- GTK/X11安定化のため `GDK_DISABLE_SHM=1` / `NO_AT_BRIDGE=1` / `--no-dbus` を適用
- 失敗時は `bash` へフォールバック
>>>>>>> theirs
=======
- GTK/X11安定化のため `GDK_DISABLE_SHM=1` / `NO_AT_BRIDGE=1` / `--no-dbus` を適用
- 各ペインは `ros_shell_init.sh` を経由して `devel/setup.bash` と ROS環境変数を自動設定
- 失敗時は `bash -lc /root/workspace/ros_shell_init.sh` へフォールバック
>>>>>>> theirs
=======
- Terminatorレイアウト `ros_8` を起動（8ペイン）
- GTK/X11安定化のため `GDK_DISABLE_SHM=1` / `NO_AT_BRIDGE=1` / `--no-dbus` を適用
- 各ペインは `ros_shell_init.sh` を経由して `devel/setup.bash` と ROS環境変数を自動設定
- 失敗時は `bash -lc /root/workspace/ros_shell_init.sh` へフォールバック
>>>>>>> theirs

---

## 5. コンテナ内でROSを起動

### 5-1. ワークスペースをビルド（初回または変更時）

```bash
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
cd /root/workspace/src/sound_send
=======
cd /root/workspace
>>>>>>> theirs
=======
cd /root/workspace
>>>>>>> theirs
=======
cd /root/workspace
>>>>>>> theirs
=======
cd /root/workspace
>>>>>>> theirs
=======
cd /root/workspace
>>>>>>> theirs
=======
cd /root/workspace
>>>>>>> theirs
=======
cd /root/workspace
>>>>>>> theirs
=======
cd /root/workspace
>>>>>>> theirs
=======
cd /root/workspace
>>>>>>> theirs
=======
cd /root/workspace
>>>>>>> theirs
=======
cd /root/workspace
>>>>>>> theirs
catkin_make
source /root/workspace/devel/setup.bash
```

### 5-2. 音声パイプライン起動

```bash
source /root/workspace/devel/setup.bash
roslaunch sound_send whisper.launch
```

必要に応じて引数を指定:

```bash
roslaunch sound_send whisper.launch mic_id:=f vol_thresh:=0.2
```

`whisper.launch` では `sound_trigger.py` と `whisper_node.py` が起動します。

---

<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
## 6. よく使う確認コマンド
=======
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
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
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs

ホスト側:

```bash
docker ps -a | grep ros_audio_container
docker logs ros_audio_container --tail 100
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
=======
docker network inspect ros_net
>>>>>>> theirs
=======
docker network inspect ros_net
>>>>>>> theirs
=======
docker network inspect ros_net
>>>>>>> theirs
=======
docker network inspect ros_net
>>>>>>> theirs
=======
docker network inspect ros_net
>>>>>>> theirs
=======
docker network inspect ros_net
>>>>>>> theirs
=======
docker network inspect ros_net
>>>>>>> theirs
=======
docker network inspect ros_net
>>>>>>> theirs
=======
docker network inspect ros_net
>>>>>>> theirs
=======
docker network inspect ros_net
>>>>>>> theirs
=======
docker network inspect ros_net
>>>>>>> theirs
```

コンテナ内:

```bash
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
=======
printenv | grep -E 'ROS_MASTER_URI|ROS_HOSTNAME|ROS_IP'
>>>>>>> theirs
=======
printenv | grep -E 'ROS_MASTER_URI|ROS_HOSTNAME|ROS_IP'
>>>>>>> theirs
=======
printenv | grep -E 'ROS_MASTER_URI|ROS_HOSTNAME|ROS_IP'
>>>>>>> theirs
=======
printenv | grep -E 'ROS_MASTER_URI|ROS_HOSTNAME|ROS_IP'
>>>>>>> theirs
=======
printenv | grep -E 'ROS_MASTER_URI|ROS_HOSTNAME|ROS_IP'
>>>>>>> theirs
=======
printenv | grep -E 'ROS_MASTER_URI|ROS_HOSTNAME|ROS_IP'
>>>>>>> theirs
=======
printenv | grep -E 'ROS_MASTER_URI|ROS_HOSTNAME|ROS_IP'
>>>>>>> theirs
=======
printenv | grep -E 'ROS_MASTER_URI|ROS_HOSTNAME|ROS_IP'
>>>>>>> theirs
=======
printenv | grep -E 'ROS_MASTER_URI|ROS_HOSTNAME|ROS_IP'
>>>>>>> theirs
=======
printenv | grep -E 'ROS_MASTER_URI|ROS_HOSTNAME|ROS_IP'
>>>>>>> theirs
=======
printenv | grep -E 'ROS_MASTER_URI|ROS_HOSTNAME|ROS_IP'
>>>>>>> theirs
rostopic list
rostopic echo /whisper_result
```

---

<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
## 7. 停止・削除
=======
## 8. 停止・削除
>>>>>>> theirs
=======
## 8. 停止・削除
>>>>>>> theirs
=======
## 8. 停止・削除
>>>>>>> theirs
=======
## 8. 停止・削除
>>>>>>> theirs
=======
## 8. 停止・削除
>>>>>>> theirs
=======
## 8. 停止・削除
>>>>>>> theirs
=======
## 8. 停止・削除
>>>>>>> theirs
=======
## 8. 停止・削除
>>>>>>> theirs
=======
## 8. 停止・削除
>>>>>>> theirs
=======
## 8. 停止・削除
>>>>>>> theirs
=======
## 8. 停止・削除
>>>>>>> theirs

```bash
cd sound_noetic
docker compose stop
# 完全に消す場合
docker compose down
```
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours

>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
