### ポイント
- `git commit` しただけでは、**自分のPC（ローカル）に記録されるだけ**です。
- `git push` して初めて、GitHub（リモート）に反映されます。
- ブランチ名が分からない場合は `git branch --show-current` で確認できます。
=======
# sound_ros_noetic 利用手順

このリポジトリは、ROS Noetic で
**音声トリガ検出 → 録音 → Whisper文字起こし → ROSトピック配信**
を行うための実行環境です。

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
- `ROS_MASTER_URI`
- `ROS_IP`
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

または直接:

```bash
docker exec -it ros_audio_container bash
```

---

## 5. コンテナ内でROSを起動

### 5-1. ワークスペースをビルド（初回または変更時）

```bash
cd /root/workspace/src/sound_send
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

## 6. よく使う確認コマンド

ホスト側:

```bash
docker ps -a | grep ros_audio_container
docker logs ros_audio_container --tail 100
```

コンテナ内:

```bash
rostopic list
rostopic echo /whisper_result
```

---

## 7. 停止・削除

```bash
cd sound_noetic
docker compose stop
# 完全に消す場合
docker compose down
```

>>>>>>> theirs
