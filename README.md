# sound_ros_noetic

このリポジトリは、ROS Noetic上で「音声検知 → 録音 → Whisper文字起こし → ROSトピック配信」を行うための環境です。

## 1. いままでの問題（なぜ受信できない？）
以前は `network_mode: host` だったため、コンテナがホストネットワークに直接ぶら下がっていました。
この状態で別コンテナ（別 compose）とROS通信をすると、
- ROS Master の場所（`ROS_MASTER_URI`）
- 各ノードが相手に教える到達先（`ROS_IP` / `ROS_HOSTNAME`）
がホスト側経由の経路になりやすく、結果として購読側がパブリッシャに戻れないことがあります。

ROS1は「Masterに登録するだけ」でなく、**実データ通信はノード同士が直接接続**します。
そのため、ノードが広告するアドレスが相手から到達可能であることが重要です。

## 2. 改善後の仕組み（学生向けにやさしく説明）
今回の改善では、コンテナ同士を同じDockerブリッジネットワーク `ros_net` に接続するようにしました。

- `ros_audio_container` というホスト名を固定。
- `ROS_MASTER_URI=http://ros_audio_container:11311` を使う。
- `ROS_HOSTNAME=ros_audio_container` を使う。

これで、同じ `ros_net` に参加している別コンテナは
`ros_audio_container` を名前解決して直接アクセスできるため、
「ホストを一度経由しないと届かない」問題を回避しやすくなります。

## 3. Docker操作を3つのbashに分割
要望に合わせて、イメージ作成・コンテナ作成・起動を分離しました。

- `01_build_image.sh` : イメージ作成
- `02_create_container.sh` : コンテナ作成（起動しない）
- `03_start_container.sh` : コンテナ起動

## 4. 使い方
```bash
cd sound_noetic
bash env_setup.sh
bash 01_build_image.sh
bash 02_create_container.sh
bash 03_start_container.sh
```

コンテナに入る:
```bash
docker exec -it ros_audio_container bash
```

## 5. 別Docker（購読側）と接続するときのポイント
購読側の compose にも同じネットワーク名 `ros_net` を設定してください。

例（購読側 compose のイメージ）:
```yaml
networks:
  ros_net:
    external: true
    name: ros_net
```

購読側の環境変数は次を基本にします。
- `ROS_MASTER_URI=http://ros_audio_container:11311`
- `ROS_HOSTNAME=<購読側コンテナ名>`

> 重要: `ROS_HOSTNAME`（または`ROS_IP`）は、必ず相手コンテナから到達できる値にしてください。


## 6. 一度実行した場合に何が起こるか（実行シミュレーション）
ここでは、初回セットアップから「1回だけ」実行したときの挙動を時系列で説明します。

### Step A: `bash env_setup.sh`
1. 既存の `.env` があれば読み込みます。
2. `ROS_MASTER_URI` と `ROS_HOSTNAME` を質問されます（Enterでデフォルト採用）。
3. `.env` に保存されます。

この時点では**コンテナはまだ作られていません**。

### Step B: `bash 01_build_image.sh`
1. `docker compose build ros_audio` が実行されます。
2. `Dockerfile` から `ros_noetic_audio` イメージがビルドされます。

この時点でも**コンテナはまだ起動していません**（イメージだけ存在）。

### Step C: `bash 02_create_container.sh`
1. `docker compose create ros_audio` が実行されます。
2. `ros_audio_container` コンテナが「作成済み（停止状態）」になります。
3. このとき `ros_net` ネットワークが必要なら作成され、コンテナが接続されます。

### Step D: `bash 03_start_container.sh`
1. `docker compose start ros_audio` が実行されます。
2. 停止中だった `ros_audio_container` が起動します。
3. 以後、`docker exec -it ros_audio_container bash` で中に入れます。

### Step E: ROS通信の実際
1. コンテナ内で `roscore` とノード（音声トリガ・録音・Whisper）を起動。
2. ノードは `ROS_MASTER_URI=http://ros_audio_container:11311` を見てMaster登録。
3. その後の実データ通信は、同じ `ros_net` 内でノード同士が直接接続。

つまり「初回1回実行」では、
- 設定ファイル作成
- イメージ作成
- コンテナ作成
- コンテナ起動
までが分離され、どこで失敗したかを切り分けしやすくなります。

### よくある誤解
- `build` だけでは動かない（イメージを作るだけ）。
- `create` だけでも動かない（コンテナを作るだけ）。
- 実際に動くのは `start` の後。

## 7. 変更内容をGitHubに上げる方法（学生向け）
「コミット」と「GitHubへ反映（push）」は別操作です。以下の順番で行います。

```bash
# 1) 変更確認
git status

# 2) 変更をステージ
git add .

# 3) コミット（ローカル履歴に保存）
git commit -m "Docker運用手順を更新"

# 4) リモートへ送信（GitHubへ反映）
git push origin <ブランチ名>
```

### ポイント
- `git commit` しただけでは、**自分のPC（ローカル）に記録されるだけ**です。
- `git push` して初めて、GitHub（リモート）に反映されます。
- ブランチ名が分からない場合は `git branch --show-current` で確認できます。
