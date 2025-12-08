# ROSマルチマシン通信のための時刻同期ガイド (Chrony編)

ROSで複数のPC（例: 自分のPCとHSR）を連携させる際、**時刻同期**は非常に重要です。時刻がずれていると、TF（座標変換）エラーが発生したり、メッセージが破棄されたりします。
ここでは、Ubuntuで標準的に使われる `chrony` を用いた設定方法を解説します。

## 概要
*   **HSR (Server)**: 時刻の基準（NTPサーバー）となります。
*   **自分のPC (Client)**: HSRの時刻に合わせに行きます。

---

## 手順 1: インストール (両方のPC)
HSR側と自分のPC側の**両方**で、以下のコマンドを実行して `chrony` をインストールします。

```bash
sudo apt update
sudo apt install chrony
```

## 手順 2: HSR側 (サーバー) の設定
HSRが「時刻の基準」となるように設定します。

1.  設定ファイルを開きます。
    ```bash
    sudo nano /etc/chrony/chrony.conf
    ```
2.  以下の行を追加（またはコメントアウト解除）して、ローカルネットワークからのアクセスを許可します。
    ```conf
    # 自分のPCのIPアドレスが含まれる範囲を指定 (例: 192.168.1.0/24)
    allow 192.168.1.0/24
    ```
3.  設定を反映させるために再起動します。
    ```bash
    sudo systemctl restart chrony
    ```

## 手順 3: 自分のPC側 (クライアント) の設定
自分のPCがHSRの時刻を見に行くように設定します。

1.  設定ファイルを開きます。
    ```bash
    sudo nano /etc/chrony/chrony.conf
    ```
2.  既存の `pool ...` や `server ...` の行をコメントアウトし、**HSRのIPアドレス**を指定します。
    ```conf
    # HSRのIPを指定 (iburstをつけると起動時に素早く同期します)
    server 192.168.1.50 minpoll 0 maxpoll 5 iburst
    ```
3.  設定を反映させます。
    ```bash
    sudo systemctl restart chrony
    ```

## 手順 4: 同期の確認
自分のPC側で以下のコマンドを実行し、同期状態を確認します。

```bash
chronyc sources
```

*   出力結果の左側に `*` (アスタリスク) がついていれば、同期完了です。
    *   `*`: 現在同期中
    *   `+`: 同期候補
    *   `?`: 接続不可（ファイアウォールなどを確認してください）

```bash
# 詳細なズレを確認
chronyc tracking
```

これで、ROSの通信が安定して行えるようになります。
