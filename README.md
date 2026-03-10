# OxTS GAD Display

OxTS NCOMを可視化するStreamlitダッシュボードです。以下に対応します。

- 速度精度（Channel 4）
- 位置精度 / 姿勢精度（Channel 3 / 5）
- GAD状態（Channel 95）
- CAN状態（Channel 78）
- 現在地と軌跡のマップ表示
- UDPライブ受信 / 録画NCOMファイル再生
- Binary Diagnostics（UDP / Playback 両対応）

## セットアップ

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## 起動

```bash
streamlit run app.py
```

## テスト

```bash
pytest -q
```

## フェイク送信で確認

別ターミナルで以下を実行:

```bash
python tests/fake_ncom_sender.py --host 127.0.0.1 --port 3000 --hz 100
```

## Playbackモード

1. `streamlit run app.py` を起動
2. サイドバーで `Input Source` を `Playback` に変更
3. `.ncom` / `.bin` / `.dat` ファイルを選択
4. `Playback Binary Diagnostics`（デフォルトOFF）を必要に応じてON
5. `Seek position (%)` スライダーで再生開始位置を指定して `Start`
6. 画面内の `Playback Binary Diagnostics` セクションで以下を確認
   - `StatusCh` / `StatusDataHex`（StatusChannel生データ）
   - `Checksums`（batch 1/2/3 の検証結果）
   - `Warnings`（例: `checksum_warn`, `unknown_channel`, `stale_accuracy`）
7. `Stop` すると表示状態はリセットされる

### Binary Diagnosticsの挙動

- 観測モードです。警告が出ても再生は継続します。
- 診断バッファは直近200件のリングバッファです。
- `stale_accuracy` は Channel 3/4/5 の `age >= 150` で付与されます。
- UDP受信時も同じ診断表示を利用できます。

## Pause / Stop

- `Pause` で受信/再生を一時停止し、`Resume` で再開できます。
- `Stop` しても表示値はリセットされません（最後の状態を保持）。
