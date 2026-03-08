# OxTS GAD Display

OxTS NCOMをUDPで受信し、速度精度・GAD・CAN状態を表示するStreamlitダッシュボードです。

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
