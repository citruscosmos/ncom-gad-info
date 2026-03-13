# OxTS NCOM/GAD ダッシュボード — 実装計画書（現行実装同期版）

## 1. 目的

OxTS の NCOM バイナリを可視化し、位置/姿勢/Heading/速度の状態監視、GAD 状態監視、CAN 状態監視、ならびに Kalman filter innovation 監視を行う。

本書は「将来計画」ではなく、**現在の実装状態と一致する設計書**として管理する。

## 2. 現在の実装スコープ（As-Is）

### 2.1 入力モード

- UDP リアルタイム受信（デフォルトポート `3000`）
- 録画ファイル再生（`.ncom` / `.bin` / `.dat`）
  - 再生速度切替（`0.5x / 1.0x / 2.0x / 5.0x`）
  - ループ再生
  - シーク開始位置指定（%）
  - 一時停止/再開

### 2.2 デコード対象

- NCOM Structure-A（72 byte 固定長）
- NavStatus `11`（Structure-B）は無視
- 累積チェックサム（1/2/3）検証
- decode 診断情報（known channel 判定、warning 集計）生成

### 2.3 ステータスチャネル対応

| Ch | 用途 | 実装状態 |
|---|---|---|
| 0 | GNSS mode / satellites | 実装済み |
| 1 | KF innovations set 1（pos/vel/pitch/heading） | 実装済み |
| 3 | Position accuracy | 実装済み |
| 4 | Velocity accuracy | 実装済み |
| 5 | Orientation accuracy | 実装済み |
| 32 | KF innovations set 2（zero-vel/no-slip/heading-lock/wspeed） | 実装済み |
| 78 | CAN bus status | 実装済み |
| 88 | KF innovations set 3（Byte0: vertical advanced slip innovation / Byte1-7: reserved） | 実装済み |
| 95 | GAD stream info | 実装済み |

## 3. 技術スタック

- Python 3.10+
- Streamlit
- pandas
- pydeck
- 標準ライブラリ（`socket`, `threading`, `struct`, `collections.deque`）

## 4. プロジェクト構成（現状）

```
ncom-gad-info/
├── app.py
├── ncom/
│   ├── constants.py
│   ├── decoder.py
│   └── status_channels.py
├── receiver/
│   ├── __init__.py
│   ├── udp_receiver.py
│   └── file_replay.py
└── tests/
    ├── test_decoder.py
    ├── test_playback_diagnostics.py
    └── fake_ncom_sender.py
```

## 5. データ処理アーキテクチャ

### 5.1 データフロー

1. UDP または Playback から 72-byte パケット供給
2. `NcomDecoder.decode_with_diagnostics()` でデコード
3. `StatusChannelDecoder` がチャネル別に更新
4. `apply_decoder_snapshot()` が `ReceiverState` に反映
5. `app.py` の `@st.fragment(run_every=1.5)` が描画更新

### 5.2 共有状態管理

- `ReceiverState` を `threading.Lock` で保護
- UI スレッドは lock 取得後にスナップショット読み取り
- 受信スレッド/再生スレッドは lock 取得後に書き込み

### 5.3 履歴バッファ

- 速度精度、位置精度、姿勢精度
- 速度現在値、姿勢現在値、位置現在値
- Innovation（set1）
- 診断バッファ（decode diagnostics）

すべて `deque(maxlen=...)` でリングバッファ化。

## 6. Innovation 取り扱い方針（現行）

### 6.1 共通デコード仕様

- Bit0: valid flag
- Bits1-7: 符号付き 7-bit 値
- スケール: `0.1 sigma`

### 6.2 主要表示ソース

- UI の「Position/Attitude/Heading/Speed」の Innovation 表示は **Ch1** を主ソースとする
  - Position: Byte 0-2
  - Speed: Byte 3-5
  - Pitch: Byte 6
  - Heading: Byte 7

### 6.3 詳細表示ソース

- Ch32: 補助 Innovation
- Ch88: set3（Byte0: vertical advanced slip innovation、Byte1-7: reserved）
- Ch95: GAD stream 監視用（innovation は GAD 側品質表示に利用）

## 7. UI 設計（現行）

### 7.1 メイン表示

- Status bar（Nav/GNSS/Packets/Errors/Source/Pause）
- Position（現在値、精度、Innovation）
- Attitude（現在値、精度、Innovation）
- Heading（現在値、精度、Innovation）
- Speed（現在値、精度、Innovation）
- Map（軌跡 + 現在位置 + 水平精度の破線円）
- Trend Graph（単一領域、カテゴリ/種別選択）

### 7.2 詳細表示（expander）

- GAD Stream Monitor（Ch95）
- KF Innovations Set 2（Ch32）
- KF Innovations Set 3（Ch88）
- Binary Diagnostics
- CAN Bus Status（Ch78）

### 7.3 サイドバー

- Input Source: UDP / Playback
- Playback ファイルアップロード
- Playback base rate / speed / loop / seek
- Binary diagnostics ON/OFF
- Start / Pause(Resume) / Stop

## 8. エラーハンドリング

- invalid length / sync mismatch / nav_status_11 は reject
- checksum 不一致は warning として採用継続
- unknown status channel は warning
- UDP bind/recv エラーは `last_error` に格納して UI 表示
- stale accuracy（age >= 150）は診断 warning と UI 注記

## 9. テスト設計（現行）

`tests/test_decoder.py` を中心に以下をカバー:

- int24 変換、checksum
- 正常パケット decode
- Ch3/4/5/78/95 decode
- Ch1/32/88 innovation decode
- nav_status=11 無視
- decode diagnostics（unknown channel / checksum warning / invalid length）
- replay packet 抽出
- `reset_receiver_state()` の初期化動作

## 10. 実装済み項目（完了）

以下は当初拡張項目だったが、現在は実装済み:

1. 位置精度・姿勢精度の表示（Ch3/5）
2. マップ表示（軌跡、現在位置、水平精度円）
3. NCOM 録画ファイル再生（seek/speed/loop/pause 含む）
4. UI 再編（カテゴリ別表示 + 単一グラフ領域）
5. Innovation 監視（Ch1/32/88 対応）

## 11. 今後の拡張対応（更新版）

### 11.1 近々対応（優先高）

1. **アラート閾値設定 UI**
   - 精度・innovation・CAN を閾値監視
   - sidebar から runtime 設定可能にする

2. **CSV/Parquet エクスポート**
   - 各履歴バッファ（current/accuracy/innovation）を保存
   - playback 解析結果の持ち出しを可能にする

### 11.2 中期対応

3. **Innovation 比較ビュー強化**
   - Ch1/32/88 を同一軸で比較
   - 条件別ハイライト（外れ値、連続悪化）

4. **Playback UX 強化**
   - フレーム単位 step 実行
   - 任意 timestamp ジャンプ
   - マーカー区間リプレイ

5. **診断レポート生成**
   - 受信品質、checksum 警告、stale 統計、CAN 品質のレポート化

### 11.3 長期対応

6. **マルチユーザー対応**
   - 受信スレッドをシングルトン化
   - viewer 側は pub/sub 配信購読

7. **大容量データ向け最適化**
   - 長時間 replay 時のメモリ/描画最適化
   - downsampling/aggregation の導入

## 12. 参考資料

- [NCOM Product Manual (Rev.250811)](https://www.oxts.com/software/navsuite/documentation/manuals/NCOM_man.pdf)
- [OxTS NCOMdecoder (C)](https://github.com/OxfordTechnicalSolutions/NCOMdecoder)
