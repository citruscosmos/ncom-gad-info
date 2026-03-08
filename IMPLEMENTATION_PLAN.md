# OxTS GAD監視リアルタイムダッシュボード — 実装計画書

## 1. 概要と目的

OxTS製GNSS/INS（xNAV, RT3000等）が出力するNCOMバイナリパケットをUDP（デフォルト:ポート3000）でリアルタイム受信し、
以下をStreamlitダッシュボード上に可視化するPythonアプリケーションを作成する。

- **速度推定精度（Velocity Accuracy）** の数値表示＋時系列グラフ
- **GAD（Generic Aiding Data）** のストリーム監視（rejection count、innovation）
- **CAN バスステータス** （送受信エラー率、エラーカウント）
- **GNSS モード / ナビゲーション状態** の常時表示

### 主なユースケース

GAD入力（例：ROSノードからの車速）が遮断された際の精度劣化を、
リアルタイムで視覚的に捉え、デバッグ・チューニングに活用する。

### 参考仕様書

- [NCOM Product Manual (Rev.250811)](https://www.oxts.com/software/navsuite/documentation/manuals/NCOM_man.pdf)
- [OxTS NCOMdecoder (C版, GitHub)](https://github.com/OxfordTechnicalSolutions/NCOMdecoder)
- [OxTS GAD-SDK Documentation](https://oxfordtechnicalsolutions.github.io/)

---

## 2. 技術スタック

| レイヤー | 技術 | 備考 |
|---------|------|------|
| 言語 | Python 3.10+ | f-string, match文、型ヒント活用 |
| NCOMデコード | **独自実装（Pure Python）** | ※PyPIに「oxts-sdk」は存在しない。公式NCOM仕様書 + C版NCOMdecoderを参考に実装 |
| バイナリ解析 | `struct` (標準ライブラリ) | LE (Little-Endian) のunpackに使用 |
| UDP受信 | `socket` + `threading.Thread` | デーモンスレッドで非同期受信 |
| スレッド間通信 | `threading.Lock` + 共有dict | session_stateへの橋渡し |
| フロントエンド | Streamlit >= 1.33 | `@st.fragment` による部分再レンダリング |
| グラフ | `st.line_chart` (Altair内蔵) | pandas DataFrame ベースの時系列プロット |
| 時系列バッファ | `collections.deque(maxlen=N)` | 過去60秒分のリングバッファ |

### 依存パッケージ (requirements.txt)

```
streamlit>=1.33
pandas>=2.0
```

> **注意:** `oxts-sdk`, `oxts`, `bitstring` 等の外部パッケージは使用しない。
> NCOMデコーダは `struct` モジュールのみで純粋に実装する。

---

## 3. プロジェクト構成

```
oxts-gad-display/
├── app.py                    # Streamlit エントリーポイント（UIレイアウト + 更新ループ）
├── ncom/
│   ├── __init__.py
│   ├── decoder.py            # NCOMパケットデコーダ本体
│   ├── constants.py          # NCOMパケット定数・変換係数
│   └── status_channels.py    # ステータスチャネル別デコーダ（Ch0, Ch4, Ch78, Ch95 等）
├── receiver/
│   ├── __init__.py
│   └── udp_receiver.py       # UDP受信バックグラウンドスレッド
├── tests/
│   ├── fake_ncom_sender.py   # テスト用ダミーNCOMパケット送信スクリプト
│   └── test_decoder.py       # ユニットテスト
├── requirements.txt
└── README.md
```

---

## 4. NCOMパケット仕様（デコーダ実装の基礎）

> **出典: NCOM Product Manual Rev.250811**

### 4.1 パケット構造（72バイト固定長）— NCOM Structure-A

パケットはStructure-A（顧客用、NavStatus ≠ 11）とStructure-B（OxTS内部用、NavStatus = 11）がある。
**NavStatus == 11 のパケットは無視すること。**

```
Offset  Length  Format    名前              説明
------  ------  ------    ----              ----
0       1       UByte     Sync              同期バイト。常に 0xE7
1       2       UShort    Time              GPS分内のミリ秒 [0-59999]
3       3       Word      AccX              X軸加速度 × 1e-4 → m/s²
6       3       Word      AccY              Y軸加速度 × 1e-4 → m/s²
9       3       Word      AccZ              Z軸加速度 × 1e-4 → m/s²
12      3       Word      AngRateX          X軸角速度 × 1e-5 → rad/s
15      3       Word      AngRateY          Y軸角速度 × 1e-5 → rad/s
18      3       Word      AngRateZ          Z軸角速度 × 1e-5 → rad/s
21      1       UByte     NavStatus         ナビゲーション状態 (Table 5)
22      1       UByte     Checksum1         bytes[1:22] の合計 & 0xFF
23      8       Double    Latitude          緯度 [rad]
31      8       Double    Longitude         経度 [rad]
39      4       Float     Altitude          高度 [m]
43      3       Word      VelNorth          北方向速度 × 1e-4 → m/s
46      3       Word      VelEast           東方向速度 × 1e-4 → m/s
49      3       Word      VelDown           下方向速度 × 1e-4 → m/s
52      3       Word      Heading           ヘディング × 1e-6 → rad (±π)
55      3       Word      Pitch             ピッチ × 1e-6 → rad (±π/2)
58      3       Word      Roll              ロール × 1e-6 → rad (±π)
61      1       UByte     Checksum2         bytes[1:61] の合計 & 0xFF ★累積
62      1       UByte     StatusChannel     ステータスチャネル番号
63      8       (varies)  StatusData        ステータスデータ（チャネル依存）
71      1       UByte     Checksum3         bytes[1:71] の合計 & 0xFF ★累積
```

> **重要: チェックサムは累積式**
> - Checksum1 = sum(bytes[1:22]) & 0xFF
> - Checksum2 = sum(bytes[1:61]) & 0xFF  (Checksum1の範囲を含む！)
> - Checksum3 = sum(bytes[1:71]) & 0xFF  (全バイトを含む！)

### 4.2 変換係数定数 (constants.py)

NCOM仕様書 Table 1, 4, 6 より：

```python
NCOM_SYNC = 0xE7
NCOM_PACKET_LENGTH = 72

# Batch A 変換係数 (Table 4)
ACC2MPS2 = 1e-4      # 加速度: 0.1 mm/s² 単位
RATE2RPS = 1e-5      # 角速度: 0.01 mrad/s 単位

# Batch B 変換係数 (Table 6)
VEL2MPS = 1e-4       # 速度: 0.1 mm/s 単位
ANG2RAD = 1e-6       # 角度: 0.001 mrad 単位

# 速度精度 (Channel 4, Table 13)
# UShort × 1mm/s → m/s への変換
VEL_ACC_SCALE = 1e-3  # 1 mm/s 単位

# 位置精度 (Channel 3, Table 12)
POS_ACC_SCALE = 1e-3  # 1 mm 単位

# 姿勢精度 (Channel 5, Table 15)
ORI_ACC_SCALE = 1e-5  # 0.01 mrad 単位

# 精度チャネルの Age 閾値
ACCURACY_AGE_INVALID = 150  # Age >= 150 → データが古すぎて無効

# NavStatus 値マッピング (Table 5)
NAV_STATUS = {
    0: "Invalid",
    1: "Raw IMU",
    2: "Initialising",
    3: "Locking",
    4: "Real-Time",           # ← 通常運用時の値
    5: "Unlocked",
    6: "Firmware Expired",
    7: "Firmware Blocked",
    10: "Status Only",
    11: "Internal (Structure-B)",  # ← 無視すべき
    20: "Trigger (Init)",
    21: "Trigger (Locking)",
    22: "Trigger (Real-Time)",
}

# GNSS Position/Velocity モード (Table 9)
GNSS_MODE = {
    0: "None", 1: "Search", 2: "Doppler", 3: "SPS",
    4: "Differential", 5: "RTK Float", 6: "RTK Integer",
    7: "WAAS", 8: "OmniSTAR", 9: "OmniSTAR HP",
    10: "No data", 11: "Blanked",
    17: "OmniSTAR XP", 18: "CDGPS",
    20: "gxDoppler", 21: "gxSPS", 22: "gxDifferential",
    23: "gxFloat", 24: "gxInteger",
    25: "ixDoppler", 26: "ixSPS", 27: "ixDifferential",
    28: "ixFloat", 29: "ixInteger",
    30: "PPP converging", 31: "PPP",
}
```

### 4.3 int24 (3バイト符号付き整数 = "Word") のデコード

NCOM仕様書ではこの型を "Word" (signed) / "UWord" (unsigned) と呼ぶ。
`struct` モジュールでは直接サポートされないため、ヘルパーを実装する：

```python
def decode_int24_le(data: bytes) -> int:
    """3バイト LE 符号付き整数 (Word) をデコード"""
    value = int.from_bytes(data[:3], byteorder='little', signed=False)
    if value >= 0x800000:
        value -= 0x1000000
    return value

def decode_uint24_le(data: bytes) -> int:
    """3バイト LE 符号なし整数 (UWord) をデコード"""
    return int.from_bytes(data[:3], byteorder='little', signed=False)
```

### 4.4 チェックサム検証

NCOM仕様書 p.57 "Checksum definition" より：
**チェックサムは累積式** — Checksum2 は bytes[1:61] 全体の合計（Checksum1の範囲も含む）。

```python
def verify_checksum(packet: bytes, batch: int) -> bool:
    """
    NCOM チェックサムを検証する。
    batch=1: Checksum1 (byte 22) = sum(bytes[1:22]) & 0xFF
    batch=2: Checksum2 (byte 61) = sum(bytes[1:61]) & 0xFF  ★累積
    batch=3: Checksum3 (byte 71) = sum(bytes[1:71]) & 0xFF  ★累積
    """
    RANGES = {1: (1, 22), 2: (1, 61), 3: (1, 71)}
    POSITIONS = {1: 22, 2: 61, 3: 71}
    start, end = RANGES[batch]
    expected = sum(packet[start:end]) & 0xFF
    actual = packet[POSITIONS[batch]]
    return expected == actual
```

---

## 5. ステータスチャネルデコーダ (status_channels.py)

NCOMパケットの Batch S（bytes 63-70）は、パケットごとに異なるステータスチャネル番号 (byte 62) を持つ。
100Hzで出力されるNCOMパケットで約200パケットごとに全チャネルが一巡する（約2秒で1サイクル）。

### 5.1 本プロジェクトで使用するステータスチャネル一覧

| Ch | 名前 | 目的 | NCOM仕様書参照 |
|----|------|------|--------------|
| **0** | Full time / GNSS mode | GNSS Position/Velocity/Orientation モード表示 | Table 8 |
| **3** | Position accuracy | 位置精度（将来拡張用に対応しておく） | Table 12 |
| **4** | Velocity accuracy | **速度精度** (North/East/Down) — メイン表示対象 | Table 13 |
| **5** | Orientation accuracy | 姿勢精度（将来拡張用に対応しておく） | Table 15 |
| **78** | CAN bus status | CAN送受信数、エラー数、成功率 | Table 73 |
| **95** | Generic aiding packet info | **GADストリームの監視** (stream ID, rejection count, innovations) | Table 83 |

### 5.2 Channel 0 — Full time / GNSS mode (Table 8)

```
Byte  Format   Definition                              Invalid when
0-3   Long     GPS分（1980-01-06 00:00 からの分数）    Value < 1000
4     UByte    衛星追尾数                               Value = 255
5     UByte    GNSS Position mode (Table 9)             Value = 255
6     UByte    GNSS Velocity mode (Table 9)             Value = 255
7     UByte    Dual antenna orientation mode (Table 9)  Value = 255
```

**実装ポイント:**
- Channel 0 は GNSS mode をそのまま数値として保持し、既知値のみ辞書で文字列化する
- 未知値は `Unknown(<code>)` として表示し、拡張値を仮定しない
- GAD の Active/Inactive 判定は Channel 95（stream_id/rejection/innovation）を主判定とする

### 5.3 Channel 4 — Velocity Accuracy (Table 13) ★主要

```
Byte  Format   Definition                    Valid when
0-1   UShort   North velocity accuracy        Age < 150
2-3   UShort   East velocity accuracy         Age < 150
4-5   UShort   Down velocity accuracy         Age < 150
6     UByte    Age (パケット経過カウンタ)     Always
7     UByte    Blended processing method      Always
```

**変換式（仕様書注記より）:**
```
velocity_accuracy_mps = raw_uint16 × 0.001   (単位: 1 mm/s)
```

**Age フィールドの意味:**
- Age はステータスチャネルが最後に更新されてからの経過パケット数
- Age < 150 (= NCOM_COUNT_TOO_OLD) の時のみデータは有効
- 100Hz出力なので Age 150 ≈ 1.5秒前のデータ

**Blended processing method (byte 7, Table 14):**
- 0: Invalid
- 1: Real-time
- 2: Simulated
- 3: Post-process forward
- 4: Post-process backward
- 5: Post-process combined

### 5.4 Channel 3 — Position Accuracy (Table 12)

```
Byte  Format   Definition                    Valid when
0-1   UShort   North position accuracy        Age < 150
2-3   UShort   East position accuracy         Age < 150
4-5   UShort   Down position accuracy         Age < 150
6     UByte    Age                            Always
7     UByte    ABD robot UMAC interface        Value ≠ 0xFF
```

**変換式:** `position_accuracy_m = raw_uint16 × 0.001` (単位: 1 mm)

### 5.5 Channel 5 — Orientation Accuracy (Table 15)

```
Byte  Format   Definition                    Valid when
0-1   UShort   Heading accuracy               Age < 150
2-3   UShort   Pitch accuracy                 Age < 150
4-5   UShort   Roll accuracy                  Age < 150
6     UByte    Age                            Always
7              Reserved
```

**変換式:** `orientation_accuracy_rad = raw_uint16 × 1e-5` (単位: 0.01 mrad)

### 5.6 Channel 78 — CAN Bus Status (Table 73) ★ハードウェア監視

```
Byte  Format   Definition                          Valid when
0-1   UShort   CAN messages transmitted              Always (cyclic counter)
2-3   UShort   CAN messages received                 Always (cyclic counter)
4     UByte    CAN transmit percent OK               Value ≠ 0xFF
5     UByte    CAN receive percent OK                Value ≠ 0xFF
6     UByte    CAN number of errors                  Always (cyclic counter)
7     UByte    CAN last error code                   Always
```

**実装ポイント:**
- `transmit_percent_ok` < 100 または `receive_percent_ok` < 100 → 通信品質低下の警告
- `error_count` が増加し続ける → CAN バスエラーの警告
- カウンタは cyclic（オーバーフロー後にラップ）

### 5.7 Channel 95 — Generic Aiding Packet Information (Table 83) ★GAD監視

```
Byte  Format  Definition                                        Valid when
0     UByte   GAD packet stream Id                               Value ≠ 0
1     UByte   Number of consecutive updates rejected              Value ≠ 0xFF
2     Byte    Bits 1-7: innovation 1 (0.1σ単位)                  Bit 0 = 1
3     Byte    Bits 1-7: innovation 2 (0.1σ単位)                  Bit 0 = 1
4     Byte    Bits 1-7: innovation 3 (0.1σ単位)                  Bit 0 = 1
5-6   UShort  Milliseconds into current GPS minute               Value < 60000
7              Reserved
```

**Innovation のデコード方法:**
```python
def decode_innovation(raw_byte: int) -> tuple[bool, float]:
    """
    Innovation バイトをデコードする。
    Returns: (is_valid, innovation_sigma)
    innovation は 0.1σ 単位の符号付き値。
    """
    is_valid = bool(raw_byte & 0x01)
    raw_value = raw_byte >> 1  # bits 1-7 (7ビット符号付き)
    if raw_value >= 64:        # 7ビット 2の補数
        raw_value -= 128
    innovation_sigma = raw_value * 0.1
    return is_valid, innovation_sigma
```

**GADストリーム状態の推定ロジック:**

Channel 95 には明示的な "Active/Timeout" フラグは存在しない。
以下のロジックで GAD ストリームの状態を推定する：

```python
@dataclass
class GadStreamInfo:
    stream_id: int
    last_seen: float            # time.monotonic()
    rejection_count: int        # 連続リジェクト数
    innovations: list[float]    # 最大3つの innovation (σ)
    timestamp_ms: int           # GADパケットのGPS分内ミリ秒

    @property
    def status(self) -> str:
        age = time.monotonic() - self.last_seen
        if age > 5.0:
            return "Timeout"           # 5秒以上 Ch95 で報告されない
        if self.rejection_count > 10:
            return "Rejected"          # 連続10回以上リジェクト
        if self.rejection_count > 0:
            return "Partially Rejected"
        return "Active"

    @property
    def innovation_quality(self) -> str:
        """Innovation が 1.0σ 以下なら Good, 超えたら Poor"""
        valid_inns = [abs(i) for i in self.innovations if i is not None]
        if not valid_inns:
            return "N/A"
        max_inn = max(valid_inns)
        if max_inn <= 1.0:
            return "Good"
        elif max_inn <= 3.5:
            return "Fair"
        return "Poor"
```

**補助的な判定 (Channel 0 との組み合わせ):**

Channel 0 の mode はファームウェアや将来拡張で未定義値が現れる可能性があるため、
確証のある mode 値以外は `Unknown(<code>)` として扱う。
GAD 使用状況は Channel 95 を主判定とし、Channel 0 は補助表示に留める。

### 5.8 StatusChannelDecoder 実装

```python
import struct
import time
from dataclasses import dataclass, field

class StatusChannelDecoder:
    """ステータスチャネルのディスパッチャー"""

    def __init__(self):
        # Channel 4: 速度精度
        self.velocity_accuracy = {"north": None, "east": None, "down": None}  # m/s
        self.velocity_accuracy_age = 255

        # Channel 3: 位置精度
        self.position_accuracy = {"north": None, "east": None, "down": None}  # m
        self.position_accuracy_age = 255

        # Channel 5: 姿勢精度
        self.orientation_accuracy = {"heading": None, "pitch": None, "roll": None}  # rad

        # Channel 0: GNSS モード
        self.gnss_pos_mode: int | None = None
        self.gnss_vel_mode: int | None = None
        self.gnss_att_mode: int | None = None
        self.num_satellites: int | None = None

        # Channel 78: CAN バスステータス
        self.can_status = {
            "tx_count": 0, "rx_count": 0,
            "tx_ok_pct": None, "rx_ok_pct": None,
            "error_count": 0, "last_error_code": 0,
        }

        # Channel 95: GAD ストリーム情報
        self.gad_streams: dict[int, GadStreamInfo] = {}

    def decode(self, channel: int, data: bytes):
        handler = self._handlers.get(channel)
        if handler:
            handler(self, data)

    def _decode_ch0_gnss_mode(self, data: bytes):
        """Channel 0: Full time, GNSS mode (Table 8)"""
        num_sats = data[4]
        pos_mode = data[5]
        vel_mode = data[6]
        att_mode = data[7]
        self.num_satellites = num_sats if num_sats != 255 else None
        self.gnss_pos_mode = pos_mode if pos_mode != 255 else None
        self.gnss_vel_mode = vel_mode if vel_mode != 255 else None
        self.gnss_att_mode = att_mode if att_mode != 255 else None

    def _decode_ch3_position_accuracy(self, data: bytes):
        """Channel 3: Position accuracy (Table 12)"""
        age = data[6]
        if age < ACCURACY_AGE_INVALID:
            self.position_accuracy["north"] = struct.unpack_from('<H', data, 0)[0] * POS_ACC_SCALE
            self.position_accuracy["east"]  = struct.unpack_from('<H', data, 2)[0] * POS_ACC_SCALE
            self.position_accuracy["down"]  = struct.unpack_from('<H', data, 4)[0] * POS_ACC_SCALE
        self.position_accuracy_age = age

    def _decode_ch4_velocity_accuracy(self, data: bytes):
        """Channel 4: Velocity accuracy (Table 13)"""
        age = data[6]
        if age < ACCURACY_AGE_INVALID:
            self.velocity_accuracy["north"] = struct.unpack_from('<H', data, 0)[0] * VEL_ACC_SCALE
            self.velocity_accuracy["east"]  = struct.unpack_from('<H', data, 2)[0] * VEL_ACC_SCALE
            self.velocity_accuracy["down"]  = struct.unpack_from('<H', data, 4)[0] * VEL_ACC_SCALE
        self.velocity_accuracy_age = age

    def _decode_ch5_orientation_accuracy(self, data: bytes):
        """Channel 5: Orientation accuracy (Table 15)"""
        age = data[6]
        if age < ACCURACY_AGE_INVALID:
            self.orientation_accuracy["heading"] = struct.unpack_from('<H', data, 0)[0] * ORI_ACC_SCALE
            self.orientation_accuracy["pitch"]   = struct.unpack_from('<H', data, 2)[0] * ORI_ACC_SCALE
            self.orientation_accuracy["roll"]    = struct.unpack_from('<H', data, 4)[0] * ORI_ACC_SCALE

    def _decode_ch78_can_status(self, data: bytes):
        """Channel 78: CAN bus status (Table 73)"""
        self.can_status["tx_count"]       = struct.unpack_from('<H', data, 0)[0]
        self.can_status["rx_count"]       = struct.unpack_from('<H', data, 2)[0]
        self.can_status["tx_ok_pct"]      = data[4] if data[4] != 0xFF else None
        self.can_status["rx_ok_pct"]      = data[5] if data[5] != 0xFF else None
        self.can_status["error_count"]    = data[6]
        self.can_status["last_error_code"] = data[7]

    def _decode_ch95_gad_info(self, data: bytes):
        """Channel 95: Generic aiding packet information (Table 83)"""
        stream_id = data[0]
        if stream_id == 0:
            return  # Invalid

        rejection_count = data[1] if data[1] != 0xFF else 0
        inn1_valid, inn1 = decode_innovation(data[2])
        inn2_valid, inn2 = decode_innovation(data[3])
        inn3_valid, inn3 = decode_innovation(data[4])
        innovations = [
            inn1 if inn1_valid else None,
            inn2 if inn2_valid else None,
            inn3 if inn3_valid else None,
        ]
        timestamp_ms = struct.unpack_from('<H', data, 5)[0]

        self.gad_streams[stream_id] = GadStreamInfo(
            stream_id=stream_id,
            last_seen=time.monotonic(),
            rejection_count=rejection_count,
            innovations=innovations,
            timestamp_ms=timestamp_ms if timestamp_ms < 60000 else 0,
        )

    _handlers = {
        0:  _decode_ch0_gnss_mode,
        3:  _decode_ch3_position_accuracy,
        4:  _decode_ch4_velocity_accuracy,
        5:  _decode_ch5_orientation_accuracy,
        78: _decode_ch78_can_status,
        95: _decode_ch95_gad_info,
    }
```

---

## 6. NCOMメインデコーダ (decoder.py)

```python
from dataclasses import dataclass

@dataclass
class NcomPacket:
    """デコード済みNCOMパケットの構造体"""
    timestamp_ms: int
    nav_status: int
    nav_status_name: str

    # Batch A: IMU
    acc_x: float   # m/s²
    acc_y: float
    acc_z: float
    ang_rate_x: float  # rad/s
    ang_rate_y: float
    ang_rate_z: float

    # Batch B: Navigation
    latitude: float    # rad
    longitude: float   # rad
    altitude: float    # m
    vel_north: float   # m/s
    vel_east: float    # m/s
    vel_down: float    # m/s
    heading: float     # rad
    pitch: float       # rad
    roll: float        # rad

    # Batch S: Status
    status_channel: int
    status_data: bytes  # 8バイト生データ

    # チェックサム検証結果
    checksum_valid: tuple[bool, bool, bool]  # (batch_a, batch_b, batch_s)


class NcomDecoder:
    """
    NCOMバイナリパケットデコーダ。

    使い方:
        decoder = NcomDecoder()
        packet = decoder.decode(raw_bytes)
        if packet is not None:
            print(packet.vel_north)
    """

    def __init__(self):
        self.status_decoder = StatusChannelDecoder()
        self._packet_count = 0
        self._error_count = 0

    def decode(self, data: bytes) -> NcomPacket | None:
        """72バイトのNCOMパケットをデコードする。失敗時はNoneを返す。"""
        if len(data) != NCOM_PACKET_LENGTH:
            self._error_count += 1
            return None
        if data[0] != NCOM_SYNC:
            self._error_count += 1
            return None

        # NavStatus == 11 (Structure-B) は無視
        nav_status = data[21]
        if nav_status == 11:
            return None

        # チェックサム検証（累積式）
        cs_valid = (
            verify_checksum(data, 1),
            verify_checksum(data, 2),
            verify_checksum(data, 3),
        )

        # Batch A (Table 4)
        timestamp_ms = struct.unpack_from('<H', data, 1)[0]
        acc_x = decode_int24_le(data[3:6]) * ACC2MPS2
        acc_y = decode_int24_le(data[6:9]) * ACC2MPS2
        acc_z = decode_int24_le(data[9:12]) * ACC2MPS2
        ang_x = decode_int24_le(data[12:15]) * RATE2RPS
        ang_y = decode_int24_le(data[15:18]) * RATE2RPS
        ang_z = decode_int24_le(data[18:21]) * RATE2RPS

        # Batch B (Table 6)
        lat = struct.unpack_from('<d', data, 23)[0]
        lon = struct.unpack_from('<d', data, 31)[0]
        alt = struct.unpack_from('<f', data, 39)[0]
        vel_n = decode_int24_le(data[43:46]) * VEL2MPS
        vel_e = decode_int24_le(data[46:49]) * VEL2MPS
        vel_d = decode_int24_le(data[49:52]) * VEL2MPS
        heading = decode_int24_le(data[52:55]) * ANG2RAD
        pitch   = decode_int24_le(data[55:58]) * ANG2RAD
        roll    = decode_int24_le(data[58:61]) * ANG2RAD

        # Batch S
        status_ch = data[62]
        status_data = data[63:71]

        # ステータスチャネルデコード
        self.status_decoder.decode(status_ch, status_data)

        self._packet_count += 1

        return NcomPacket(
            timestamp_ms=timestamp_ms,
            nav_status=nav_status,
            nav_status_name=NAV_STATUS.get(nav_status, f"Unknown({nav_status})"),
            acc_x=acc_x, acc_y=acc_y, acc_z=acc_z,
            ang_rate_x=ang_x, ang_rate_y=ang_y, ang_rate_z=ang_z,
            latitude=lat, longitude=lon, altitude=alt,
            vel_north=vel_n, vel_east=vel_e, vel_down=vel_d,
            heading=heading, pitch=pitch, roll=roll,
            status_channel=status_ch,
            status_data=status_data,
            checksum_valid=cs_valid,
        )
```

---

## 7. UDP受信バックグラウンドスレッド (udp_receiver.py)

### 7.1 設計方針

- `threading.Thread` のデーモンスレッドとして実行
- ソケットの `settimeout()` で定期的にループ制御（停止要求への応答性確保）
- デコード済みデータを `threading.Lock` で保護された共有辞書に格納
- Streamlit側は Lock を取得して最新スナップショットを読み取る

### 7.2 ReceiverState — スレッド間共有データストア

```python
import socket
import threading
import time
from collections import deque
from dataclasses import dataclass, field

@dataclass
class ReceiverState:
    """スレッド間で共有されるデータストア（Lock保護下でアクセス）"""
    lock: threading.Lock = field(default_factory=threading.Lock)

    # --- Channel 4: 速度精度 ---
    vel_acc_north: float | None = None  # m/s
    vel_acc_east: float | None = None
    vel_acc_down: float | None = None
    vel_acc_age: int = 255

    # 速度精度の時系列バッファ（過去60秒分）
    # Channel 4 は約2秒に1回更新されるため、maxlen=30で60秒分
    # 各要素: (timestamp_sec: float, north: float, east: float, down: float)
    accuracy_history: deque = field(default_factory=lambda: deque(maxlen=60))

    # --- Channel 0: GNSS モード ---
    gnss_pos_mode: int | None = None
    gnss_vel_mode: int | None = None
    gnss_pos_mode_name: str = "Unknown"
    gnss_vel_mode_name: str = "Unknown"
    num_satellites: int | None = None

    # --- Channel 95: GAD ストリーム ---
    gad_streams: dict = field(default_factory=dict)  # {stream_id: GadStreamInfo}

    # --- Channel 78: CAN ステータス ---
    can_status: dict = field(default_factory=dict)

    # --- ナビゲーション状態 ---
    nav_status: str = "Unknown"
    nav_status_code: int = 0

    # --- 統計 ---
    packet_count: int = 0
    error_count: int = 0
    last_packet_time: float = 0.0

    # --- 制御 ---
    is_running: bool = False
```

### 7.3 NcomUdpReceiver — 受信スレッド

```python
class NcomUdpReceiver(threading.Thread):
    """NCOMパケットをUDPで受信しデコードするバックグラウンドスレッド"""

    def __init__(self, state: ReceiverState, host: str = "", port: int = 3000):
        super().__init__(daemon=True)
        self.state = state
        self.host = host
        self.port = port
        self._stop_event = threading.Event()
        self._decoder = NcomDecoder()

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.host, self.port))
        sock.settimeout(1.0)

        with self.state.lock:
            self.state.is_running = True

        try:
            while not self._stop_event.is_set():
                try:
                    data, addr = sock.recvfrom(256)
                except socket.timeout:
                    continue

                packet = self._decoder.decode(data)
                if packet is None:
                    with self.state.lock:
                        self.state.error_count += 1
                    continue

                now = time.monotonic()
                sd = self._decoder.status_decoder

                with self.state.lock:
                    self.state.nav_status = packet.nav_status_name
                    self.state.nav_status_code = packet.nav_status
                    self.state.packet_count += 1
                    self.state.last_packet_time = now

                    # Channel 4: 速度精度 (新しいデータがあればコピー)
                    va = sd.velocity_accuracy
                    if va["north"] is not None:
                        prev_n = self.state.vel_acc_north
                        self.state.vel_acc_north = va["north"]
                        self.state.vel_acc_east = va["east"]
                        self.state.vel_acc_down = va["down"]
                        self.state.vel_acc_age = sd.velocity_accuracy_age

                        # 値が変わった時のみ時系列に追加（Ch4は~2秒に1回更新）
                        if prev_n != va["north"]:
                            self.state.accuracy_history.append((
                                now, va["north"], va["east"], va["down"],
                            ))

                    # Channel 0: GNSS モード
                    if sd.gnss_pos_mode is not None:
                        self.state.gnss_pos_mode = sd.gnss_pos_mode
                        self.state.gnss_vel_mode = sd.gnss_vel_mode
                        self.state.gnss_pos_mode_name = GNSS_MODE.get(sd.gnss_pos_mode, f"Unknown({sd.gnss_pos_mode})")
                        self.state.gnss_vel_mode_name = GNSS_MODE.get(sd.gnss_vel_mode, f"Unknown({sd.gnss_vel_mode})")
                        self.state.num_satellites = sd.num_satellites

                    # Channel 95: GAD ストリーム
                    if sd.gad_streams:
                        self.state.gad_streams = {
                            k: v for k, v in sd.gad_streams.items()
                        }

                    # Channel 78: CAN ステータス
                    if sd.can_status.get("tx_ok_pct") is not None:
                        self.state.can_status = dict(sd.can_status)
        finally:
            sock.close()
            with self.state.lock:
                self.state.is_running = False

    def stop(self):
        self._stop_event.set()
```

### 7.4 スレッド安全性のポイント

| 問題 | 対策 |
|------|------|
| StreamlitのUI更新と受信スレッドの競合 | `threading.Lock` で共有データを保護 |
| `deque` への append と iteration の競合 | Lock 内で `list(deque)` でスナップショット取得 |
| スレッド停止の遅延 | `socket.settimeout(1.0)` + `threading.Event` |
| Streamlit のページリロードでスレッド重複起動 | `st.session_state` にスレッド参照を保持し、既存チェック |

### 7.5 時系列バッファのサイズ設計

Channel 4 (Velocity accuracy) は約200パケットに1回送信される（2秒に1回）。
60秒分 = 30サンプル。余裕を持って `maxlen=60` に設定。

---

## 8. Streamlit ダッシュボード (app.py)

### 8.1 アーキテクチャ

```
┌──────────────────────────────────────────────────────────┐
│ Streamlit Process                                         │
│                                                           │
│  ┌──────────────┐     ┌───────────────────────────────┐  │
│  │ UDP Receiver  │     │  UI Main Thread                │  │
│  │ (daemon)      │────▶│  @st.fragment で部分更新       │  │
│  │               │     │                               │  │
│  │ NcomDecoder   │     │  ┌── Nav/GNSS Status ──┐      │  │
│  │   ├── Ch0     │     │  │ Mode: GenAid  Sats:12│     │  │
│  │   ├── Ch4     │     │  └─────────────────────┘      │  │
│  │   ├── Ch78    │     │  ┌── Vel Accuracy ─────┐      │  │
│  │   └── Ch95    │     │  │ N/E/D metrics+chart  │     │  │
│  │               │     │  └─────────────────────┘      │  │
│  └───────┬───────┘     │  ┌── GAD Streams ──────┐      │  │
│          │              │  │ ID | Reject | Innov  │     │  │
│   ReceiverState         │  └─────────────────────┘      │  │
│   (Lock-protected)      │  ┌── CAN Bus Alerts ──┐      │  │
│                         │  │ Errors / OK rate    │      │  │
│                         └──┴─────────────────────┴──────┘  │
└──────────────────────────────────────────────────────────┘
```

### 8.2 UIレイアウト詳細

```
┌──────────────────────────────────────────────────────────────────────┐
│ OxTS GAD Monitor                                  [▶ Start] [⏹ Stop] │
├───────────┬──────────────────────────────────────────────────────────┤
│ Sidebar   │  Main Area                                               │
│           │                                                          │
│ 設定:     │  ── Nav: Real-Time │ GNSS Vel: GenAid │ Sats: 12 ──    │
│ ・Port    │                                                          │
│ ・更新間隔│  ┌──────────┐ ┌──────────┐ ┌──────────┐                │
│           │  │ Vel N σ  │ │ Vel E σ  │ │ Vel D σ  │                │
│           │  │ 23 mm/s  │ │ 19 mm/s  │ │ 41 mm/s  │                │
│           │  │ Δ -2     │ │ Δ +1     │ │ Δ +5     │                │
│           │  └──────────┘ └──────────┘ └──────────┘                │
│           │                                                          │
│           │  ┌──────── Velocity Accuracy (60s) ────────────────┐    │
│           │  │  ~~~~\/~~~~\/~~~~/\~~~  North (mm/s)             │    │
│           │  │  __________/\______    East  (mm/s)              │    │
│           │  │  _______/\_________    Down  (mm/s)              │    │
│           │  └──────────────────────────────────────────────────┘    │
│           │                                                          │
│           │  ┌──────── GAD Stream Monitor ──────────────────────┐   │
│           │  │ Stream │ Status  │ Rejections │ Innovation │ Qual│   │
│           │  │ ID=1   │ Active  │ 0          │ 0.3σ       │ ✅ │   │
│           │  │ ID=3   │ Timeout │ -          │ -          │ ❌ │   │
│           │  │ ID=5   │ Active  │ 2          │ 1.8σ       │ ⚠️ │   │
│           │  └──────────────────────────────────────────────────┘   │
│           │                                                          │
│           │  ┌──────── CAN Bus Status ──────────────────────────┐   │
│           │  │ TX OK: 100%  RX OK: 98%  Errors: 3              │   │
│           │  └──────────────────────────────────────────────────┘   │
└───────────┴──────────────────────────────────────────────────────────┘
```

### 8.3 実装の核心：`@st.fragment` による部分更新

```python
import streamlit as st
import pandas as pd
import time

def init_session():
    if "receiver_state" not in st.session_state:
        st.session_state.receiver_state = ReceiverState()
    if "receiver_thread" not in st.session_state:
        st.session_state.receiver_thread = None


@st.fragment(run_every=1.5)
def realtime_metrics():
    """速度精度、GAD、CANをリアルタイム更新する部分レンダラ"""
    state = st.session_state.receiver_state

    with state.lock:
        vn = state.vel_acc_north
        ve = state.vel_acc_east
        vd = state.vel_acc_down
        vel_age = state.vel_acc_age
        history = list(state.accuracy_history)
        gad = dict(state.gad_streams)
        can = dict(state.can_status)
        nav = state.nav_status
        nav_code = state.nav_status_code
        gnss_pos = state.gnss_pos_mode_name
        gnss_vel = state.gnss_vel_mode_name
        sats = state.num_satellites
        pkts = state.packet_count
        last_time = state.last_packet_time

    # --- 接続ステータス + GNSS モード ---
    if last_time > 0 and (time.monotonic() - last_time) < 3.0:
        status_text = f"Nav: **{nav}** | GNSS Vel: **{gnss_vel}** | GNSS Pos: **{gnss_pos}** | Sats: **{sats or 'N/A'}** | Packets: {pkts:,}"
        if nav_code == 4:
            st.success(status_text)
        else:
            st.warning(status_text)
    elif last_time > 0:
        st.warning(f"No data for {time.monotonic() - last_time:.0f}s | Last nav: {nav}")
    else:
        st.info("Waiting for NCOM data on UDP...")

    # --- 速度精度メトリクス ---
    st.subheader("Velocity Accuracy")
    col1, col2, col3 = st.columns(3)
    prev = st.session_state.get("prev_vel_acc", (None, None, None))

    def fmt_mms(val):
        """m/s → mm/s で表示（人間に見やすい）"""
        return f"{val * 1000:.1f} mm/s" if val is not None else "N/A"

    def delta_mms(cur, prev_val):
        if cur is not None and prev_val is not None:
            d = (cur - prev_val) * 1000
            return f"{d:+.1f}"
        return None

    col1.metric("North σ", fmt_mms(vn), delta_mms(vn, prev[0]))
    col2.metric("East σ",  fmt_mms(ve), delta_mms(ve, prev[1]))
    col3.metric("Down σ",  fmt_mms(vd), delta_mms(vd, prev[2]))
    st.session_state.prev_vel_acc = (vn, ve, vd)

    if vel_age >= ACCURACY_AGE_INVALID:
        st.caption("⚠ Velocity accuracy data too old (age ≥ 150)")

    # --- 時系列グラフ ---
    if history:
        df = pd.DataFrame(history, columns=["time", "North", "East", "Down"])
        now = time.monotonic()
        df["Seconds Ago"] = now - df["time"]
        df = df[df["Seconds Ago"] <= 60]
        # mm/s に変換して表示
        for col_name in ["North", "East", "Down"]:
            df[col_name] = df[col_name] * 1000
        chart_df = df[["Seconds Ago", "North", "East", "Down"]].set_index("Seconds Ago")
        st.line_chart(chart_df, use_container_width=True, y_label="mm/s")
    else:
        st.caption("Waiting for velocity accuracy data (Channel 4)...")

    # --- GAD ストリームモニタ ---
    st.subheader("GAD Stream Monitor")
    active_streams = [info for info in gad.values() if info.status != "Timeout"]
    if active_streams:
        st.success(f"Channel 95 reports {len(active_streams)} active GAD stream(s)")
    else:
        st.warning("Only timeout GAD streams are currently visible on Channel 95")

    if gad:
        gad_rows = []
        for sid, info in sorted(gad.items()):
            status = info.status
            quality = info.innovation_quality
            valid_inns = [f"{i:.1f}σ" for i in info.innovations if i is not None]
            inn_str = ", ".join(valid_inns) if valid_inns else "N/A"

            if status == "Active" and quality == "Good":
                indicator = "✅"
            elif status == "Active":
                indicator = "⚠️"
            elif status == "Timeout":
                indicator = "❌"
            else:
                indicator = "🔶"

            gad_rows.append({
                "Stream ID": sid,
                "Status": status,
                "Rejections": info.rejection_count,
                "Innovations": inn_str,
                "Quality": quality,
                "": indicator,
            })
        st.dataframe(pd.DataFrame(gad_rows), use_container_width=True, hide_index=True)
    else:
        st.caption("No GAD streams detected yet (Channel 95).")

    # --- CAN バスステータス ---
    st.subheader("CAN Bus Status")
    if can:
        tx_ok = can.get("tx_ok_pct")
        rx_ok = can.get("rx_ok_pct")
        errors = can.get("error_count", 0)

        can_cols = st.columns(4)
        can_cols[0].metric("TX OK", f"{tx_ok}%" if tx_ok is not None else "N/A")
        can_cols[1].metric("RX OK", f"{rx_ok}%" if rx_ok is not None else "N/A")
        can_cols[2].metric("Errors", str(errors))
        can_cols[3].metric("Last Error Code", str(can.get("last_error_code", 0)))

        if tx_ok is not None and tx_ok < 100:
            st.warning(f"CAN TX quality degraded: {tx_ok}% OK")
        if rx_ok is not None and rx_ok < 100:
            st.warning(f"CAN RX quality degraded: {rx_ok}% OK")
        if errors > 0:
            st.error(f"CAN bus errors detected: {errors} errors, last code: {can.get('last_error_code', 0)}")
    else:
        st.caption("Waiting for CAN bus data (Channel 78)...")


def main():
    st.set_page_config(page_title="OxTS GAD Monitor", layout="wide")
    st.title("OxTS GAD Monitor")
    init_session()

    with st.sidebar:
        st.header("Settings")
        port = st.number_input("UDP Port", value=3000, min_value=1, max_value=65535)

        col_start, col_stop = st.columns(2)
        start = col_start.button("Start", use_container_width=True)
        stop = col_stop.button("Stop", use_container_width=True)

    if start and st.session_state.receiver_thread is None:
        state = st.session_state.receiver_state
        thread = NcomUdpReceiver(state, port=port)
        thread.start()
        st.session_state.receiver_thread = thread
        st.toast("Streaming started!")

    if stop and st.session_state.receiver_thread is not None:
        st.session_state.receiver_thread.stop()
        st.session_state.receiver_thread = None
        st.toast("Streaming stopped.")

    realtime_metrics()

if __name__ == "__main__":
    main()
```

### 8.4 `@st.fragment` の動作メカニズム

| 特性 | 説明 |
|------|------|
| `run_every=1.5` | 1.5秒ごとにフラグメント関数のみ再実行 |
| ページ全体リロード | **発生しない**（フラグメント外のUI要素は維持） |
| session_state | フラグメント内からもアクセス可能 |
| 注意点 | フラグメント内で `st.button` 等のウィジェットは使用しない（状態管理の問題） |

### 8.5 更新間隔の設計根拠

- NCOM出力レート: 100Hz（10ms間隔）
- Channel 4 (velocity accuracy) 更新間隔: 約2秒に1回（200パケット周期）
- Channel 95 (GAD info) 更新間隔: 約2秒に1回
- Streamlitの描画オーバーヘッド: 200-500ms
- **推奨更新間隔: 1.5秒** — Channel 4 の更新周期よりやや短い

---

## 9. エラーハンドリングと堅牢性

### 9.1 パケットレベル

| エラー | 対策 |
|--------|------|
| パケット長 ≠ 72 | デコードスキップ + error_count++ |
| Sync ≠ 0xE7 | デコードスキップ |
| NavStatus == 11 (Structure-B) | 無視（OxTS内部パケット） |
| チェックサム不一致 | パケットを使用するがログに warning |
| 速度精度の Age ≥ 150 | UIに「data too old」の注釈表示 |
| Channel 95 stream_id == 0 | スキップ（Invalid） |

### 9.2 ネットワークレベル

| エラー | 対策 |
|--------|------|
| UDP bind失敗 | `st.error()` で明示、リトライ不要 |
| データ途絶 | UIに「No data for Xs」警告 |
| バッファオーバーフロー | `deque(maxlen)` で自動破棄 |

### 9.3 Streamlit固有

| 問題 | 対策 |
|------|------|
| ページリロードでスレッド重複 | `st.session_state` に参照保持 + 既存チェック |
| ブラウザタブ閉じ → ゾンビスレッド | `daemon=True` で自動停止 |
| 複数ユーザー同時アクセス → ポート競合 | 単一ユーザー前提。将来的にシングルトン受信 |

---

## 10. テスト戦略

### 10.1 パケットシミュレータ (tests/fake_ncom_sender.py)

実機がない環境でのテスト用。UDPでダミーNCOMパケットを送信する。

テストシナリオ:
1. **正常動作**: NavStatus=4, 速度精度が安定、GAD Stream Active
2. **GAD遮断**: 途中で Channel 95 の送信を停止 → UIでTimeout検知
3. **精度劣化**: velocity accuracy を徐々に増加 → グラフに劣化が表示
4. **CAN エラー**: Channel 78 のエラーカウントを増加 → UIにアラート

シミュレータの実装要件:
- ステータスチャネルを周期的にサイクルする（実機と同様）
- Channel 4: sin 波で速度精度を変動（10-50 mm/s 範囲）
- Channel 95: GAD stream_id=1 を Active/Timeout で切り替え
- Channel 78: CAN エラーを間欠的に発生
- Checksum は正しく計算する（累積式）

### 10.2 ユニットテスト対象 (tests/test_decoder.py)

| テスト | 内容 |
|--------|------|
| `decode_int24_le` | 0, 正最大(0x7FFFFF), 負最大(0x800001), -1(0xFFFFFF) |
| `verify_checksum` | 正常パケット / 破損パケット / 累積チェックサムの正確性 |
| `NcomDecoder.decode` | 完全パケットの全フィールド検証 |
| `_decode_ch4_velocity_accuracy` | UShort→m/s変換, Age有効/無効 |
| `_decode_ch95_gad_info` | stream_id, rejection_count, innovation デコード |
| `_decode_ch78_can_status` | エラーカウント, OK% |
| `decode_innovation` | Bit0有効判定, 7ビット符号付きデコード |
| NavStatus=11 | Structure-B パケットが正しく無視されること |

---

## 11. 将来の拡張候補（スコープ外だが設計で考慮）

1. **位置精度・姿勢精度の表示追加** — Channel 3/5 は既にデコード対応済み
2. **CSVエクスポート** — accuracy_history を pandas DataFrame として保存
3. **アラート閾値の設定** — 精度がN mm/sを超えたら通知
4. **マップ表示** — lat/lon を `st.map()` または `pydeck` で表示
5. **NCOM録画ファイルの再生** — UDP受信の代わりにファイル読み込みモード
6. **Innovation 時系列グラフ** — Channel 95 の innovation を時系列で表示
7. **マルチユーザー対応** — シングルトン受信スレッド + Pub/Sub配信
