# NHC12 図書館デモ（返却・貸出／双腕試作）

Yaskawa NHC12 で本の返却と貸出を行うサンプル集です。3つの構成があります。

| 構成 | 内容 |
|---|---|
| `run_library.py` | 移動台車上の単腕。返却2冊を棚へ収納し、依頼本1冊を貸出ボックスへ運ぶ |
| `crowded_shelf.py` | 固定の双腕。隣接本を押して隙間を広げ、傾いた本を支えて出し入れする試作 |
| `mobile_dual.py` | 移動台車上の双腕。棚Aで隙間を開けて返却し、棚Bから貸出本を取り出す試作 |

本IDと棚位置は `scene.py` の `CATALOG`（単腕版）で対応付けています。
探索は既知カタログの照合で、画像認識や未知の本の発見は行いません。

## 共通の前提と近似

- Isaac Sim 6.0 / Isaac Lab 3.0 / 本リポジトリが導入済みの環境（開発コンテナ等）
- 公式ロボット資産 `Robots/Yaskawa/Motoman Next/NHC12/NHC12_A00.usd` へのアクセス
- 物理は CPU PhysX 固定です（`--device cpu`）
- 台車移動は指定経路に沿うキネマティックな更新で、車輪駆動やスリップは評価しません
- 把持はトング開閉の表示と理想的な固定拘束による近似で、本は剛体として扱います
- 動画は記録した USD 形状と PhysX 姿勢の CUDA 描画で、RTX カメラ映像ではありません

## 単腕・移動版（run_library）

```bash
# 全工程の自動実行。成功時は終了コード 0
/isaac-sim/python.sh -m isaaclab_arena_examples.library.run_library \
  --device cpu --headless --viz none --output /tmp/library_result.json

# 姿勢を記録して動画を作る（CAPTURE は新規または空のフォルダ）
CAPTURE=/tmp/library-capture-new
/isaac-sim/python.sh -m isaaclab_arena_examples.library.run_library \
  --device cpu --headless --viz none --capture-fps 10 \
  --capture-dir "$CAPTURE" --output "$CAPTURE/result.json"
/isaac-sim/python.sh -m isaaclab_arena_examples.library.render_video "$CAPTURE"
```

GUI で確認する場合は `--headless --viz none` を `--viz kit --keep-open` に変えます。
`--inspect` は動作を開始せずシーンだけ確認します。
手元の USD は `--robot-usd /path/to/NHC12_A00.usd` で指定できます。

`result.json` に本ID・棚番、カタログ照合・把持・返却・貸出のイベントと、
各本の目標・実測姿勢・速度・解放状態を保存します。
成功条件は、全3冊が解放済みで目標から各軸 8 mm 未満、姿勢誤差 0.1 rad 未満、
静止状態を 0.5 秒維持し、貸出対象のカタログ照合が完了することです。

## 密集棚・固定双腕版（crowded_shelf）

検証済みの NHC12 を2本使い、片方が返却本を保持し、もう片方が隣接本を押して
隙間を広げます。周囲の本も自由な剛体で、1冊は15度傾けて配置します。
隣接本を目標座標へ直接移す処理はありません。

```bash
/isaac-sim/python.sh -m isaaclab_arena_examples.library.crowded_shelf \
  --device cpu --headless --viz none --output-dir /tmp/library-crowded-new
/isaac-sim/python.sh -m isaaclab_arena_examples.library.render_crowded /tmp/library-crowded-new
```

`--no-capture --stop-after <工程名>` は途中までの診断用で、
診断完了は本の返却・貸出の成功を意味しません。
接触操作は静摩擦0.12・動摩擦0.08の設定で試験しています。
未知の摩擦、本の変形、任意の倒れ方への対応は保証していません。

## 移動双腕版（mobile_dual）

同型アーム2本を1台の台車に載せ、右腕で棚Aの隙間を広げ、左腕で返却本を挿入し、
台車で棚Bへ移動して右腕で貸出本を取り出します。
挿入目標は退避後に再測定した隙間の中央とし、カタログ上の棚位置は
`catalog_target` として結果に残します（最終判定は計画目標が基準です）。
トングの接触形状は開閉に追従しない固定パッドです。

```bash
/isaac-sim/python.sh -m isaaclab_arena_examples.library.mobile_dual \
  --device cpu --headless --viz none --output-dir /tmp/library-mobile-dual-new
/isaac-sim/python.sh -m isaaclab_arena_examples.library.render_mobile_dual /tmp/library-mobile-dual-new
```

成功条件は、再測定した隙間への返却本の配置と貸出本のボックス内配置、
両対象本の解放・静止、傾き20度未満、4回のステーション到着です。

## テスト

配置判定とカタログ整合性の回帰テスト（シミュレーション不要）:

```bash
python -m pytest -q isaaclab_arena_examples/tests/test_library_placement.py
```

## 検証状況

3構成とも、開発環境（CPU PhysX、描画なし）で全工程の成功を確認済みです。
いずれも固定条件のデモの検証であり、摩擦把持、未知の本配置、障害物回避の
成功を保証するものではありません。
