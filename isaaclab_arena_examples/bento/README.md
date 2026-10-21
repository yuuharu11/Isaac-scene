# Yaskawa NHC12 弁当盛り付けデモ

MOTOMAN NEXT NHC12（6軸）と幅広の開閉トングで、ご飯・卵焼き・唐揚げ・ブロッコリーを
仕切り付き弁当箱の4区画へ順に移す、独立した Isaac Lab のサンプルです。
学習済みポリシーは不要です。Arena の環境レジストリには登録していません。

## 構成

| ファイル | 役割 |
|---|---|
| `simulation.py` | シーン構築、IK 制御、盛り付け工程と成功判定 |
| `meal.py` | 食品の寸法・色・質量・供給位置・配置区画と配置判定 |
| `visuals.py` | 食品・弁当箱・トングの手続き的メッシュ生成 |
| `capture.py` | USD 形状と実測姿勢の記録（動画用） |
| `render_video.py` | 記録からの CUDA ラスタライズ動画生成（nvdiffrast） |

## 前提

- Isaac Sim 6.0 / Isaac Lab 3.0 / 本リポジトリが導入済みの環境（開発コンテナ等）
- 公式ロボット資産 `Robots/Yaskawa/Motoman Next/NHC12/NHC12_A00.usd` へのアクセス
- 物理は CPU PhysX 固定です（`--device cpu`）
- 動画生成には CUDA GPU と `nvdiffrast`、`imageio-ffmpeg`、Pillow が必要です

## 実行

リポジトリルートから実行します。コンテナ内では `python` の代わりに
`/isaac-sim/python.sh` を使います（以下同様）。

```bash
# 全工程の自動実行。成功時は終了コード 0、失敗時は非 0
/isaac-sim/python.sh -m isaaclab_arena_examples.bento.simulation \
  --device cpu --headless --viz none \
  --output /tmp/yaskawa_bento_result.json

# GUI で確認（--keep-open を付けると判定後も開いたまま）
/isaac-sim/python.sh -m isaaclab_arena_examples.bento.simulation \
  --device cpu --viz kit --keep-open

# 動かさずにシーンだけ確認
/isaac-sim/python.sh -m isaaclab_arena_examples.bento.simulation \
  --device cpu --viz kit --inspect --max-steps 20000
```

手元の USD を使う場合は `--robot-usd /path/to/NHC12_A00.usd` を指定します。
参照先の configuration や mesh も解決できるよう、元のフォルダ構成を維持してください。
`--waypoint-timeout` で各移動の制限時間を変更できます。

結果 JSON には成功/失敗理由、ステップ数、最終フェーズ、食品の実測位置・
姿勢（XYZW）・速度・解放状態・区画内判定を記録します。

## 画像収集と動画

```bash
CAPTURE_DIR=/tmp/bento-capture-new   # 新規または空のフォルダを指定

# 1. 実シミュレーションの形状と姿勢を収集
/isaac-sim/python.sh -m isaaclab_arena_examples.bento.simulation \
  --device cpu --headless --viz none --capture-fps 10 \
  --capture-dir "$CAPTURE_DIR" --output "$CAPTURE_DIR/result.json"

# 2. 全体動画（960x540、30 fps）
/isaac-sim/python.sh -m isaaclab_arena_examples.bento.render_video \
  "$CAPTURE_DIR" --save-frames --width 960 --height 540 --fps 30

# 3. 食材の近接動画
/isaac-sim/python.sh -m isaaclab_arena_examples.bento.render_video \
  "$CAPTURE_DIR" --camera bento --width 960 --height 540 --fps 30

# 4. ロボットのロゴ近接画像だけを作る
/isaac-sim/python.sh -m isaaclab_arena_examples.bento.render_video \
  "$CAPTURE_DIR" --camera robot --preview-only
```

`scene.npz` にメッシュと実測姿勢、`capture.json` に収集条件と結果、
`video.json` に描画条件を保存します。動画は Isaac Sim の RTX カメラ映像ではなく、
記録した USD 形状と PhysX 姿勢を CUDA で描画したものです。

## ロボットとシーン

- 関節は `joint_1_s`、`joint_2_l`、`joint_3_u`、`joint_4_r`、`joint_5_b`、`joint_6_t`
- 手先は `link_6_t`、トングの食品中心 TCP はローカル `(0, 0, -0.20)` m
- 作業台の上面と固定アームのベースは高さ 0.75 m
- 食品は詳細な表示メッシュと単純な直方体の衝突形状を分けて生成します。
  寸法・色・質量・供給位置・配置先は `meal.py` で変更できます。
  台・箱の形状を変えるときは `simulation.py` と区画境界も合わせて変更してください。

## 物理と判定の範囲

- アームは CPU PhysX で動かし、減衰最小二乗 IK で関節位置を制御します。
- 保持は理想的な固定拘束による近似です。トングの開閉は形状のアニメーションで、
  顎の摩擦・握る力や、米粒をすくう動作はモデル化していません。
- 成功には、全食品が解放済みで、回転を考慮した全寸法が割当区画内に収まり、
  底面が弁当床から 8 mm 以内にあることを要求します。静止判定も併せて確認します。
- 固定の経由点を使うデモで、障害物回避の経路計画は実装していません。

## テスト

配置判定の回帰テスト（シミュレーション不要）:

```bash
python -m pytest -q isaaclab_arena_examples/tests/test_bento_placement.py
```

面ごとの材質が三角形化後も保たれることのテストは、Isaac Sim 起動が必要です:

```bash
/isaac-sim/python.sh -m pytest -q isaaclab_arena_examples/tests/test_bento_capture.py
```

## 検証状況

開発環境（CPU PhysX、描画なし）で全4品の配置成功を確認済みです。
固定配置のデモの検証であり、摩擦把持や異なる配置での成功を保証するものではありません。
