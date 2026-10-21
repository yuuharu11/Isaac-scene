# 図書館デモ：Galbot S1

現行の開発対象は **Galbot S1＋左右SVH五指ハンド** です。
コード・動画を探す入口は [共通カタログ](../../demos/README.md) と `/work/outputs/README.md` です。

## コードの対応

| 用途 | ファイル |
| --- | --- |
| 現行タスク・制御・判定 | [s1_library_task.py](s1_library_task.py) |
| 部屋・棚・本・返却先 | [s1_library_scene.py](s1_library_scene.py) |
| 腕の逆運動学 | [s1_library_ik.py](s1_library_ik.py) |
| 2画面の動画生成 | [render_s1_library.py](render_s1_library.py) |
| S1本体・ハンド・材質 | `galbot_s1_robot.py`、`galbot_s1_hands.py`、`galbot_s1_materials.py` |
| S1単体の導入確認 | `galbot_s1_bringup.py`、`render_galbot_s1.py`、`verify_s1_hands.py` |
| G1系の比較用コード | `galbot_robot.py`、`galbot_bringup.py`、`render_galbot.py` |
| 旧NHC12・混雑棚・双腕の派生 | `run_library.py`、`scene.py`、`mobile_controller.py`、`crowded_*`、`mobile_dual*` |
| 共有の記録・描画 | `capture.py`、`crowded_capture.py`、`render_video.py`。弁当の記録・描画処理も利用 |

旧実装とアセットは保持しています。過去の経緯・旧版の起動方法・S1/G1導入確認は [HISTORY.md](HISTORY.md) にまとめています。
旧S1動画は削除済みで、最新版の実測記録だけを保存しています。

## このコンテナの実行環境

既存コンテナ内の非rootユーザーで、以下の環境変数とPython実体を使います。
新しいDockerや依存環境は作成しません。標準ランタイムのPythonを使う場合は、この環境固有の設定は不要です。

```bash
cd /work/.worktrees/galbot-library
export OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y
export WARP_CACHE_PATH=/tmp/library-20261019-warp
export XDG_CACHE_HOME=/tmp/library-20261019-cache
export LD_LIBRARY_PATH=/work/data/arena-runtime/sysroot/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
LIBRARY_PYTHON=/work/data/arena-runtime/python/cpython-3.12.15-linux-x86_64-gnu/bin/python3.12
```

## S1による図書館タスク

確認用の動画一覧・最新版・検証結果は `/work/outputs/README.md` にまとめています。
最新版は `/work/outputs/library-latest/s1-library.mp4`（全景＋手元の2画面、文字なし）です。

作業ブランチ: `yuuharu11/feature/galbot-s1-library-task`。
作業ディレクトリ: `/work/.worktrees/galbot-library`。
G1系の導入コードと従来NHC12デモは保持しています。

`s1_library_task.py` はS1本体・左右SVH五指ハンドを1台のロボットとして動かします。
返却ボックスと貸出ボックスは棚から離し、2つの書架を配置しています。

1. 左手で返却本LIB-001を取り出し、持ち上げた後、低い運搬姿勢へ移る。
2. 共通台座で書架Aへ移動する。
3. 右手で茶色のNBR-001をロボットから見て右（ワールド負のY）へ15 cmずらし、棚に置いて右手を退避する。
4. 右手の退避後、左手で赤いLIB-001を茶色と緑の間（A-01）へ入れて解放する。
5. 書架Bへ移動し、上半身を40 cm上げ、高さ1.63 mのB-03の貸出本LIB-003を右手で取り出す。
6. 走行前に両腕と上半身を下げ、貸出ボックスへ運んで本を入れ、指を開いて解放する。

本IDと場所の対応は `s1_library_scene.py` の `CATALOG` に保存しています。
未知の本の画像認識や棚探索は行いません。隙間は茶色の本を5 cm持ち上げて右へ15 cm移すことで確保し、
並んだ本を力制御で押し広げる処理ではありません。周囲の本は固定支持物で、倒れた本の復旧は未対応です。

既存のランタイム環境変数と非rootユーザーを使います。新しいDockerや依存環境の再インストールは不要です。

```bash
cd /work/.worktrees/galbot-library
S1_LIBRARY_CAPTURE=/tmp/s1-library-new
"$LIBRARY_PYTHON" \
  -c 'import sys,runpy; sys.path.append("/work/.venv/lib/python3.12/site-packages"); runpy.run_module("isaaclab_arena_examples.library.s1_library_task",run_name="__main__")' \
  --device cpu --headless --viz none --output-dir "$S1_LIBRARY_CAPTURE"
"$LIBRARY_PYTHON" \
  -c 'import sys,runpy; sys.path.append("/work/.venv/lib/python3.12/site-packages"); runpy.run_module("isaaclab_arena_examples.library.render_s1_library",run_name="__main__")' \
  "$S1_LIBRARY_CAPTURE" --preview-only
```

最後の `--preview-only` を省くと `s1-library.mp4` を生成します。左に固定の全景、右に頭部付近の仮想カメラを同期して並べます（1920×720）。右のカメラは作業位置を向き、実機の光学カメラや頭部関節の視線制御を再現するものではありません。文字ラベルや凡例は描画しません。
この動画用描画は影マップと鏡面反射を無効にし、動く明暗の帯や光沢のちらつきを抑えています。
動画は15 Hzの実測姿勢を22.5 fps（1.5倍速）で再生します。`--speedup 1` で等速にできます。RTXカメラ映像ではありません。
`--probe-pick` は最初の本の持ち上げまで、`--probe-shelf` は茶色の本をずらして隙間を作るところまで、
`--no-capture` は物理動作と判定のみを実行します。
保存先には新規・空のディレクトリを指定してください。

ターゲット3冊には質量・重力・家具との衝突があります。本を目標位置へ直接書き換えません。
保持には測定した相対姿勢を使う理想的な固定拘束を使い、ロボットと対象本の衝突は除外します。
このタスクではSVH手指の衝突形状を無効にしています。指は関節で開閉しますが、
手指と家具・本との接触は評価しません。腕・台座の衝突形状と、本・家具の衝突は残しています。
五指の摩擦だけによる把持成功を示す実装ではありません。
ロボットの重力・自己衝突は無効、台座の走行は指定経路に沿うルート姿勢更新です。
車輪牽引力・SLAM・障害物回避・人のいる図書館での運用安全性は評価していません。

各工程で全駆動関節の目標誤差0.025未満、全関節速度0.05未満、手首位置誤差15 mm未満を
連続0.6秒以上維持してから次へ進みます。最終判定は3冊の目標誤差25 mm未満、
速度ノルム0.1未満、傾き15度未満、全冊の解放、アクセス空間の確保、茶色の本の右15 cm移動先への着座（誤差5 mm未満）、高さ1.60 m以上での把持、7.0–7.5 mの台座移動です。

## 保存済みの最新版

- 動画: `/work/outputs/demos/library-s1/latest/s1-library.mp4`
- 実測・検証: 同じフォルダの `result.json`、`verification.json`、`hand_geometry_check.json`
- 実行: `20261022-multiview`。118.679秒の動作を1.5倍速で約79秒にしています。

右手を退避してから左手で赤を挿入します。全記録姿勢で左右ハンドの形状範囲は分離しています。
この確認は手指の物理接触や全身の衝突回避を保証するものではありません。

新しい実行の保存・最新版への登録は [管理手順](../../demos/README.md#実行を追加する) に従ってください。

## 旧NHC12版の手順とテスト

単腕移動版・密集棚固定双腕版・移動双腕版の実行方法と検証条件は
[HISTORY.md](HISTORY.md)を参照してください。既存のモジュールは保持しています。
配置判定とカタログ整合性の回帰テストは
`isaaclab_arena_examples/tests/test_library_placement.py` にあります。
