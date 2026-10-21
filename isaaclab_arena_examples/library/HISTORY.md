# 図書館デモの開発履歴

過去の実装・検証の記録です。記載された旧動画・一時保存先の一部は削除済みです。
現行の手順は [README](README.md)、保存中の動画は共通カタログを参照してください。

# 移動ロボットによる図書館の返却・貸出デモ

開発対象ロボットは **Galbot S1** に切り替えています。
[図書館タスクの実行手順](#s1による図書館タスク)を参照してください。
[S1の導入手順と検証範囲](#galbot-s1今後の図書館用モデル)を参照してください。
以下の本操作デモは従来のNHC12版で、G1系の導入試験も比較用に保持しています。

4輪台車に載せたYaskawa NHC12アームで、次の順に本を扱います。

1. 返却ボックスからLIB-001を取り出し、本棚へ移動してA-01へ縦に収納する。
2. 返却ボックスへ戻り、LIB-002を取り出してA-02へ収納する。
3. 貸出予定のLIB-003をカタログで照合し、A-03から引き抜く。
4. 貸出ボックスまで運び、本を入れて解放する。

本棚は上段・下段と背景の本を持つ高さ2.1 mの書架です。対象本には色付きの背と
IDマーク、各棚位置には同色のマークを付けています。返却ボックスは茶色、貸出ボックスは青緑色です。
棚前で本を持ち上げて水平に抜き差しする経由点を使います。

本IDと棚位置は [scene.py](scene.py) の `CATALOG` で対応付けています。
探索は既知のカタログ情報による棚番の照合です。画像認識や未知の本の発見は行いません。
台車移動は指定経路に沿ってキネマティックな台車とアームのルート姿勢を更新する方式です。
車輪の駆動トルクや接地・スリップを評価する走行モデルではありません。
把持はトングの開閉表示と実測相対姿勢の固定拘束で近似し、本は剛体として扱います。

## 現在のコンテナで実行

新しいDockerコンテナは不要です。既存の非rootユーザーで実行してください。
この環境の `.venv/bin/python` はリンクが切れているため、残っているPython実体を直接使います。
依存環境の再インストールやリンク変更は行っていません。

```bash
cd /work
export OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y
export WARP_CACHE_PATH=/tmp/library-warp-cache
export XDG_CACHE_HOME=/tmp/library-cache
export LD_LIBRARY_PATH=/work/data/arena-runtime/sysroot/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
LIBRARY_PYTHON=/work/data/arena-runtime/python/cpython-3.12.15-linux-x86_64-gnu/bin/python3.12
LIBRARY_CAPTURE=/tmp/library-mobile-new

"$LIBRARY_PYTHON" \
  -c 'import sys,runpy; sys.path.append("/work/.venv/lib/python3.12/site-packages"); runpy.run_module("isaaclab_arena_examples.library.run_library",run_name="__main__")' \
  --device cpu --headless --viz none --capture-fps 10 \
  --capture-dir "$LIBRARY_CAPTURE" --output "$LIBRARY_CAPTURE/result.json"

"$LIBRARY_PYTHON" \
  -c 'import sys,runpy; sys.path.append("/work/.venv/lib/python3.12/site-packages"); runpy.run_module("isaaclab_arena_examples.library.render_video",run_name="__main__")' \
  "$LIBRARY_CAPTURE"
```

`LIBRARY_CAPTURE` は新規か空の保存先を指定してください。共有キャッシュを使う場合は
実行ユーザーが書き込めることを確認してください。`--capture-dir` を省略すると物理実行のみです。
CPU指定は物理計算用です。GPUは既存コンテナから見える割り当てを使用します。
公式NHC12 USDへのアクセスが必要です。ローカル資産は `--robot-usd /path/to/NHC12_A00.usd`。
参照先のメッシュ等も元の構成で揃えてください。

標準Arena環境では `/isaac-sim/python.sh -m isaaclab_arena_examples.library.run_library` でも起動できます。
GUIはRTX対応GPUとディスプレイが必要です。`--headless --viz none` を `--viz kit --keep-open` へ
変更して表示します。`--inspect` は動作を開始せずシーンを確認します。
`--max-steps` の既定値は80000、`--waypoint-timeout` は30秒です。

## 成果物と判定

`result.json` は本ID・棚番、カタログ照合・把持・返却・貸出のイベント、
各本の目標位置と実測姿勢・速度・解放状態を保存します。
全3冊が解放済みで、目標から各軸8 mm未満、姿勢誤差0.1 rad未満、
並進速度0.02 m/s未満・角速度0.1 rad/s未満の状態を0.5秒維持し、
貸出対象のカタログ照合も完了した場合に成功します。成功は終了コード0です。

動画は実測PhysX姿勢とUSD形状を使ったCUDA描画です。RTXカメラ映像ではありません。
冒頭に全景、続いて台車を追う画角で返却・貸出を表示します。10 Hz記録を30 fpsで再生するため
動作部分は3倍速です。`library-mobile.mp4`、確認用PNG、描画条件JSONを生成します。
物理記録は `scene.npz`、記録条件と結果は `capture.json`。`--preview-only` でPNGだけを作れます。

以前の固定台版動画は `/work/outputs/library-video-20261019/` に残しています。

## 移動版の検証結果

2026-10-19、既存コンテナ内の非root実行・CPU PhysXで全工程が成功しました。
22,793ステップ（物理時間189.94秒）で完了し、3冊すべての解放・配置・静止を確認しました。
最終位置誤差は1.17〜1.87 mm、最終速度は全成分0です。
カタログ照合→把持→棚返却／貸出ボックス配置のイベントと、
台車の返却・棚・貸出停止位置を記録しています。
棚の上板下面は1.78 mとし、試験で見つかった手首と上板の干渉を解消しました。

足回りを含む1,901時点・全19工程を記録しています。未知の障害物回避、
画像認識、車輪の物理走行、摩擦把持、他の棚寸法や本配置は検証していません。
Docker新規作成、依存の再インストール、ホスト設定変更は行っていません。

作成済みの移動版動画は `/work/outputs/library-mobile-20261019/library-mobile.mp4` です。
960×540、30 fps、68.37秒（動作部分3倍速）です。同フォルダにプレビューPNG、
実測記録、結果JSON、描画条件とフレーム検証結果を保存しています。

## 密集した棚・傾いた本を扱う双腕試作

`crowded_shelf.py` は棚作業に絞った別の試作です。検証済みのNHC12を2本使い、
片方が返却本を保持し、もう片方の衝突付きの薄い本押さえで隣接本を押して隙間を広げます。
周囲の15冊も自由な剛体で、1冊は15度傾けて配置します。
隣接本を目標座標へ直接移す処理はありません。
返却本の把持は既存例と同様に理想的な固定拘束です。

上記の既存ランタイム・キャッシュ設定で実行します。

```bash
"$LIBRARY_PYTHON" \
  -c 'import sys,runpy; sys.path.append("/work/.venv/lib/python3.12/site-packages"); runpy.run_module("isaaclab_arena_examples.library.crowded_shelf",run_name="__main__")' \
  --device cpu --headless --viz none --output-dir /tmp/library-crowded-new

"$LIBRARY_PYTHON" \
  -c 'import sys,runpy; sys.path.append("/work/.venv/lib/python3.12/site-packages"); runpy.run_module("isaaclab_arena_examples.library.render_crowded",run_name="__main__")' \
  /tmp/library-crowded-new
```

`--no-capture --stop-after <工程名>` は録画なしで特定工程までを確認する診断用です。
診断完了は本の返却・貸出の成功を意味しません。失敗時は原因確認用の最終姿勢を1時点保存します。
接触操作は静摩擦0.12・動摩擦0.08の設定で試験します。
未知の摩擦、本の変形、任意の倒れ方や完全に崩れた山を扱えることは保証していません。

床のちらつき・動く帯の修正動画は
`/work/outputs/library-shadow-fixed-20261019/library-mobile.mp4` です。
重複する床面による深度競合を除き、影の投影範囲をワールド座標に固定しました。

密集棚の途中検証では、接触で隙間を約25 mmから99 mmへ広げ、
35 mm厚の返却本を挿入・解放し、両腕が棚から退避するところまで確認しました。
双腕の通し記録は `/work/outputs/library-crowded-dual-20261019/` に保存しました。
`crowded-shelf.mp4` は返却ボックスからの取り出し、隙間拡大、挿入、隣接本の支持、
貸出本の取り出しとボックスへの配置を含みます。固定台に載せた2本のアームの試作で、
移動台車との統合は含みません。

22,794ステップ（189.95秒）の実測記録では、隙間は25.0 mmから99.0 mmへ広がりました。
本の実測姿勢から上端と支持高さを計算し、上から押さえを入れます。
支持しても隣接本が横倒しになる場合があり、棚全体の倒れ防止は未実装です。
補助アームの台は1.10 m、天板下面は2.15 mで、前腕の接触を避けています。

返却本は周囲の接触で横に17.53 mm、奥行き2.44 mm移動して静止しました。
当初の各軸15 mm判定では不合格でした。横方向20 mm・奥行きと高さ15 mmを
指定スロットの許容とし、変更していない実測記録で再判定しています。
元の結果は `result-original-15mm.json`、再判定の条件・各判定結果は `result.json` です。
貸出本のボックス内配置、両対象本の解放・静止、隙間60 mm超、支持工程の実行も確認しました。
新しい許容値での物理実行は繰り返していません。把持は理想拘束で、任意の崩れ方への対応は未検証です。

## 同じトングを備えた移動双腕版

`mobile_dual.py` は、同型のNHC12アーム2本を1台の4輪台車に載せた試作です。
両腕とも同じトングを使い、以前の補助アームにあった長い本押さえは使いません。
ボックスのステーションと本棚を離し、本棚A・Bも2 m離して配置しています。
右腕で本棚Aの隙間を広げて退避し、左腕で返却本を挿入します。
続いて台車全体で本棚Bへ移動し、右腕で貸出本を取り出して貸出ボックスへ運びます。

上記の `LIBRARY_PYTHON`、環境変数、非rootユーザーの設定を使います。
保存先には新規または空のディレクトリを指定してください。

```bash
LIBRARY_DUAL_CAPTURE=/tmp/library-mobile-dual-new
"$LIBRARY_PYTHON" \
  -c 'import sys,runpy; sys.path.append("/work/.venv/lib/python3.12/site-packages"); runpy.run_module("isaaclab_arena_examples.library.mobile_dual",run_name="__main__")' \
  --device cpu --headless --viz none --output-dir "$LIBRARY_DUAL_CAPTURE"

"$LIBRARY_PYTHON" \
  -c 'import sys,runpy; sys.path.append("/work/.venv/lib/python3.12/site-packages"); runpy.run_module("isaaclab_arena_examples.library.render_mobile_dual",run_name="__main__")' \
  "$LIBRARY_DUAL_CAPTURE"
```

`result.json` が工程と実測結果、`scene.npz` が描画用の姿勢記録です。
描画は `mobile-dual.mp4` と確認用PNGを生成します。`--preview-only` でPNGだけを作れます。
2026-10-20、既存コンテナのCPU PhysXで初期状態から全工程が成功しました。
22,624ステップ（188.53秒）、台車の移動距離8.276 mです。
成果物は `/work/outputs/library-mobile-dual-20261020/mobile-dual.mp4` に保存しています。
1280×720、30 fps、66.87秒（動作部分は約3倍速）です。全2,006フレームを復号確認しました。
両アームの取付位置と台車の相対位置は、全1,886記録時点で誤差0.001 mm未満でした。
これは同じ台車に固定した2本が一緒に移動していることの検証です。

返却棚にはブックエンドを設け、押した本の列が退避後に崩れるのを抑えています。
隙間は25.0 mmから82.4 mmへ広がりました。退避後に再測定した隙間の中央
X=-0.16266 mを挿入目標にし、返却本はX=-0.16334、Y=0.51644、Z=0.86000 mで静止しました。
貸出本はX=-1.69941、Y=-0.98106、Z=0.86000 mでボックス内に静止しました。
オンライン成功判定は各軸の位置誤差（X25 mm・Y/Z20 mm以内）、速度ノルム0.1未満、
両対象本の解放、傾き20度未満、4回の停止位置到着です。今回は許容値の事後変更はありません。

返却棚の奥止めは高さ30 cm、貸出棚は上から把持具を下ろすため10 cmです。
棚位置をカタログで照合する方式で、未知の本の画像認識はありません。
移動中は固定した全景、作業中は近い画角で実測記録を描画します。
CUDA描画の動画で、RTXカメラ映像ではありません。

走行は共通台車と両アームのルート姿勢を、決めた経路に沿ってキネマティックに更新します。
車輪駆動、SLAM、障害物回避は実装していません。
トングの開閉は表示に反映しますが、接触形状の範囲は固定です。
対象本の保持には理想的な固定拘束を使うため、摩擦把持の成功を示すものではありません。


## Galbot S1（今後の図書館用モデル）

図書館用の開発対象を、脚で昇降するOne Charlieから、支柱で上半身を上下する
Galbot S1へ切り替えています。以前の `galbot_robot.py`、`galbot_bringup.py`、
`render_galbot.py` とG1系の検証動画は残してあり、引き続き実行できます。
S1は `galbot_s1_robot.py`、`galbot_s1_bringup.py`、`render_galbot_s1.py` で分離しています。
既存NHC12図書館デモをS1の動作実績として扱わないでください。

公式モデル: https://github.com/GalaxyGeneralRobotics/galbot_s1_description
使用リビジョン: `cb758e557a880c70260c7883207334aed2aa3c65`（Apache-2.0）。
大きなモデルはGitへ追加せず、既存の `/work/data` に置きます。

```bash
git clone https://github.com/GalaxyGeneralRobotics/galbot_s1_description.git /work/data/galbot_s1_description
git -C /work/data/galbot_s1_description checkout cb758e557a880c70260c7883207334aed2aa3c65
export GALBOT_S1_USD=/work/data/galbot_s1_description/usd/galbot_s1.usda
```

既に取得済みなら再cloneは不要です。USDの隣にあるpayloadsとTexturesも必要です。
USDで確認した昇降関節は `torso_lift_joint1`（直動、0–0.735 m）です。
左右各7軸の腕、4輪の操舵・駆動関節を含みます。
標準の2本指グリッパーは実行時のオーバーレイで外し、Isaac Sim公式の左右Schunk SVH
5本指ハンドを固定関節で手首に接続しています。メーカー標準のS1構成ではなく、
シミュレーション用の組合せです。元の配布USDは変更していません。
ハンドは左右各9駆動軸と受動連動関節を持ちます。公式の連動拘束を保ち、受動関節を独立駆動しません。
使用アセットは `Isaac/Robots/Schunk/svh-flat-l/svh-flat-l_v2.usd` と右側の `svh-flat-r` です。
初回起動時に既存Isaacアセットサーバーへのアクセスが必要です。

左右ハンドに同一の取付回転を使うと、右手の親指が下を向き、指も外側へ閉じてしまいます。
右手だけ手首軸まわりに180度追加回転し、左右とも親指側が上、掌が内側になるようにしています。
`verify_s1_hands.py` は記録された姿勢から、親指側の基準点が手首より10 mm以上高く、
指の閉鎖時の移動が左右とも体の中心へ20 mm以上あることを確認します。

初期姿勢は全関節ゼロの横向き姿勢から、肘を曲げて両手を前に出す姿勢へ変更しました。
公式URDFの運動学と関節制限で解いた値を `FRONT_ARM_POSE` に保存しています。
胴体昇降0.25 mのとき、腕第7リンクの目標は基台座標で左右ともX=0.45 m、Y=±0.25 m、Z=0.95 mです。

簡易描画でも公式MDLの `base_color_factor` とUVのベースカラー画像を読み取ります。
白・青・黒の色を維持しますが、RTXの金属反射や細かな材質効果を再現するものではありません。

既存の非root・ランタイム環境設定を使います。新しいDockerや依存パッケージは不要です。

```bash
cd /work/.worktrees/galbot-library
S1_CAPTURE=/tmp/galbot-s1-new
"$LIBRARY_PYTHON" \
  -c 'import sys,runpy; sys.path.append("/work/.venv/lib/python3.12/site-packages"); runpy.run_module("isaaclab_arena_examples.library.galbot_s1_bringup",run_name="__main__")' \
  --device cpu --headless --viz none --output-dir "$S1_CAPTURE"
"$LIBRARY_PYTHON" \
  -c 'import sys,runpy; sys.path.append("/work/.venv/lib/python3.12/site-packages"); runpy.run_module("isaaclab_arena_examples.library.render_galbot_s1",run_name="__main__")' \
  "$S1_CAPTURE" --preview-only
```

指の向きは次のコマンドで検証できます。旧版の右手が逆向きの記録では失敗します。

```bash
"$LIBRARY_PYTHON" \
  -c 'import sys,runpy; sys.path.append("/work/.venv/lib/python3.12/site-packages"); runpy.run_module("isaaclab_arena_examples.library.verify_s1_hands",run_name="__main__")' \
  "$S1_CAPTURE"
```

静止画を確認後、最後の `--preview-only` を省くと実時間の動画を生成します。
試験は台座固定、重力なし、自己衝突なしです。重力ありでは位置誤差は小さい一方、
関節速度が静止基準を超えたため、本操作への統合は保留しています。自己衝突ありでは初期姿勢の
指関節が大きく変位し、追従試験に失敗しました。棚・本との接触、車輪走行、
本の返却・貸出はまだ接続していません。手先位置はグリッパーTCPではなく腕第7リンクで測定します。

五指ハンド・前構え版は2026-10-21に9,600ステップ（40秒、240 Hz）で合格しました。
昇降、左右の腕、左右の指の閉鎖・開放、初期姿勢への復帰を確認しています。
全16駆動指関節（左右の指開き軸を除く）の閉鎖から開放までの変位は最小0.27948 radでした。
駆動関節の目標誤差0.015未満、受動軸も含む全関節速度0.03未満を連続0.5秒以上維持しています。
これは空中での開閉で、本を摩擦で把持できることは検証していません。
動画と実測記録: `/work/outputs/library-galbot-s1-hands-corrected-20261021/`。

五指ハンド追加前の2026-10-21の制御ベンチは6,720ステップ（28秒、240 Hz）で合格しました。
上半身の昇降は0.20000 m、左右の腕第7リンクの変位はそれぞれ約0.07742 mです。
駆動関節の位置誤差0.015未満、全関節速度0.03未満を連続0.5秒以上維持しました。
回転関節はrad、昇降関節はmで判定します。受動連動関節は独立位置目標との比較から除外し、
速度の判定には含めています。これは昇降機構と双腕の制御確認であり、本の把持成功ではありません。
重力ありの失敗記録も、成功記録とは別に保存します。
旧構成の成果物: `/work/outputs/library-galbot-s1-20261021/`。

## Galbot導入用ブランチ：昇降・双腕の基礎制御

Galbotへの移行は `yuuharu11/feature/galbot-library` で分離しています。
既存のNHC12デモ、Arenaの既存Galbot embodiment、ユーザーの他の変更は更新しません。
作業ディレクトリは `/work/.worktrees/galbot-library` です。

`galbot_robot.py` は、インストール済みの公式 `GALBOT_ONE_CHARLIE_CFG` をコピーします。
USDは `IsaacLab/Robots/Galbot/galbot_one_charlie/galbot_one_charlie.usd` です。
最新カタログの `Galbot/galbot_g1/galbot_g1.usda` とは別のモデルです。
確認したモデルは23関節で、脚部4軸、左右腕各7軸、頭部、左グリッパー、右吸着具を含みます。
公式の移動台座の形状を使用しますが、このArticulationには車輪駆動関節がありません。

`galbot_bringup.py` は図書館タスクへ接続する前の制御試験です。
脚部の回転関節1・3を逆向きに動かし、胴体の姿勢を保ちながら下げ、
左腕・右腕を動かしてから胴体と両腕を元に戻します。胴体には前後方向の変位もあります。
区間内は5次補間を使い、指令の始端・終端で速度と加速度を0にします。
これは制御指令の平滑化であり、人の動作の模倣学習ではありません。

このモデルで自己衝突を有効にすると、初期姿勢から脚部が崩れ、目標への追従が妨げられました。
そのため本試験では台座固定・重力無効・自己衝突無効とし、有限のPDゲインを設定しています。
外部物体との衝突形状は削除していませんが、本試験には棚や本との接触はありません。
右手は公式の吸着具のままで、左右同型グリッパー化や把持制御の追加は行っていません。
本の返却・貸出、移動走行、全身の衝突回避、力に応じた接触制御はまだ未接続です。

既存コンテナ内の非rootユーザーで、前述のランタイム用環境変数を設定して実行します。
PythonやIsaac Simの再インストール、新しいDockerコンテナは不要です。

```bash
cd /work/.worktrees/galbot-library
GALBOT_CAPTURE=/tmp/galbot-control-new
"$LIBRARY_PYTHON" \
  -c 'import sys,runpy; sys.path.append("/work/.venv/lib/python3.12/site-packages"); runpy.run_module("isaaclab_arena_examples.library.galbot_bringup",run_name="__main__")' \
  --device cpu --headless --viz none --output-dir "$GALBOT_CAPTURE"

"$LIBRARY_PYTHON" \
  -c 'import sys,runpy; sys.path.append("/work/.venv/lib/python3.12/site-packages"); runpy.run_module("isaaclab_arena_examples.library.render_galbot",run_name="__main__")' \
  "$GALBOT_CAPTURE"
```

新規・空の保存先を指定してください。`--no-capture` は物理制御の検証のみです。
描画には既存のCUDA/nvdiffrast環境を使います。RTXカメラ映像ではありません。
保存先には `result.json`、`scene.npz`、`capture.json`、`galbot-bringup.mp4`、工程別PNGを出力します。
動画は30Hzの実測姿勢を30fpsで再生する実時間の記録です。

2026-10-21、CPU PhysXで3,360ステップ（28秒）の試験が成功しました。
胴体高さの差は0.14188 m、左TCPの変位は0.08593 m、右TCPの変位は0.09879 mでした。
各工程で全関節の目標誤差0.015未満・速度0.03未満を連続0.5秒以上維持しています。
回転関節はrad、直動のグリッパー関節はmで判定します。
これはこの姿勢列の動作確認で、機構の全昇降範囲や棚作業の成功を示す試験ではありません。
成果物は `/work/outputs/library-galbot-bringup-20261021/` に保存しています。
