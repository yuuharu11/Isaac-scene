# 図書館デモ：Galbot S1・移動双腕2枚コテ

## 最新版v4：等間隔の密な棚・取り出す本だけ白タグ

[動画・代表画像・測定](/work/outputs/library/README.md) ／ [再現条件](/work/outputs/library-review/library-v4/REPRODUCE.md)

棒状のブックエンドと青・緑の棚目印を除去し、白タグは取り出す貸出本1冊だけにしました。
返却・貸出段は初期各21冊、対象の両隣を含め同じ7 mm間隔です。返却位置は最初は本で埋まっており、右手で本の列を押し寄せて空け、左手で1冊返却します。貸出も1冊です。

全43工程・118.35秒を検証し、前回の163.10秒から27.4%短縮。
同じ7走行区間は76.60→48.10秒でした。速度・棚密度・隙間の作り方を同時に変更しています。

`--packed-shelves --blocker-angle 0 --motion-speed 1.8 --wheel-speed-limit 6 --travel-speed 0.40 --travel-acceleration 0.25 --arrival-gain 1.2 --jaw-force 4 --loan-jaw-force 4` を使用します。全オプションと起動方法は再現条件を参照してください。

厚さ3 mmの刃に対して、接近・開放の片側開口21 mmを使用します。押す予定の列は移動方向・量・回転・着座・静止を検証し、その他の本の変位上限5 mmは維持します。開放は接触力0.05 N未満、開口の指令幅に対する到達、従来どおりの速度・静定時間で確認します。
貸出本は高速な後退時に傾いた3 N条件の失敗記録を残し、返却本と同じ4 Nで輸送・下降と全工程を再検証しました。
高速駆動系はシミュレーション上の仮定です。本体の自己衝突は既存設定で無効、摩擦・モーターは未校正、本・刃は剛体です。

## 前回版v3：左手で1冊返却・右手で隙間確保

[最新動画・画像・測定結果](/work/outputs/library/README.md) ／ [再現手順](/work/outputs/library-review/library-v3/REPRODUCE.md)

返却本を左手で1冊運び、右手で傾いた隣本を起こして9 cm横へ移して空きを作り、左手で返却します。貸出も1冊です。
作業段は通常の本間隔6 mm、隣本21→31冊。7本棚に窓・閲覧テーブル・通路ラグを加え、全景カメラを低くしました。
全44工程・物理時間163.10秒・実走9.24 m、3冊すべての物理判定に合格。同じ7走行区間は公開中だったR2の115.75秒から76.60秒へ33.8%短縮しました。作業量も各2冊→各1冊に変わるため、全体時間の差は速度だけの効果ではありません。

```bash
python -m isaaclab_arena_examples.library.s1_mobile_spatula \
  --device cpu --viz none --output-dir NEW_EMPTY_DIRECTORY \
  --batch-books 1 --expanded-room --dense-shelves --cylinder-wheels \
  --motion-speed 1.4 --wheel-speed-limit 4 \
  --travel-speed 0.28 --travel-acceleration 0.18 \
  --arm-control-period 0.016666666666666666 \
  --adaptive-grasp-wait --jaw-close-duration 1 --max-jaw-wait 6 \
  --jaw-force 4 --loan-jaw-force 3
```

`--motion-speed` は腕と昇降の軌道時間を短縮し、把持・開放の静定待機を維持します。`--dense-shelves` は `--batch-books 1` 専用です。
駆動輪上限4 rad/sは元の1.5 rad/sを上回る高速駆動系を想定したシミュレーション条件で、実機の確認済み能力ではありません。把持力・車輪トルク上限と評価閾値は維持します。本体の自己衝突は既存設定で無効、摩擦やモーターは未校正です。
実行環境は通常リポジトリのDocker手順を使い、今回の既存非root環境の例外承認と起動ラッパーは再現手順に記録しています。

## 従来版R2の記録

> 返却2冊・貸出2冊のR2全工程を検証済み。測定結果と比較は [改善記録](/work/outputs/library-review/library-v2/README.md) に保存しています。

広い部屋、返却ボックス、書架、貸出ボックスで、左右の手首に付けた可変幅の2枚コテを使います。
台車は実車輪で移動します。従来の1冊返却・1冊貸出に加え、返却2冊・貸出2冊を1往復で扱う構成と、
計7棚の蔵書を配置した部屋を選択できます。傾いた隣本の移動を含め、評価対象は計5冊です。
作業ブランチは `yuuharu11/feature/library-s1-spatula-tilted-books` です。

保存動画と結果は [出力一覧](/work/outputs/README.md)、管理方法は [共通カタログ](../../demos/README.md) にあります。
[公開中の全体動画](/work/outputs/library/video.mp4) と
[従来版の検証・試行の整理](/work/outputs/demos/library-s1-mobile-spatula/runs/20261022-mobile-spatula-full/README.md) を保持します。
従来版は全44工程・実走9.29 m、3冊の保持・着座・開放・抜去を検証した記録です。
従来の2冊ずつの版は全66工程のR2です。実行先は
[20261023-library-v2-4books-r2](/work/outputs/demos/library-s1-mobile-spatula/runs/20261023-library-v2-4books-r2/) です。
**R2は全工程を完了し、評価対象5冊すべてが既存基準を合格しました。**
棚前だけの検証や途中停止のprobeは参考枠とし、全体工程の成功とは分けます。

初回の[4冊全工程run](/work/outputs/demos/library-s1-mobile-spatula/runs/20261023-library-v2-4books/) は、
2冊目の貸出配置で空の右工具と左腕が干渉し、未着座のまま解放動作へ進んで失敗しました。
`success=false` の実測・接触・診断を保持し、公開版へ昇格していません。
R2では、右の貸出本を置いた後に空の右工具を外上方へ退避する1工程を追加しています。
さらに開口前に実測した本のOBB底面と支持面の差が既存の5 mm未満であることを必須にし、
未着座での開放を禁止します。native cylinder車輪の初期base高さも公式URDFと床面から計算し、
接地前の落下を避けます。これらは初期化・経路・安全判定の修正で、動作中の本や台車の姿勢上書き、
接触の無効化、数値閾値の緩和は行いません。R2で全工程を再検証し、工具と腕の予期しない接触は0件でした。


## 全体工程（R2は66工程）

1. 返却ボックスの2冊を左右で順に把持し、縁より高く持ち上げて運搬姿勢へ移す。
2. 両冊を保持して通路を車輪走行し、書架Aへ近づく。
3. 右の返却本を予約した空きslotへ入れ、着座・開放・コテ抜去する。
4. 右のコテを傾いた隣本へ合わせ、把持して持ち上げ、手前へ抜く。横のslotへ移して着座・開放・抜去する。
5. 左で運んだ返却本を空いた場所へ入れ、着座・開放・抜去する。
6. 通路へ下がって書架Bへ移動し、上半身を上げる。
7. 高い棚の貸出2冊を左右で順に把持・抜去する。片腕の操作中も他方の保持を継続監視する。
8. 両冊と両工具が棚から離れたことを確認し、台車を後退、両腕を運搬姿勢へ移して上半身を下げる。
9. 貸出ボックスへ車輪走行し、右の本を着座・開放・抜去する。空の右工具を外上方へ退避してから左の本を着座・開放・抜去する。

`--batch-books 1` は従来の工程、`--batch-books 2` は上記の2冊ずつの工程です。
本ID・source/target・左右担当・評価phaseは [s1_library_plan.py](s1_library_plan.py) の `catalog_for` を使います。
作業段の隣本は動的剛体です。追加の背景蔵書・未使用段は静的支持物とし、作業経路の外へ置きます。
棚の支持部と机の支柱を床・天板へ接続し、作業面と本の目標座標は維持しています。

## 実装と物理条件

| 用途 | ファイル |
| --- | --- |
| 起動・物理step・記録・全体工程の実行 | [s1_mobile_spatula.py](s1_mobile_spatula.py) |
| catalog、1冊／2冊の工程と評価metadata | [s1_library_plan.py](s1_library_plan.py) |
| 実車輪・腕・jaw制御、把持力profile、待機・clearance判定 | [s1_mobile_control.py](s1_mobile_control.py) |
| 作業書架・動的な本・返却／貸出ボックス | [s1_library_scene.py](s1_library_scene.py) |
| 背景5棚と未使用段の静的蔵書 | [s1_library_room.py](s1_library_room.py) |
| コテ形状・直動関節 | [dual_spatula_probe.py](dual_spatula_probe.py) の `make_dual_tool` |
| 公式URDFに基づく腕の逆運動学 | [s1_library_ik.py](s1_library_ik.py) |
| 毎物理stepの接触・滑り・隣本変位・phase別force、実材質の読出し | [s1_library_diagnostics.py](s1_library_diagnostics.py) |
| 本・工具・ロボットの実測geometry/pose記録 | [crowded_capture.py](crowded_capture.py) |
| 全対象本と工程の実測評価 | [analyze_mobile_spatula.py](analyze_mobile_spatula.py) |
| 元S1のMDL色・textureの読込み | [asset_materials.py](asset_materials.py) |
| 材質・局所AO・CUDA描画 | [material_renderer.py](material_renderer.py)、[render_video.py](render_video.py) |
| 全景＋手元、平滑化したカメラ、動画出力 | [render_s1_library.py](render_s1_library.py) |

本は重力・摩擦・接触で保持し、本への固定ジョイント・吸着・姿勢の上書きは使いません。
固定ジョイントはコテの手首取付だけです。コテは120×3×80 mmです。
候補profileは返却本・傾いた隣本を各jaw 4 N、貸出本を各jaw 3 Nとし、閉じる前に物理driveへ設定して読戻し検証します。
設定は保持中に維持され、次の把持で本に応じた値を選びます。driveのjaw軸上限4→3 Nは25%減です。
記録する接触力は3Dの `force_matrix_w` のノルムで、面法線への射影は保存していません。
drive上限と実測のコテ接触力の大きさのピークは区別し、実測ピーク減は同じ20 Hz同士で比較します。
3 Nを傾いた隣本へ一律適用する案は滑り余裕が小さく、候補profileには採用していません。

台座は浮動体で、4輪の操舵・駆動モータと床の摩擦で移動します。
候補はnative cylinderの車輪接触を用います。ロボットのルート位置・速度を動作中に書き換えず、
台座・工具・本へ追従外力や重力補償を与えません。
車輪配置は公式URDFから読み、駆動トルク上限20 N m、操舵100 N m、関節速度上限1.5 rad/sを使います。
モータ、工具質量、摩擦は実機未校正です。

物理stepは240 Hz、腕のIK更新は60 Hzです。soft closeと、上限付きの実測待機を使います。
待機は平滑化していない240 Hzの本の実姿勢差分から並進・角速度を測り、既存の
0.01 m/s・0.05 rad/s未満、接触／開放、工具位置の条件を0.5秒継続し、close ramp完了を確認します。
raw solver速度と実姿勢変化の不一致は診断記録へ残し、数値閾値を緩めず測定対象を明示しています。

公式S1には初期の自己衝突形状の重なりがあるため、本体の自己衝突は無効です。
工具・本・棚・床との外部接触と重力は有効です。全身の自己干渉まで含む実機成立性は未検証です。
本と板は剛体で、変形・傷・ページ・認識・未知の厚さ・人のいる環境は評価しません。
コテ接触力の大きさを減らす測定はできても、本の損傷を減らしたという証明にはなりません。

## 実行と確認

通常は既存の開発用Docker内で非root実行します。この `/work` では、ユーザー承認済みの例外として
既存nativeランタイムを `nobody` で実行しています。この例外を他の環境へ自動適用しません。
新しい空の出力先を指定してください。候補の明示引数は次のとおりです。

```bash
python -m isaaclab_arena_examples.library.s1_mobile_spatula \
  --device cpu --viz none --output-dir /tmp/s1-library-v2-new \
  --batch-books 2 --expanded-room --cylinder-wheels \
  --dt 0.004166666666666667 --arm-control-period 0.016666666666666666 \
  --jaw-force 4 --loan-jaw-force 3 --jaw-close-duration 1 \
  --adaptive-grasp-wait --max-jaw-wait 6 \
  --travel-speed 0.11 --travel-acceleration 0.08
python -m isaaclab_arena_examples.library.analyze_mobile_spatula \
  /tmp/s1-library-v2-new --finalize
python -m isaaclab_arena_examples.library.render_s1_library \
  /tmp/s1-library-v2-new --preview-only
```

上記は対応する実行環境内の非root Pythonで使います。このセッションのnative起動wrapperは
`/tmp/library-spatula-20261022/run-mobile.sh` です。同wrapperに上記の物理引数を渡せます。
Python実体は `/work/data/arena-runtime/python/cpython-3.12.15-linux-x86_64-gnu/bin/python3.12`、
追加site-packagesは `/work/.venv/lib/python3.12/site-packages` です。
環境変数と起動書式は [既存ランタイムの手順](SHELF_CONTACT_REFERENCE.md#実行) を参照してください。
freezeした実行ソース・hash・引数は実行先の `source/` と `source_manifest.json` で確認します。

静止画と判定を確認してから `--preview-only` を外して動画を作ります。
`--speedup 1` は等速、既定は1.5倍速です。`--start` / `--end` で短い区間だけ描画できます。
記録された物理姿勢を使い、演出用の物理姿勢補間はしません。仮想カメラのみ平滑化します。
2冊版の手元注視点は各stationで一定にし、1冊版のカメラを維持します。
元S1の材質色を復元していますが、textureは頂点ベイクで細部が近似されます。
描画はCUDA rasterization・局所screen-space AOで、RTX映像ではありません。

`--probe-drive` は実車輪走行、`--probe-shelf` は傾いた隣本、`--probe-high` は高棚の代表検証です。
`--stop-after` は途中停止、`--diagnostic-phase grip_blocker` は指定phaseの240 Hz詳細記録です。
部分runの最終判定failを全体失敗／成功へ読み替えません。
`--blocker-angle 0` は直立本、既定の `-10` は傾斜本です。傾斜は重力で静定させ、角度を固定し続けません。

## 判定と記録

判定はcatalogの全対象本に適用します。予備probeの成功だけで全工程を合格にしません。
既存の主な条件は、最終位置誤差25 mm未満、着座誤差5 mm未満、保持中の相対滑り10 mm未満、
両jaw接触率80%超、動的隣本の最大変位5 mm未満、工具接触の侵入0.5 mm未満、
停車誤差6 mm未満と0.7秒超の静定です。各本のlift・高棚把持・解放・コテ抜去・最終速度／傾きも確認します。
閾値直前の候補は余裕もレビューし、丸めて合格にしません。

- `result.json` / `verification.json`：全工程と各冊の判定、停車位置、経路。
- `motion_audit.json`：240 Hzの保持・滑り・隣本変位／回転、phase別jaw forceとslewの最大値・時刻。
- `contact_pairs.json`：工具・本・隣本・ロボットの接触相手、impulse、侵入、ピーク時刻。
- `diagnostic_samples.json`：指定phaseの詳細poseとraw速度。rootの240 Hz記録は含みません。
- `scene.npz` / `capture.json`：実測geometry・姿勢・poststep時刻と描画metadata。
- `runtime_result.json`：評価で更新する前の実行結果。

### R2の結果

全66工程・299.65秒・実走9.25 m、評価対象5冊すべて合格。走行7区間は146.25→115.75秒（20.9%短縮）。返却・貸出の作業量は各1冊から各2冊へ増えています。

| 本 | 最終位置誤差 | 最大滑り | 両コテ接触率 | 判定 |
| --- | ---: | ---: | ---: | --- |
| 返却1 | 0.13 mm | 1.32 mm | 99.3% | 合格 |
| 傾斜本 | 0.16 mm | 1.96 mm | 95.0% | 合格 |
| 貸出1 | 1.31 mm | 3.14 mm | 99.0% | 合格 |
| 返却2 | 0.21 mm | 1.62 mm | 98.1% | 合格 |
| 貸出2 | 3.97 mm | 6.63 mm | 99.1% | 合格 |

周囲の本の最大変位0.797 mm、工具の最大接触侵入0.0033 mm。予期しない工具／腕接触0件、10回の開閉待機はすべて実測静定を確認。[詳細測定・比較](/work/outputs/library-review/library-v2/README.md) と [実行記録](/work/outputs/demos/library-s1-mobile-spatula/runs/20261023-library-v2-4books-r2/README.md) を参照してください。

## 試行の整理

- [棚前だけの接触検証](SHELF_CONTACT_REFERENCE.md)：固定台座の部品試験。全体動作の代用にしない。
- [2枚コテの比較・失敗例](DUAL_SPATULA_STUDY.md)、[1枚コテの初期検討](SPATULA_STUDY.md)。
- [旧五指版の全体工程](SVH_REFERENCE.md)：元の広い部屋の参考。理想把持と指定経路走行を含む。
- [過去の実装](HISTORY.md)：NHC12、G1系など。旧コードと保存動画は保持する。
- [今回の設計・描画・診断レビュー](/work/outputs/library-review/library-v2/analysis/README.md)：部分検証・契約検証の根拠。

動画・モデル・実測配列はGitへ追加せず、新しい実行IDへ保存します。
旧版と診断を削除せず、検証済みの全体版の実行IDだけを `latest.json` に記録します。
閲覧用は [outputs/library](/work/outputs/library/README.md) にまとめ、旧版は過去動画一覧から引き続き閲覧できます。

関節限界までの最小測定余裕は約0.57度で小さく、実機展開には姿勢の再調整が必要です。3／4 Nは駆動力の上限で、慣性・受動接触を含む実接触力のピーク値ではありません。
