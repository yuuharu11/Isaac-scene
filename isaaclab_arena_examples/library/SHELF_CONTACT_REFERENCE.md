# 参考：固定台座での棚前接触検証

この文書は棚前だけの部品検証の手順です。全体工程の入口は [README.md](README.md) です。
作業ブランチは `codex/feature/library-s1-spatula-tilted-books` です。
コードと保存動画の入口は [共通カタログ](../../demos/README.md) と [出力一覧](/work/outputs/README.md) です。

確認済みの最新版は [返却動画](/work/outputs/archive/README.md) と
[検証・試行の整理](/work/outputs/archive/README.md) です。
初期−10度の1条件で1/240秒・1/480秒とも成立しました。範囲と記録復元の詳細も同じ文書に残しています。

## 採用案：2枚コテによる棚前の返却

右腕に3 mm厚の薄板を2枚取り付け、左右の直動関節を各4 Nで駆動します。
本に固定ジョイントを作らず、重力・摩擦・接触で保持します。本の位置は動作中に書き換えません。

1. 傾いて隣の本にもたれた本へコテを合わせて差し込み、挟む。
2. 持ち上げて手前へ引き出し、横へ移して棚に置き、コテを開いて抜く。
3. 棚前に用意した返却本を同じコテで挟み、持ち上げる。
4. 全冊の実測占有範囲から、開いたコテが収まる空き区間を選び、棚に下ろす。
5. 開放・抜去後の安定と、周囲の本を動かしていないことを確認する。

今回の対象は**棚前の操作工程**です。離れた返却ボックスからの取り出し・台車の走行・貸出工程は含みません。
以前の部屋全体を使う双腕五指版は [SVH_REFERENCE.md](SVH_REFERENCE.md) に保存しています。
その版の理想把持と指定経路走行を、新しい接触把持の実証と混同しないでください。

| 用途 | コード・記録 |
| --- | --- |
| 返却の実行入口 | [s1_spatula_return.py](s1_spatula_return.py) |
| 手先・本・PhysX制御・工程・判定 | [dual_spatula_probe.py](dual_spatula_probe.py) |
| 貫通・持ち上げ・着座の追加評価 | [analyze_dual_spatula.py](analyze_dual_spatula.py) |
| 実測姿勢の全景＋手元描画 | [render_spatula_probe.py](render_spatula_probe.py) |
| 2枚コテの比較実験・失敗例 | [DUAL_SPATULA_STUDY.md](DUAL_SPATULA_STUDY.md) |
| 1枚コテの初期検討 | [SPATULA_STUDY.md](SPATULA_STUDY.md) |
| 以前の五指・双腕・移動デモ | [SVH_REFERENCE.md](SVH_REFERENCE.md)、[HISTORY.md](HISTORY.md) |

周囲の本も動的剛体です。未知の本の認識、柔らかい表紙やページの変形、損傷は評価していません。
S1は関節ドライブで動かし、台座は固定します。工具と手首の取付だけに固定ジョイントを使います。
公式モデルに初期から自己衝突形状の重なりがあるため、参考実行には明示的に
`--no-s1-self-collisions` を指定します。工具・本・棚・ロボットの外部接触は有効です。
全身の自己衝突を含む実機成立性と、未校正のモータ・摩擦パラメータは未検証です。

## 実行

通常はリポジトリのDocker環境内で非root実行します。
この作業環境ではユーザー承認により、既存Isaac Sim 6.0ランタイムを非rootで直接使っています。
既存環境を再インストールする必要はありません。空の出力先を指定してください。

```bash
python -m isaaclab_arena_examples.library.s1_spatula_return \
  --device cpu --viz none --no-s1-self-collisions \
  --output-dir /tmp/s1-spatula-return-new
python -m isaaclab_arena_examples.library.analyze_dual_spatula \
  /tmp/s1-spatula-return-new
python -m isaaclab_arena_examples.library.render_spatula_probe \
  /tmp/s1-spatula-return-new
```

最初に工程ごとの静止画を確認し、最後の描画コマンドへ `--video` を加えると等速動画を生成します。
描画は記録済みの実測姿勢を使用し、再シミュレーションや見栄えのための軌道補間を行いません。
`--angles 0` で直立本、`--dt 0.0020833333333333333` で480 Hzの確認ができます。
本が大きく傾く条件や把持力不足の失敗例は比較実験の文書に残しています。

この `/work` の既存ランタイムを使う場合は、非rootシェルで次の形に置き換えます。
キャッシュ先はそのユーザーが書き込めるディレクトリを指定します。

```bash
cd /work
export OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y
export XDG_CACHE_HOME=/tmp/library-return-cache WARP_CACHE_PATH=/tmp/library-return-warp
export LD_LIBRARY_PATH=/work/data/arena-runtime/sysroot/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
LIBRARY_PYTHON=/work/data/arena-runtime/python/cpython-3.12.15-linux-x86_64-gnu/bin/python3.12
"$LIBRARY_PYTHON" -c 'import sys,runpy; sys.path.append("/work/.venv/lib/python3.12/site-packages"); runpy.run_module("isaaclab_arena_examples.library.s1_spatula_return",run_name="__main__")' \
  --device cpu --viz none --no-s1-self-collisions --output-dir /tmp/s1-spatula-return-new
```

## 整理方針

採用候補の返却工程、2枚コテの比較実験、1枚コテの初期試験、旧五指デモを分けて辿れる構成にしています。
大きい動画・モデル・実測配列はGitに追加しません。古い実行は削除せず、新しい実行IDへ保存します。
成功した動画と検証記録を確認してから最新版を更新します。
