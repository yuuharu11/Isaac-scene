# シミュレーションの管理

デモを探す入口はこのページと、管理ツールが生成する `outputs/README.md` です。
コードと手順はGitで管理し、大きい動画・実測記録・モデルはGit管理外に置きます。

## 登録済みデモ

| ID | 用途 | コード・実行手順 |
| --- | --- | --- |
| `library-s1` | 開発中の図書館タスク | [S1の手順](../isaaclab_arena_examples/library/README.md#s1による図書館タスク) |
| `bento` | 弁当の盛り付け | [弁当の手順](../isaaclab_arena_examples/bento/README.md) |
| `galbot-g1` | G1系の導入確認・比較用 | [G1の手順](../isaaclab_arena_examples/library/HISTORY.md#galbot導入用ブランチ昇降双腕の基礎制御) |
| `library-nhc12` | 旧図書館実装・参照用 | [旧版の手順](../isaaclab_arena_examples/library/HISTORY.md) |

S1の旧動画は削除済みです。弁当で現存する動画は旧吸着版です。
現在のトング版の映像と混同しないよう、保存実行に注記を付けています。
既存のコードはモジュールパスを保っています。弁当の `capture.py` と `render_video.py` は
図書館からも利用する共通処理を含むため、弁当フォルダを丸ごと削除しないでください。

## 保存先

```text
<outputs>/
  README.md                    # 全デモの一覧（自動生成）
  demos/
    library-s1/
      latest -> runs/20261022-multiview
      runs/
        20261022-multiview/
          s1-library.mp4
          result.json          # シミュレーションの成功判定
          capture.json         # 記録条件
          scene.npz            # 実測姿勢と形状
          verification.json    # 独立検証（ある場合）
          source_manifest.json # 実行ソースの対応表（ある場合）
          registration.json    # 登録日時・由来・注記
          *.png
    bento/
    galbot-g1/
```

この環境では共有成果物ルートを `/work/outputs` にしています。
管理ツールの既定値は、このチェックアウト内の `outputs/` です。
ワークツリー利用時は `--outputs /work/outputs` を明示してください。
過去の動画リンクは互換リンクとして残しますが、新規保存は上の配置にそろえます。

## 一覧・検証

管理ツールは標準Pythonだけで動き、Isaac Simを起動しません。
以下は管理コードのあるチェックアウトのルートで実行します。

```bash
python3 demos/manage.py --outputs /work/outputs list
python3 demos/manage.py --outputs /work/outputs check
python3 demos/manage.py --outputs /work/outputs index
```

`check` はコードの参照先と、最新版のリンク・動画・`result.json` の成功値を確認します。
動画の全フレームや物理動作を再検証するコマンドではありません。
このコンテナで `python3` が使えない場合、既存のPython実体を使います。

```bash
/work/data/arena-runtime/python/cpython-3.12.15-linux-x86_64-gnu/bin/python3.12 \
  demos/manage.py --outputs /work/outputs list
```

## 実行を追加する

1. 各デモのREADMEに従い、既存コンテナ内・非rootでシミュレーションを新規の一時保存先へ実行する。
2. 代表姿勢の静止画と必要な動作・接触を検証してから、動画を生成・確認する。
3. 完成したキャプチャフォルダを次のコマンドで登録する。

```bash
python3 demos/manage.py --outputs /work/outputs register library-s1 \
  /tmp/my-library-capture --run 20261022-trial02 \
  --note '変更点・検証内容・残る制限' --promote
```

登録はフォルダのコピーです。元のキャプチャは削除せず、同じ実行IDへの上書きもしません。
`--promote` は動画が存在し、`result.json` の `success` が `true` の実行だけを最新版にします。
実機同等の成功を保証する判定ではないので、映像と検証内容も確認してください。
失敗記録や未確認の実行は `--promote` なしで保存し、最新版と分けて管理します。
実行に使ったソースは `source_manifest.json` などで別途記録します。
登録時のコードを過去の実行ソースとみなすことはしません。

古い実行を消すときは、各デモの `latest` が指す実行を残し、不要な `runs/<実行ID>` だけを削除します。
互換リンクや文書から参照されていないかも確認し、最後に `index` と `check` を実行します。
自動削除は行いません。

## 新しいシミュレーションを追加する

1. `isaaclab_arena_examples/<名前>/` にコードとREADMEを置く。
2. [catalog.json](catalog.json) の `demos` にID・タイトル・状態・コード・README・実行/描画モジュール・動画ファイル名・注意点を追加する。
3. 成功判定をキャプチャ内の `result.json` に `{"success": true}` として保存する。
4. `check` でコードの参照先を確認し、動画生成後に `register` する。

状態は `active`（開発対象）、`maintained`（保持中）、`reference`（比較用）、`legacy`（旧実装）を使います。
未登録のデモには動画リンクを作りません。原本のあるモデル・データは移動せず、マウント先を各READMEに記載してください。
