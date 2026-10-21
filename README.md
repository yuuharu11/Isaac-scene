# Isaac-scene

Isaac Lab-Arena を土台に、Galbot S1 や Yaskawa NHC12 を使った操作デモを追加している
開発用リポジトリです。上流（isaac-sim/IsaacLab-Arena）の公式リポジトリでは
ありません。

## 土台

| 項目 | 内容 |
|---|---|
| ベース | Isaac Lab-Arena `release/0.3.0`（コミット `8737b4ce`） |
| Isaac Sim | 6.0 |
| Isaac Lab | 3.0 |
| Python | 3.12 以上 |
| ライセンス | Apache 2.0（[LICENSE.md](LICENSE.md)）。Isaac Sim 本体は NVIDIA のライセンスに従います |

土台の導入方法や概念は、上流のドキュメントを参照してください。

- [上流ドキュメント](https://isaac-sim.github.io/IsaacLab-Arena/release/0.3.0/index.html)
- [上流リポジトリ](https://github.com/isaac-sim/IsaacLab-Arena)

## このリポジトリで追加したもの

上流に、次の独立したサンプルを追加しています。いずれも Arena の環境
レジストリには登録しておらず、学習済みポリシーは使いません。
物理は CPU PhysX、把持は理想的な固定拘束による近似です。

| サンプル | 内容 | 説明 |
|---|---|---|
| 図書館（Galbot S1・双腕五指） | 返却本の収納と高い棚からの貸出 | [S1 の README](isaaclab_arena_examples/library/README.md) |
| 弁当盛り付け | NHC12 単腕が4品目を弁当箱の区画へ移す | [bento の README](isaaclab_arena_examples/bento/README.md) |
| 図書館（単腕・移動） | 返却本の収納と依頼本の貸出 | [旧NHC12版の手順](isaaclab_arena_examples/library/HISTORY.md) |
| 図書館（固定・双腕） | 隣接本を押して隙間を広げる密集棚の試作 | 同上 |
| 図書館（移動・双腕） | 台車で棚間を移動する双腕の試作 | 同上 |

回帰テストは `isaaclab_arena_examples/tests/` にあります
（`test_bento_placement.py`、`test_bento_capture.py`、`test_library_placement.py`）。
配置判定とカタログ整合性のテストはシミュレーションなしで実行できます。

## デモと成果物の管理

[共通カタログ](demos/README.md)にコード・実行手順・動画と結果の保存方法をまとめています。
生成した一覧は `outputs/README.md`、動画・実測記録・モデルはGit管理外です。
S1が現在の開発対象で、NHC12版とG1系の導入確認は参照用に保持しています。

## 実行例

導入済みの開発コンテナ内で、リポジトリルートから実行します。
コンテナ内では `python` の代わりに `/isaac-sim/python.sh` を使います。

```bash
# 弁当盛り付け（成功時は終了コード 0）
/isaac-sim/python.sh -m isaaclab_arena_examples.bento.simulation \
  --device cpu --headless --viz none \
  --output /tmp/yaskawa_bento_result.json

# 図書館・単腕移動版
/isaac-sim/python.sh -m isaaclab_arena_examples.library.run_library \
  --device cpu --headless --viz none --output /tmp/library_result.json

# 配置判定のテストだけ（シミュレーション不要）
python -m pytest -q \
  isaaclab_arena_examples/tests/test_bento_placement.py \
  isaaclab_arena_examples/tests/test_library_placement.py
```

各サンプルの前提、判定条件、近似の範囲は、それぞれの README を参照してください。

## ディレクトリ構成

```text
.
├── isaaclab_arena/               # 上流のコア（環境・タスク・評価）
├── isaaclab_arena_environments/  # 上流の環境定義
├── isaaclab_arena_examples/      # 上流の例＋追加サンプル（bento / library）
├── isaaclab_arena_g1/            # 上流の拡張（G1）
├── isaaclab_arena_gr00t/         # 上流の拡張（GR00T）
├── isaaclab_arena_openpi/        # 上流の拡張（OpenPI）
├── docker/                       # 開発コンテナ
├── docs/                         # 上流の Sphinx ドキュメント
└── submodules/                   # Isaac Lab 等のサブモジュール
```

## ローカルファイルについて

- ルートの `data/` と `outputs/` は `.gitignore` で無視しています。
  データセットや実行成果物はコミットしません。
- 実行結果の JSON や動画は、コマンドの引数で指定した場所に出力されます。

## 注意

- 土台の Isaac Lab-Arena は alpha（v0.3）で、API は変更されることがあります。
- 追加サンプルは固定条件のデモです。摩擦把持、障害物回避、未知の物体配置への
  対応を保証するものではありません。
- 上流側の問題は上流リポジトリへ、このリポジトリの追加分への問い合わせは
  本リポジトリの Issue へお願いします。

## 謝辞

本リポジトリは [NVIDIA Isaac Lab](https://github.com/isaac-sim/IsaacLab) 上の
Isaac Lab-Arena を土台としています。引用が必要な場合は、上流 README の
Citation を参照してください。
