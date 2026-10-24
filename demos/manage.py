# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Catalog local simulation artifacts without importing Isaac Sim."""

import argparse
import json
import re
import shutil
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def catalog():
    """Return registered demos after checking their code entry points."""
    data = json.loads((ROOT / "demos/catalog.json").read_text())
    assert data["schema_version"] == 1, "Unsupported catalog version."
    demos = {}
    for demo in data["demos"]:
        name = identifier(demo["id"])
        assert name not in demos, f"Duplicate demo: {name}"
        for key in ("code", "readme"):
            path = (ROOT / demo[key]).resolve()
            assert path.is_relative_to(ROOT) and path.exists(), f"Missing {key}: {path}"
        for key in ("simulation", "renderer"):
            assert (ROOT / (demo[key].replace(".", "/") + ".py")).is_file(), demo[key]
        assert Path(demo["video"]).name == demo["video"], "Video must be a filename."
        demos[name] = demo
    return demos


def identifier(value):
    """Return a safe single-component demo or run identifier."""
    assert re.fullmatch(r"[a-z0-9][a-z0-9_-]*", value), f"Invalid ID: {value}"
    return value


def runs_dir(outputs, demo):
    """Return a run directory confined to the selected output root."""
    path = outputs / "demos" / identifier(demo) / "runs"
    assert path.resolve().is_relative_to(outputs.resolve()), "Run path escapes output root."
    return path


def latest_run(outputs, demo):
    """Return the selected run, or None if no latest run is registered."""
    runs = runs_dir(outputs, demo)
    selection = runs.parent / "latest.json"
    if selection.is_file():
        data = json.loads(selection.read_text())
        assert data["schema_version"] == 1, "Unsupported latest selection version."
        path = runs / identifier(data["run"])
        assert path.resolve().parent == runs.resolve() and path.is_dir(), f"Invalid latest run: {path}"
        return path
    # Read old installations, but new registrations never create symlinks.
    link = runs.parent / "latest"
    if link.is_symlink():
        path = link.resolve()
        assert path.parent == runs.resolve() and path.is_dir(), f"Invalid latest link: {link}"
        return path
    assert not link.exists(), f"Expected a legacy latest symlink: {link}"
    return None


def successful(path, demo):
    """Return whether the run has a successful result and a nonempty video."""
    report = path / "result.json"
    video = path / demo["video"]
    return (
        report.is_file()
        and json.loads(report.read_text()).get("success") is True
        and video.is_file()
        and video.stat().st_size > 0
    )


def register(outputs, demo, source, run_id, promote=False, note=""):
    """Copy an immutable run and optionally select it as latest.

    Args:
        outputs: Local artifact root.
        demo: Catalog entry.
        source: Existing capture directory.
        run_id: Unique run identifier.
        promote: Select only successful video runs as latest.
        note: Human-readable provenance or limitations.
    """
    source = source.resolve()
    assert source.is_dir(), f"Missing capture: {source}"
    target = runs_dir(outputs, demo["id"]) / identifier(run_id)
    assert not target.exists() and not target.is_symlink(), f"Run already exists: {target}"
    assert not target.resolve().is_relative_to(source), "Destination cannot be inside source."
    if promote:
        assert successful(source, demo), "Latest requires result.json success=true and a nonempty video."
    latest_run(outputs, demo["id"])
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source, target)
    # Preserve any original manifest. Registration time does not identify simulation source revision.
    manifest = dict(
        schema_version=1,
        demo=demo["id"],
        run=run_id,
        registered_at=datetime.now(timezone.utc).isoformat(),
        imported_from=str(source),
        note=note,
        source_provenance="See source_manifest.json if present; otherwise unknown.",
    )
    (target / "registration.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    if promote:
        selection = target.parent.parent / "latest.json"
        temporary = selection.with_name(".latest-pending.json")
        assert not temporary.exists() and not temporary.is_symlink(), f"Pending promotion exists: {temporary}"
        temporary.write_text(json.dumps(dict(schema_version=1, run=run_id), indent=2) + "\n")
        temporary.replace(selection)
        legacy = selection.with_name("latest")
        if legacy.is_symlink():
            legacy.unlink()
    return target


def write_index(outputs, demos):
    """Generate an artifact index linking code, instructions and latest captures."""
    lines = [
        "# シミュレーション一覧",
        "",
        "この一覧は `demos/manage.py index` で生成します。",
        "",
        "**最初に見るもの：[最新の図書館デモを再生](library/video.mp4)**（2枚コテ・移動・返却・貸出の全工程）。",
        "",
        "| 見たいもの | 開く場所 |",
        "| --- | --- |",
        "| 最新の図書館デモと代表画像・検証概要 | [図書館デモのページ](library/README.md) |",
        "| これまでのデモ・比較実験の動画 | [過去も含めた動画一覧](previews/README.md) |",
        "| 物理記録や過去データを再解析 | [アーカイブ一覧・復元方法](archive/README.md) |",
        "",
        demos.get("library-s1-mobile-spatula", {}).get(
            "improvement_status", "図書館の改善状況：容量は軽量化済み。カクつき・動作の遅さは未改善です。"
        ),
        "",
        f"[管理方法・新規デモの追加]({ROOT / 'demos/README.md'})",
        "",
        "| デモ | 状態 | コード・手順 | 最新の保存動画 |",
        "| --- | --- | --- | --- |",
    ]
    for name, demo in demos.items():
        run = latest_run(outputs, name)
        video = f"[動画]({(run / demo['video']).relative_to(outputs)})" if run else "未登録"
        if run is None and demo.get("archive"):
            archived = outputs / demo["archive"]
            assert archived.resolve().is_relative_to(outputs.resolve()), "Archive escapes output root."
            if archived.is_file():
                video = f"[圧縮保管]({archived.relative_to(outputs)})"
        if run and demo.get("view_video") and demo.get("view_run") == run.name:
            view = outputs / demo["view_video"]
            assert view.resolve().is_relative_to(outputs.resolve()), "View video escapes output root."
            if view.is_file():
                video = f"[軽量動画]({view.relative_to(outputs)})"
        lines.append(
            f"| {demo['title']} | {demo['status']} | [コード]({ROOT / demo['code']})・[手順]({ROOT / demo['readme']}) |"
            f" {video} |"
        )
    for name, demo in demos.items():
        lines.extend([
            "",
            f"## {demo['title']}",
            "",
            demo["notes"],
            "",
            f"実行: `{demo['simulation']}`  / 描画: `{demo['renderer']}`",
        ])
        run = latest_run(outputs, name)
        if run:
            rel = run.relative_to(outputs)
            lines.extend(["", f"保存実行: `{rel}`"])
            registration = run / "registration.json"
            if registration.exists():
                note = json.loads(registration.read_text()).get("note")
                if note:
                    lines.extend(["", note])
            for filename in (
                "result.json",
                "verification.json",
                "source_manifest.json",
                "registration.json",
                "return_insert.png",
                "loan_extract.png",
                "preview-last.png",
            ):
                if (run / filename).is_file():
                    lines.append(f"- [{filename}]({rel / filename})")
    lines.extend([
        "",
        "## 保存方針",
        "",
        "コード・カタログ・手順はGit管理し、動画・実測記録・モデルはGitに追加しません。",
        (
            "各デモは `demos/<ID>/runs/<実行ID>/` に保存し、`latest.json` は確認済みの実行IDを保持します。"
            "シンボリックリンクは新規作成しません。"
        ),
        "再実行は新しい実行IDで保存します。登録操作は既存の実行を上書き・削除しません。",
        "過去の実験・旧版は [圧縮アーカイブ](archive/README.md)、調査の索引は `library-review/` にあります。",
        "",
    ])
    outputs.mkdir(parents=True, exist_ok=True)
    (outputs / "README.md").write_text("\n".join(lines))


def main():
    """List, register, validate or index local demo runs."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outputs", type=Path, default=ROOT / "outputs")
    sub = parser.add_subparsers(dest="command", required=True)
    for command in ("list", "index", "check"):
        sub.add_parser(command)
    add = sub.add_parser("register")
    add.add_argument("demo")
    add.add_argument("source", type=Path)
    add.add_argument("--run", required=True)
    add.add_argument("--promote", action="store_true")
    add.add_argument("--note", default="")
    args = parser.parse_args()
    outputs = args.outputs.resolve()
    demos = catalog()
    if args.command == "register":
        assert args.demo in demos, f"Unknown demo: {args.demo}"
        print(register(outputs, demos[args.demo], args.source, args.run, args.promote, args.note))
        write_index(outputs, demos)
    elif args.command == "index":
        write_index(outputs, demos)
        print(outputs / "README.md")
    else:
        for name, demo in demos.items():
            run = latest_run(outputs, name)
            if args.command == "check" and run:
                assert successful(run, demo), f"Invalid latest run: {run}"
            if args.command == "check" and not run and demo.get("archive"):
                archived = outputs / demo["archive"]
                assert archived.resolve().is_relative_to(outputs.resolve()), "Archive escapes output root."
                assert archived.is_file() and archived.stat().st_size > 0, f"Missing archive: {archived}"
            location = str(run) if run else ("archived: " + demo["archive"] if demo.get("archive") else "no saved run")
            print(f"{name}: {demo['status']} | {location}")


if __name__ == "__main__":
    main()
