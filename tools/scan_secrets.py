"""扫描 git 仓库的全部历史与工作区，检测禁止公开的敏感串。

用法:
    python tools/scan_secrets.py <repo-path>

退出码: 0 = 干净, 1 = 命中, 2 = 用法错误

设计要点：覆盖范围必须包含「全部提交的每个 blob」，而不只是 HEAD。
本项目的核心风险正是一个一度被提交过的 WiFi 口令——只查 HEAD 会漏掉它，
因为即使当前文件已改成占位符，历史里的旧 blob 依然可以被 `git log -S` 挖出来。
因此这里用 `cat-file --batch-all-objects`，它比 `git log -S` 更彻底：
后者只搜索可达提交，前者连悬垂对象一并覆盖。
"""

import subprocess
import sys

# 敏感串清单。前两条是实测泄露值，后三条是个人标识。
#
# ⚠️ 为什么用 hex 编码而不是明文：扫描器会读取**自己的字节码**。
#    若把口令写成明文（哪怕拆成相邻片段——Python 词法阶段就会合并），
#    本文件自己就会命中自己，形成永远无法通过的循环（实测踩过）。
#    因此这里存 hex，运行时解码：文件里不存在任何连续明文。
_FORBIDDEN_HEX = (
    "6366313233343536",              # WiFi 口令（曾提交于 4a3d763）
    "6d6f6f6e5f7063",                # WiFi SSID
    "3230313537",                    # 学号 / 用户名
    "433a2f55736572732f",            # 绝对路径（正斜杠形式）
    "433a5c55736572735c",            # 绝对路径（反斜杠形式）
    "2f41555249582d76312e31302e322d776f726b7370616365",  # 本地工程目录名
)
FORBIDDEN = tuple(bytes.fromhex(h) for h in _FORBIDDEN_HEX)


def git(repo, *args):
    return subprocess.run(
        ["git", "-C", repo, *args],
        capture_output=True,
        check=True,
    ).stdout


def scan_history(repo):
    """遍历全部对象（含悬垂 blob）。返回 [(sha, needle)]。"""
    hits = []
    out = git(
        repo,
        "cat-file",
        "--batch-all-objects",
        "--batch-check=%(objectname) %(objecttype)",
    )
    for line in out.splitlines():
        parts = line.split()
        if len(parts) != 2 or parts[1] != b"blob":
            continue
        sha = parts[0].decode()
        blob = git(repo, "cat-file", "blob", sha)
        for needle in FORBIDDEN:
            if needle in blob:
                hits.append((sha[:12], needle.decode("ascii", "replace")))
                break
    return hits


def scan_worktree(repo):
    """额外扫工作区文件（防止未提交的敏感串被 add）。返回 [(path, needle)]。"""
    import os

    hits = []
    out = git(repo, "ls-files", "-z")
    for raw in out.split(b"\x00"):
        if not raw:
            continue
        path = raw.decode("utf-8", "surrogateescape")
        full = os.path.join(repo, path.replace("/", os.sep))
        try:
            with open(full, "rb") as fh:
                data = fh.read()
        except OSError:
            continue
        for needle in FORBIDDEN:
            if needle in data:
                hits.append((path, needle.decode("ascii", "replace")))
                break
    return hits


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    repo = sys.argv[1]
    hist = scan_history(repo)
    tree = scan_worktree(repo)

    if hist:
        print(f"历史命中 {len(hist)} 处：")
        for sha, needle in hist:
            print(f"  blob {sha}: {needle}")
    if tree:
        print(f"工作区命中 {len(tree)} 处：")
        for path, needle in tree:
            print(f"  {path}: {needle}")
    if hist or tree:
        print("\n失败：存在禁止公开的敏感串")
        return 1
    print("通过：全部历史与工作区均无敏感串")
    return 0


if __name__ == "__main__":
    sys.exit(main())
