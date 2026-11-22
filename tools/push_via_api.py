# -*- coding: utf-8 -*-
"""GitHub Git Data API 同步脚本 —— 当 git push 被网络重置时用它推送改动。

用法：python tools/push_via_api.py
凭证：从 git credential（GitHub 登录缓存）里读，不在代码里存 token。
"""
import base64
import json
import os
import subprocess
import urllib.error
import urllib.request

OWNER = "zcy-t"
REPO = "ros2-mini-sentry-homework"
LOCAL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
API = "https://api.github.com"
TEXT_EXT = (".sh", ".md", ".txt", ".py", ".cpp", ".hpp", ".xml", ".yaml", ".html", ".launch")


def token():
    out = subprocess.run(["git", "credential", "fill"],
                         input="protocol=https\nhost=github.com\n\n",
                         capture_output=True, text=True, check=False).stdout
    for line in out.splitlines():
        if line.startswith("password="):
            return line.split("=", 1)[1].strip()
    raise SystemExit("找不到 GitHub 凭证，请先 git push 一次或重新登录")


HDRS = {"Authorization": "Bearer " + token(), "Accept": "application/vnd.github+json",
        "User-Agent": "doubao-push", "Content-Type": "application/json"}


def call(method, path, payload=None, soft=False):
    data = None if payload is None else json.dumps(payload, ensure_ascii=False).encode("utf-8")
    req = urllib.request.Request(API + path, data=data, headers=HDRS, method=method)
    try:
        with urllib.request.urlopen(req, timeout=90) as r:
            return json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        if soft:
            return None
        raise SystemExit(f"HTTP {e.code} {method} {path}: {e.read().decode('utf-8', 'replace')[:400]}")


def read(rel):
    with open(os.path.join(LOCAL, rel), "rb") as f:
        raw = f.read()
    return raw.replace(b"\r\n", b"\n") if rel.endswith(TEXT_EXT) else raw


def main():
    files = subprocess.run(["git", "-C", LOCAL, "-c", "core.quotepath=false", "ls-files", "-z"],
                           capture_output=True, text=True, check=True).stdout.split("\0")
    files = [f.replace("\\", "/") for f in files if f.strip()]
    message = subprocess.run(["git", "-C", LOCAL, "log", "-1", "--pretty=%s"],
                             capture_output=True, text=True, check=True).stdout.strip() or "更新作业内容"

    head = call("GET", f"/repos/{OWNER}/{REPO}/commits?per_page=1", soft=True)
    if not head:
        init = call("PUT", f"/repos/{OWNER}/{REPO}/contents/.gitignore",
                    {"message": "初始化仓库", "branch": "main",
                     "content": base64.b64encode(read(".gitignore")).decode("ascii")})
        parent = init["commit"]["sha"]
    else:
        parent = head[0]["sha"]
    base_tree = call("GET", f"/repos/{OWNER}/{REPO}/git/commits/{parent}")["tree"]["sha"]

    entries = []
    for rel in files:
        raw = read(rel)
        blob = call("POST", f"/repos/{OWNER}/{REPO}/git/blobs",
                    {"content": base64.b64encode(raw).decode("ascii"), "encoding": "base64"})
        entries.append({"path": rel, "mode": "100644", "type": "blob", "sha": blob["sha"]})

    tree = call("POST", f"/repos/{OWNER}/{REPO}/git/trees", {"base_tree": base_tree, "tree": entries})
    commit = call("POST", f"/repos/{OWNER}/{REPO}/git/commits",
                  {"message": message, "tree": tree["sha"], "parents": [parent]})
    call("PATCH", f"/repos/{OWNER}/{REPO}/git/refs/heads/main", {"sha": commit["sha"], "force": True})
    print("已推送", len(entries), "个文件：", commit["html_url"])


if __name__ == "__main__":
    main()
