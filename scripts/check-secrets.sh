#!/usr/bin/env bash
# 提交前密钥扫描：已跟踪文件 + 工作区（排除模型/二进制/构建产物）。
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
PATTERN='(sk-[A-Za-z0-9]{20,}|AKID[A-Za-z0-9]{12,}|(api[_-]?key|secret|token)[[:space:]]*[:=][[:space:]]*["'"'"'][^"'"'"']{16,})'
status=0

echo "=== tracked files ==="
if git grep -nEI "$PATTERN" -- . 2>/dev/null | grep -v -E '\.env\.example|README|\.md:'; then
  echo "!! tracked files contain possible secrets"; status=1
else
  echo "clean"
fi
if git ls-files --error-unmatch .env >/dev/null 2>&1; then
  echo "!! .env is TRACKED and must be removed from git"; status=1
fi

echo "=== worktree (excluding .env/models/build/install/.git) ==="
if grep -rInE "$PATTERN" . \
    --exclude-dir=.git --exclude-dir=MODELS --exclude-dir=build \
    --exclude-dir=install --exclude-dir=log \
    --exclude='.env' --exclude='.env.*' \
    --exclude='*.onnx' --exclude='*.pt' --exclude='*.so*' \
    --exclude='*.STL' --exclude='*.stl' 2>/dev/null \
    | grep -v -E '\.env\.example|README|\.md:'; then
  echo "!! worktree contains possible secrets"; status=1
else
  echo "clean"
fi

exit $status
