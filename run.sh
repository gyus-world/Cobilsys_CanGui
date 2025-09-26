#!/bin/bash
set -euo pipefail

# 스크립트 위치로 이동
cd "$(dirname "$0")"

# 실행권한 보장(실패해도 무시)
chmod +x Cobilsys_CanGui 2>/dev/null || true

# 터미널과 완전히 분리해 실행 (표준입출력/에러 모두 끊고 백그라운드)
setsid ./Cobilsys_CanGui >/dev/null 2>&1 </dev/null &

# 바로 종료 (런처 스크립트는 남지 않게)
exit 0
