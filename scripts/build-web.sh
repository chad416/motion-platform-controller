#!/usr/bin/env bash
# Builds the browser demo: mcue/ and web/mcue_web.c to web/mcue.wasm with
# clang's wasm32 target (no Emscripten, no JavaScript glue). The module has no
# imports: libc comes from wasi-libc, and nothing in mcue/ does I/O.
#
#   sudo apt-get install clang lld wasi-libc libclang-rt-18-dev-wasm32   # Ubuntu 24.04
#   bash scripts/build-web.sh
#   python3 -m http.server -d web 8000    # then open http://localhost:8000
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CC="${WASM_CC:-clang}"

"$CC" --target=wasm32-wasi -O2 -std=c11 -Wall -Wextra -Werror \
	-nostartfiles -Wl,--no-entry -Wl,-z,stack-size=262144 -Wl,--strip-all \
	-I"$REPO/mcue/include" \
	"$REPO"/mcue/src/*.c "$REPO/web/mcue_web.c" -lm \
	-o "$REPO/web/mcue.wasm"

echo "web/mcue.wasm: $(wc -c <"$REPO/web/mcue.wasm") bytes"
