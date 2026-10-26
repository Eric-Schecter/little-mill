#!/bin/sh
set -eu
cd "$(dirname "$0")"
em++ -fPIC -O3 -DNDEBUG -pthread -sSIDE_MODULE=1 -sWASM_BIGINT -sSUPPORT_LONGJMP=wasm -fvisibility=hidden -shared -s -Wl,-soname,cnc_manifold_web.wasm -o cnc_manifold_web.new.wasm extension.cpp.o libgodot-cpp.a libmanifold.a libocl.a libclipper.a
printf '%s\n' 'Replace site/cnc_manifold_web.wasm with cnc_manifold_web.new.wasm to use the relinked module.'
