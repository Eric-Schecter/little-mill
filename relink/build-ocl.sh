#!/bin/sh
set -eu
cd "$(dirname "$0")"
mkdir -p work
cd work
tar -xzf ../opencamlib-source.tar.gz
tar -xzf ../boost-headers-1.85.0.tar.gz
mkdir -p objects
for source in opencamlib/src/algo/*.cpp opencamlib/src/geo/*.cpp opencamlib/src/cutters/*.cpp opencamlib/src/dropcutter/*.cpp opencamlib/src/common/*.cpp; do
 object=$(printf '%s' "$source" | tr '/' '_').o
 em++ -std=c++17 -O3 -DNDEBUG -pthread -fPIC -I. -Iopencamlib/src -Iopencamlib/src/algo -Iopencamlib/src/geo -Iopencamlib/src/cutters -Iopencamlib/src/dropcutter -Iopencamlib/src/common -c "$source" -o "objects/$object"
done
emar rcs ../libocl.a objects/*.o
