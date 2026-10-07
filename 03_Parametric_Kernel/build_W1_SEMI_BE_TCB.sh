#!/usr/bin/env bash
set -e

# 编译 C++ 几何内核 (OpenCASCADE 7.x+)
g++ -O3 -std=c++17 -fopenmp -Wno-deprecated-declarations \
    gen_W1_SEMI_BE_TCB_RevD.cpp -o run_wok_tcb_revd \
    -I/usr/include/opencascade \
    -lTKernel -lTKMath -lTKG2d -lTKG3d -lTKGeomBase -lTKBRep \
    -lTKTopAlgo -lTKPrim -lTKBO -lTKFillet \
    -lTKSTEP -lTKSTEPBase -lTKSTEPAttr -lTKXSBase

echo "[+] Compilation successful. Executing STEP compilation..."
./run_wok_tcb_revd
