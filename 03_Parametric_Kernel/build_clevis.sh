cat << 'EOF' > build.sh
#!/usr/bin/env bash
set -euo pipefail

# Build Configuration
SRC="clevis_parametric_generator.cpp"
BIN="clevis_generator_bin"
THREADS="${OMP_NUM_THREADS:-8}"
export OMP_NUM_THREADS="${THREADS}"

echo "=========================================================="
echo " Building Defensive Publication Reference Implementation"
echo " Target: ${BIN} (Parallel Threads: ${THREADS})"
echo "=========================================================="

# Auto-detect OpenCASCADE headers
OCCT_INC="/usr/include/opencascade"
if [ ! -d "${OCCT_INC}" ]; then
    echo "[ERROR] OpenCASCADE header directory not found at ${OCCT_INC}"
    exit 1
fi

g++ -O3 -std=c++17 -fopenmp -Wno-deprecated-declarations "${SRC}" -o "${BIN}" \
    -I"${OCCT_INC}" \
    -lTKernel -lTKMath -lTKG2d -lTKG3d -lTKGeomBase -lTKBRep \
    -lTKTopAlgo -lTKPrim -lTKBO -lTKFillet \
    -lTKSTEP -lTKSTEPBase -lTKSTEPAttr -lTKXSBase

echo "[+] Compilation successful. Executing generator..."
"./${BIN}"

echo "=========================================================="
echo "[COMPLETE] Output verified: clevis_bracket_standard_25x25.step"
echo "=========================================================="
EOF

chmod +x build.sh
./build.sh