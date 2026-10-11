#!/usr/bin/env bash
# ==============================================================================
# PIPELINE RUNNER : WUL_MANIFOLD.sh
# COMPONENT ID   : WUL_MANIFOLD_RevA (Taxonomy 7.1)
# DESCRIPTION    : Production-Grade Semiconductor Process Gas Manifold
# KERNEL ENGINE   : OpenCASCADE Technology (OCCT)
# ==============================================================================
set -euo pipefail

# 1. Resolve working directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

TARGET_SRC="WUL_MANIFOLD_RevA.cpp"
TARGET_BIN="run_wok_manifold_builder"
OUTPUT_STEP="WUL_MANIFOLD_RevA.STEP"
OUTPUT_PDF="WUL_MANIFOLD_RevA_2.pdf"

echo "=========================================================="
echo "[+] Initializing Build Pipeline for ${TARGET_SRC}"
echo "[+] Working Directory: ${SCRIPT_DIR}"
echo "=========================================================="

if [ ! -f "${TARGET_SRC}" ]; then
    echo "[ERROR] Source file ${TARGET_SRC} not found!"
    exit 1
fi

# 2. Auto-detect OpenCASCADE development headers
OCCT_INC=""
POSSIBLE_INC_PATHS=(
    "/usr/include/opencascade"
    "/usr/local/include/opencascade"
    "/usr/include/oce"
    "/opt/opencascade/inc"
)

for inc_path in "${POSSIBLE_INC_PATHS[@]}"; do
    if [ -f "${inc_path}/gp_Pnt.hxx" ]; then
        OCCT_INC="${inc_path}"
        echo "[+] Found OpenCASCADE headers at: ${OCCT_INC}"
        break
    fi
done

if [ -z "${OCCT_INC}" ]; then
    echo "[ERROR] OpenCASCADE headers not found in standard system paths!"
    echo "[!] Run: sudo apt-get update && sudo apt-get install -y libocct-*-dev"
    exit 1
fi

# 3. Native compiler flags and linker libraries
CXX="g++"
CXXFLAGS="-O3 -std=c++17 -Wall -Wextra"
INCLUDES="-I${OCCT_INC}"
LIBS="-lTKernel -lTKMath -lTKG2d -lTKG3d -lTKGeomBase -lTKBRep -lTKTopAlgo -lTKPrim -lTKBO -lTKBool -lTKFillet -lTKSTEP -lTKSTEPBase -lTKSTEPAttr -lTKXSBase"

# 4. Compile native binary
rm -f "${TARGET_BIN}"
echo "[+] Compiling C++ parametric geometry kernel..."
${CXX} ${CXXFLAGS} ${INCLUDES} "${TARGET_SRC}" -o "${TARGET_BIN}" ${LIBS}

if [ ! -f "${TARGET_BIN}" ]; then
    echo "[ERROR] Compilation failed! Binary not generated."
    exit 1
fi
echo "[SUCCESS] Native builder generated: ./${TARGET_BIN}"

# 5. Execute kernel to generate 3D CAD STEP model
echo "[+] Executing geometry kernel..."
./"${TARGET_BIN}" "${SCRIPT_DIR}/${OUTPUT_STEP}"

# 6. Extract cryptographic SHA-256 fingerprints
echo ""
echo "=========================================================="
echo "[CRYPTO-VERIFICATION AUDIT ASSETS]"
echo "=========================================================="

if [ -f "${OUTPUT_STEP}" ] && [ -s "${OUTPUT_STEP}" ]; then
    STEP_SIZE=$(wc -c < "${OUTPUT_STEP}")
    STEP_HASH=$(sha256sum "${OUTPUT_STEP}" | awk '{print $1}')
    echo "[ASSET] 3D CAD STEP MODEL"
    echo "  File : ${OUTPUT_STEP}"
    echo "  Size : ${STEP_SIZE} bytes"
    echo "  Hash : ${STEP_HASH}"
else
    echo "[ERROR] Model file ${OUTPUT_STEP} missing or empty!"
    exit 1
fi

if [ -f "${OUTPUT_PDF}" ] && [ -s "${OUTPUT_PDF}" ]; then
    PDF_SIZE=$(wc -c < "${OUTPUT_PDF}")
    PDF_HASH=$(sha256sum "${OUTPUT_PDF}" | awk '{print $1}')
    echo "[ASSET] 2D MANUFACTURING DRAWING"
    echo "  File : ${OUTPUT_PDF}"
    echo "  Size : ${PDF_SIZE} bytes"
    echo "  Hash : ${PDF_HASH}"
elif [ -f "WUL_MANIFOLD_RevA.pdf" ] && [ -s "WUL_MANIFOLD_RevA.pdf" ]; then
    PDF_SIZE=$(wc -c < "WUL_MANIFOLD_RevA.pdf")
    PDF_HASH=$(sha256sum "WUL_MANIFOLD_RevA.pdf" | awk '{print $1}')
    echo "[ASSET] 2D MANUFACTURING DRAWING (Legacy Name)"
    echo "  File : WUL_MANIFOLD_RevA.pdf"
    echo "  Size : ${PDF_SIZE} bytes"
    echo "  Hash : ${PDF_HASH}"
fi

echo "=========================================================="
echo "[SUCCESS] WUL_MANIFOLD_RevA build pipeline completed."
echo "=========================================================="