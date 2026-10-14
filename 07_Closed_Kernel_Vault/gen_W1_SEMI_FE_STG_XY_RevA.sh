#!/usr/bin/env bash
# ==============================================================================
# WOLFF3D OMNIKERNEL (WOK) PIPELINE RUNNER
# TAXONOMY ID    : 1.1 SEMICONDUCTOR FRONT-END EQUIPMENT
# COMPONENT ID   : W1-SEMI-FE-STG-XY_RevA
# REVISION       : Rev A (Production Baseline)
# KERNEL ENGINE   : OpenCASCADE Technology (OCCT)
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

TARGET_SRC="gen_W1_SEMI_FE_STG_XY_RevA.cpp"
TARGET_BIN="run_wok_semi_fe_builder"
OUTPUT_STEP="W1-SEMI-FE-STG-XY_RevA_3.STEP"
OUTPUT_PDF="W1-SEMI-FE-STG-XY_RevA_3.pdf"

echo "=========================================================="
echo "[+] Initializing Build Pipeline for Taxonomy 1.1"
echo "[+] Target Source : ${TARGET_SRC}"
echo "=========================================================="

if [ ! -f "${TARGET_SRC}" ]; then
    echo "[ERROR] Source file ${TARGET_SRC} not found!"
    exit 1
fi

# Detect OpenCASCADE include path
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
        break
    fi
done

if [ -z "${OCCT_INC}" ]; then
    echo "[ERROR] OpenCASCADE headers not found!"
    exit 1
fi

CXX="g++"
CXXFLAGS="-O3 -std=c++17 -Wall -Wextra"
INCLUDES="-I${OCCT_INC}"
LIBS="-lTKernel -lTKMath -lTKG2d -lTKG3d -lTKGeomBase -lTKBRep -lTKTopAlgo -lTKPrim -lTKBO -lTKBool -lTKFillet -lTKSTEP -lTKSTEPBase -lTKSTEPAttr -lTKXSBase"

echo "[+] Compiling C++ parametric geometry kernel..."
${CXX} ${CXXFLAGS} ${INCLUDES} "${TARGET_SRC}" -o "${TARGET_BIN}" ${LIBS}

echo "[+] Executing geometry kernel to generate STEP entity..."
./"${TARGET_BIN}" "${OUTPUT_STEP}"

echo ""
echo "=========================================================="
echo "[CRYPTO-VERIFICATION AUDIT ASSETS : W1-SEMI-FE]"
echo "=========================================================="

if [ -f "${OUTPUT_STEP}" ] && [ -s "${OUTPUT_STEP}" ]; then
    STEP_SIZE=$(wc -c < "${OUTPUT_STEP}")
    STEP_HASH=$(sha256sum "${OUTPUT_STEP}" | awk '{print $1}')
    echo "[ASSET] 3D CAD STEP MODEL"
    echo "  File : ${OUTPUT_STEP}"
    echo "  Size : ${STEP_SIZE} bytes"
    echo "  Hash : ${STEP_HASH}"
fi

if [ -f "${OUTPUT_PDF}" ] && [ -s "${OUTPUT_PDF}" ]; then
    PDF_SIZE=$(wc -c < "${OUTPUT_PDF}")
    PDF_HASH=$(sha256sum "${OUTPUT_PDF}" | awk '{print $1}')
    echo "[ASSET] 2D MANUFACTURING DRAWING"
    echo "  File : ${OUTPUT_PDF}"
    echo "  Size : ${PDF_SIZE} bytes"
    echo "  Hash : ${PDF_HASH}"
fi

echo "=========================================================="
echo "[SUCCESS] Pipeline for W1-SEMI-FE-STG-XY_RevA completed."
echo "=========================================================="