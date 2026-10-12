#!/usr/bin/env bash
# ==============================================================================
# PIPELINE RUNNER : WUA_END_EFFECTOR.sh
# COMPONENT ID   : WUA_END_EFFECTOR_RevA (Taxonomy 7.2)
# DESCRIPTION    : Production-Grade Monolithic Lithography Wafer End-Effector
# KERNEL ENGINE   : OpenCASCADE Technology (OCCT)
# ==============================================================================
set -euo pipefail

# 1. Resolve working directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

TARGET_SRC="WUA_END_EFFECTOR_RevA.cpp"
TARGET_BIN="run_wok_effector_builder"
OUTPUT_STEP="WUA_END_EFFECTOR_RevA.STEP"

echo "=========================================================="
echo "[+] Initializing Build Pipeline for Case 7.2"
echo "[+] Target Source    : ${TARGET_SRC}"
echo "[+] Working Directory: ${SCRIPT_DIR}"
echo "=========================================================="

if [ ! -f "${TARGET_SRC}" ]; then
    echo "[ERROR] Source file ${TARGET_SRC} not found in ${SCRIPT_DIR}!"
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
echo "[CRYPTO-VERIFICATION AUDIT ASSETS : CASE 7.2]"
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

# Locate latest 2D drawing PDF
PDF_FOUND=false
for pdf_candidate in "WUA_END_EFFECTOR_REVA_5.pdf" "WUA_END_EFFECTOR_REVA_4.pdf" "WUA_END_EFFECTOR_RevA.pdf"; do
    if [ -f "${pdf_candidate}" ] && [ -s "${pdf_candidate}" ]; then
        PDF_SIZE=$(wc -c < "${pdf_candidate}")
        PDF_HASH=$(sha256sum "${pdf_candidate}" | awk '{print $1}')
        echo "[ASSET] 2D MANUFACTURING DRAWING"
        echo "  File : ${pdf_candidate}"
        echo "  Size : ${PDF_SIZE} bytes"
        echo "  Hash : ${PDF_HASH}"
        PDF_FOUND=true
        break
    fi
done

if [ "${PDF_FOUND}" = false ]; then
    echo "[WARNING] No 2D PDF drawing detected in directory for hash audit."
fi

echo "=========================================================="
echo "[SUCCESS] WUA_END_EFFECTOR_RevA build pipeline completed."
echo "=========================================================="