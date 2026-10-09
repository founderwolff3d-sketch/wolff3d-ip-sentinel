#!/usr/bin/env bash
# ==============================================================================
# PIPELINE RUNNER : W3_ROBOT_6AXIS.sh
# COMPONENT       : W3_ROBOT_6AXIS_RevA (6-Axis F/T Sensor Isotropic Elastic Body)
# KERNEL ENGINE   : OpenCASCADE Technology (OCCT)
# ==============================================================================
set -euo pipefail

TARGET_SRC="W3_ROBOT_6AXIS_RevA.cpp"
TARGET_BIN="run_wok_6axis_rev_a"
OUTPUT_STEP="W3_ROBOT_6AXIS_RevA.STEP"
OUTPUT_PDF="W3_ROBOT_6AXIS_RevA.pdf"

echo "=========================================================="
echo "[+] Initializing Build Pipeline for ${TARGET_SRC}..."
echo "=========================================================="

# 1. Check Source Existence
if [ ! -f "${TARGET_SRC}" ]; then
    echo "[ERROR] Source file ${TARGET_SRC} not found!"
    exit 1
fi

# 2. Compiler and Linker Flags
CXX="g++"
CXXFLAGS="-O3 -std=c++17 -Wall -Wextra"
INCLUDES="-I/usr/include/opencascade"
LIBS="-lTKernel -lTKMath -lTKG2d -lTKG3d -lTKGeomBase -lTKBRep -lTKTopAlgo -lTKPrim -lTKBO -lTKBool -lTKFillet -lTKSTEP -lTKSTEPBase -lTKSTEPAttr -lTKXSBase"

# 3. Compile Native Kernel Binary
rm -f "${TARGET_BIN}"
echo "[+] Compiling C++ parametric kernel with OCCT..."
${CXX} ${CXXFLAGS} ${INCLUDES} "${TARGET_SRC}" -o "${TARGET_BIN}" ${LIBS}

if [ ! -f "${TARGET_BIN}" ]; then
    echo "[ERROR] Compilation failed! Binary not produced."
    exit 1
fi
echo "[SUCCESS] Native binary generated: ${TARGET_BIN}"

# 4. Execute Kernel to Generate STEP Model
echo "[+] Executing geometry kernel..."
./"${TARGET_BIN}"

# 5. Cryptographic Fingerprint Verification
if [ -f "${OUTPUT_STEP}" ] && [ -s "${OUTPUT_STEP}" ]; then
    STEP_SIZE=$(wc -c < "${OUTPUT_STEP}")
    STEP_HASH=$(sha256sum "${OUTPUT_STEP}" | awk '{print $1}')
    echo "----------------------------------------------------------"
    echo "[ASSET] 3D CAD STEP MODEL"
    echo "[+] File : ${OUTPUT_STEP}"
    echo "[+] Size : ${STEP_SIZE} bytes"
    echo "[+] Hash : ${STEP_HASH}"
    echo "----------------------------------------------------------"
else
    echo "[ERROR] Output file ${OUTPUT_STEP} is missing or empty!"
    exit 1
fi

if [ -f "${OUTPUT_PDF}" ] && [ -s "${OUTPUT_PDF}" ]; then
    PDF_SIZE=$(wc -c < "${OUTPUT_PDF}")
    PDF_HASH=$(sha256sum "${OUTPUT_PDF}" | awk '{print $1}')
    echo "[ASSET] 2D MANUFACTURING DRAWING"
    echo "[+] File : ${OUTPUT_PDF}"
    echo "[+] Size : ${PDF_SIZE} bytes"
    echo "[+] Hash : ${PDF_HASH}"
    echo "----------------------------------------------------------"
fi

echo "[SUCCESS] Pipeline completed successfully."
echo "=========================================================="