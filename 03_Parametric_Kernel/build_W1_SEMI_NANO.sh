#!/usr/bin/env bash
# ==============================================================================
# COMPILATION & EXECUTION PIPELINE : W1_SEMI_NANO_RevA
# STANDARD KERNEL ENGINE           : OpenCASCADE Technology (OCCT)
# ==============================================================================
set -euo pipefail

TARGET_SRC="W1_SEMI_NANO_RevA.cpp"
TARGET_BIN="run_wok_nano_rev_a"
OUTPUT_STEP="W1_SEMI_NANO_RevA.STEP"

echo "------------------------------------------------------------------"
echo "[+] Initializing Build Environment for ${TARGET_SRC}..."
echo "------------------------------------------------------------------"

# Detect OpenCASCADE Include Paths
OCCT_INC="/usr/include/opencascade"
if [ ! -d "${OCCT_INC}" ]; then
    echo "[!] Standard OCCT path not found, detecting via pkg-config..."
    OCCT_INC=$(pkg-config --cflags-only-I opencascade 2>/dev/null || echo "")
fi

# Define Compiler & Flags
CXX="g++"
CXXFLAGS="-O3 -std=c++17 -Wall -Wextra"
INCLUDES="-I/usr/include/opencascade"
LIBS="-lTKernel -lTKMath -lTKG2d -lTKG3d -lTKGeomBase -lTKBRep -lTKTopAlgo -lTKPrim -lTKBO -lTKBool -lTKFillet -lTKSTEP -lTKSTEPBase -lTKSTEPAttr -lTKXSBase"

# Clean Previous Binary Artifact
rm -f "${TARGET_BIN}"

# Compile Kernel
echo "[+] Compiling C++ parametric kernel..."
${CXX} ${CXXFLAGS} ${INCLUDES} "${TARGET_SRC}" -o "${TARGET_BIN}" ${LIBS}

if [ ! -f "${TARGET_BIN}" ]; then
    echo "[ERROR] Compilation failed! Binary not generated."
    exit 1
fi
echo "[SUCCESS] Binary compiled successfully: ${TARGET_BIN}"

# Execute Kernel to Produce STEP File
echo "[+] Executing geometry generator..."
./"${TARGET_BIN}"

# Verification & Assertion
if [ -f "${OUTPUT_STEP}" ] && [ -s "${OUTPUT_STEP}" ]; then
    FILE_SIZE=$(wc -c < "${OUTPUT_STEP}")
    echo "------------------------------------------------------------------"
    echo "[STATUS] SUCCESS: ${OUTPUT_STEP} generated successfully."
    echo "[INFO]   File Size: ${FILE_SIZE} bytes."
    echo "------------------------------------------------------------------"
else
    echo "[ERROR] Output file ${OUTPUT_STEP} is missing or empty!"
    exit 1
fi