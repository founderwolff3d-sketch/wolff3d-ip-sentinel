# 1. 编译（仅链接实际存在的核心库）
g++ -O3 -std=c++17 build_WOLFF3D_LC_RevB.cpp -o build_revB \
    -I/usr/include/opencascade \
    -lTKernel -lTKMath -lTKG2d -lTKG3d -lTKGeomBase -lTKBRep \
    -lTKTopAlgo -lTKPrim -lTKBO -lTKFillet -lTKSTEP -lTKSTEPBase -lTKXSBase

# 2. 运行并生成 STEP 文件
./build_revB