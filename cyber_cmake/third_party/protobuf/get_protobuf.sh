# ==============================================================================
# -- Get PROTOBUF and compile it with libc++ --------------------------------------
# ==============================================================================
#CXX_TAG=c8
#export CC=/usr/bin/clang-8
#export CXX=/usr/bin/clang++-8

CXX_TAG=gcc
export CC=/usr/bin/gcc
export CXX=/usr/bin/g++

# PROTOBUF
PROTOBUF_VERSION=v3.3.0
PROTOBUF_BASENAME=protobuf-${PROTOBUF_VERSION}-${CXX_TAG}

PROTOBUF_LIBCXX_INCLUDE=${PWD}/${PROTOBUF_BASENAME}-libcxx-install/include
PROTOBUF_LIBCXX_LIBPATH=${PWD}/${PROTOBUF_BASENAME}-libcxx-install/lib

PROTOBUF_LIBSTDCXX_INCLUDE=${PWD}/${PROTOBUF_BASENAME}-libstdcxx-install/include
PROTOBUF_LIBSTDCXX_LIBPATH=${PWD}/${PROTOBUF_BASENAME}-libstdcxx-install/lib

# buidl protobuf
build_protobuf() {

    rm -Rf \
        ${PROTOBUF_BASENAME}-libcxx-build ${PROTOBUF_BASENAME}-libstdcxx-build \
        ${PROTOBUF_BASENAME}-libcxx-install ${PROTOBUF_BASENAME}-libstdcxx-install

    if [ -d ${PROTOBUF_BASENAME}-source ]; then
        echo "${PROTOBUF_BASENAME}-source already existence"
    else
        echo "====================Retrieving PROTOBUF.=============================="
        wget https://github.com/google/protobuf/releases/download/v3.3.0/protobuf-cpp-3.3.0.tar.gz
        mkdir -p ${PROTOBUF_BASENAME}-source
        tar -xvf protobuf-cpp-3.3.0.tar.gz --strip-components 1 -C "${PROTOBUF_BASENAME}-source"
        rm protobuf-cpp-3.3.0.tar.gz
    fi

    echo "===================Building PROTOBUF with libstdc++.==============="

    mkdir -p ${PROTOBUF_BASENAME}-libstdcxx-build

    pushd ${PROTOBUF_BASENAME}-libstdcxx-build >/dev/null

    cmake -G "Ninja" \
        -DCMAKE_CXX_FLAGS="-std=c++11" \
        -DBUILD_SHARED_LIBS=true \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DCMAKE_BUILD_TYPE=Release \
        -Dprotobuf_BUILD_TESTS=OFF \
        -DCMAKE_INSTALL_PREFIX="../${PROTOBUF_BASENAME}-libstdcxx-install" \
        ../${PROTOBUF_BASENAME}-source/cmake

    ninja

    ninja install

    popd >/dev/null
    rm -Rf ${PROTOBUF_BASENAME}-libcxx-build ${PROTOBUF_BASENAME}-libstdcxx-build

}

if [[ -d "${PROTOBUF_BASENAME}-libcxx-install" && -d "${PROTOBUF_BASENAME}-libstdcxx-install" ]]; then
    echo "${PROTOBUF_BASENAME} already installed."
else

    build_protobuf

fi
