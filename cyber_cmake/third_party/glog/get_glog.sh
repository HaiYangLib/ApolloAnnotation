# ==============================================================================
# -- Get GLOG and compile it with libc++ --------------------------------------
# ==============================================================================
#CXX_TAG=c8
#export CC=/usr/bin/clang-8
#export CXX=/usr/bin/clang++-8

CXX_TAG=gcc
export CC=/usr/bin/gcc
export CXX=/usr/bin/g++

# GFLAG
GFLAG_VERSION=v2.2.0
GFLAG_BASENAME=gflag-${GFLAG_VERSION}-${CXX_TAG}

GFLAG_LIBCXX_INCLUDE=${PWD}/${GFLAG_BASENAME}-libcxx-install/include
GFLAG_LIBCXX_LIBPATH=${PWD}/${GFLAG_BASENAME}-libcxx-install/lib

GFLAG_LIBSTDCXX_INCLUDE=${PWD}/${GFLAG_BASENAME}-libstdcxx-install/include
GFLAG_LIBSTDCXX_LIBPATH=${PWD}/${GFLAG_BASENAME}-libstdcxx-install/lib

# build GFLAG
build_gfalg() {

    rm -Rf \
        ${GFLAG_BASENAME}-libcxx-build ${GFLAG_BASENAME}-libstdcxx-build \
        ${GFLAG_BASENAME}-libcxx-install ${GFLAG_BASENAME}-libstdcxx-install

    if [ -d ${GFLAG_BASENAME}-source ]; then
        echo "${GFLAG_BASENAME}-source already existence"
    else
        echo "====================Retrieving GFLAG.=============================="
        git clone --depth=1 -b ${GFLAG_VERSION} https://github.com/gflags/gflags.git ${GFLAG_BASENAME}-source
    fi

    echo "====================Building GFLAG with libstdc++.==============="

    mkdir -p ${GFLAG_BASENAME}-libstdcxx-build

    pushd ${GFLAG_BASENAME}-libstdcxx-build >/dev/null

    cmake -G "Ninja" \
        -DCMAKE_CXX_FLAGS="-std=c++11" \
        -DBUILD_SHARED_LIBS=true \
        -DCMAKE_INSTALL_PREFIX="../${GFLAG_BASENAME}-libstdcxx-install" \
        ../${GFLAG_BASENAME}-source

    ninja

    ninja install

    popd >/dev/null
    rm -Rf ${GFLAG_BASENAME}-libcxx-build ${GFLAG_BASENAME}-libstdcxx-build

}

# GLOG
GLOG_VERSION=v0.3.5
GLOG_BASENAME=glog-${GLOG_VERSION}-${CXX_TAG}

GLOG_LIBCXX_INCLUDE=${PWD}/${GLOG_BASENAME}-libcxx-install/include
GLOG_LIBCXX_LIBPATH=${PWD}/${GLOG_BASENAME}-libcxx-install/lib

GLOG_LIBSTDCXX_INCLUDE=${PWD}/${GLOG_BASENAME}-libstdcxx-install/include
GLOG_LIBSTDCXX_LIBPATH=${PWD}/${GLOG_BASENAME}-libstdcxx-install/lib

# buidl glog
build_glog() {

    rm -Rf \
        ${GLOG_BASENAME}-libcxx-build ${GLOG_BASENAME}-libstdcxx-build \
        ${GLOG_BASENAME}-libcxx-install ${GLOG_BASENAME}-libstdcxx-install

    if [ -d ${GLOG_BASENAME}-source ]; then
        echo "${GLOG_BASENAME}-source already existence"
    else
        echo "====================Retrieving GLOG.=============================="
        git clone --depth=1 -b ${GLOG_VERSION} https://github.com/google/glog.git ${GLOG_BASENAME}-source
    fi


    echo "====================Building GLOG with libstdc++.==============="

    mkdir -p ${GLOG_BASENAME}-libstdcxx-build

    pushd ${GLOG_BASENAME}-libstdcxx-build >/dev/null

    cmake -G "Ninja" \
        -DCMAKE_CXX_FLAGS="-std=c++11" \
        -DBUILD_SHARED_LIBS=true \
        -DWITH_GFLAGS=OFF \
        -DCMAKE_INSTALL_PREFIX="../${GLOG_BASENAME}-libstdcxx-install" \
        ../${GLOG_BASENAME}-source

    ninja

    ninja install

    popd >/dev/null
    rm -Rf ${GLOG_BASENAME}-libcxx-build ${GLOG_BASENAME}-libstdcxx-build

}

if [[ -d "${GLOG_BASENAME}-libcxx-install" && -d "${GLOG_BASENAME}-libstdcxx-install" ]]; then
    echo "${GLOG_BASENAME} already installed."
else

    build_gfalg
    build_glog

fi
