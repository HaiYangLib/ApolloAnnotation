# ==============================================================================
# -- Get POCO and compile it with libc++ --------------------------------------
# ==============================================================================
CXX_TAG=c8
export CC=/usr/bin/clang-8
export CXX=/usr/bin/clang++-8

POCO_VERSION=poco-1.11.1-release
POCO_BASENAME=poco-${POCO_VERSION}-${CXX_TAG}

POCO_LIBCXX_INCLUDE=${PWD}/${POCO_BASENAME}-libcxx-install/include
POCO_LIBCXX_LIBPATH=${PWD}/${POCO_BASENAME}-libcxx-install/lib

POCO_LIBSTDCXX_INCLUDE=${PWD}/${POCO_BASENAME}-libstdcxx-install/include
POCO_LIBSTDCXX_LIBPATH=${PWD}/${POCO_BASENAME}-libstdcxx-install/lib

if [[ -d "${POCO_BASENAME}-libcxx-install" && -d "${POCO_BASENAME}-libstdcxx-install" ]]; then
  echo "${POCO_BASENAME} already installed."
else
  rm -Rf \
    ${POCO_BASENAME}-libcxx-build ${POCO_BASENAME}-libstdcxx-build \
    ${POCO_BASENAME}-libcxx-install ${POCO_BASENAME}-libstdcxx-install

  if [ ! -d "${POCO_BASENAME}-source" ]; then

    echo "=====================Retrieving POCO.========================"
    git clone --depth=1 -b ${POCO_VERSION} https://github.com/pocoproject/poco.git ${POCO_BASENAME}-source

  fi

  echo "===================Building Poco with libstdc++.=================="

  mkdir -p ${POCO_BASENAME}-libstdcxx-build

  pushd ${POCO_BASENAME}-libstdcxx-build >/dev/null

  cmake -G "Ninja" \
    -DCMAKE_CXX_FLAGS="-std=c++14" \
    -DCMAKE_INSTALL_PREFIX="../${POCO_BASENAME}-libstdcxx-install" \
    ../${POCO_BASENAME}-source

  ninja

  ninja install

  popd >/dev/null

  rm -Rf ${POCO_BASENAME}-libcxx-build ${POCO_BASENAME}-libstdcxx-build
  #rm -Rf ${POCO_BASENAME}-source ${POCO_BASENAME}-libcxx-build ${POCO_BASENAME}-libstdcxx-build

fi