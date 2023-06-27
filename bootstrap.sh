#!/bin/bash -e

show_usage() {
    echo -e "Bootstrap build environment"
    echo -e ""
    echo -e "USAGE:"
    echo -e "\t`basename $0` [OPTIONS]"
    echo -e ""
    echo -e "OPTIONS:"
    echo -e "\t-v/--verbose:\tEnable verbose logging (set -x)"
}

while [[ $# -gt 0 ]]; do
    case $1 in
        -v|--verbose)
            shift
            set -x
            ;;
        -h|--help|-help)
            show_usage
            exit 0
            ;;
        *)
            echo "Unknown option \`$1\`"
            show_usage
            exit 1
            ;;
    esac
done

TOOLS_DIR=.tools
TOOLCHAIN_DIR=${PWD}/${TOOLS_DIR}/llvm
DOWNLOAD_DIR=${PWD}/${TOOLS_DIR}/downloads

LLVM_VERSION="16.0.0"

OS=$(uname | tr '[:upper:]' '[:lower:]')
ARCH=$(uname -m| tr '[:upper:]' '[:lower:]')

download_tool() {
    mkdir -p ${DOWNLOAD_DIR}
    wget -P ${DOWNLOAD_DIR} $1
}

download_toolchain() {
    INSTALL_DIR=${TOOLCHAIN_DIR}

    DIR_NAME=LLVMEmbeddedToolchainForArm-${LLVM_VERSION}
    TAR_NAME=${DIR_NAME}-${OS}.tar.gz
    URL=https://github.com/ARM-software/LLVM-embedded-toolchain-for-Arm/releases/download/release-${LLVM_VERSION}/${TAR_NAME}
    download_tool ${URL}

    mkdir -p ${TOOLCHAIN_DIR}
    pushd ${TOOLCHAIN_DIR}
    tar -xf ${DOWNLOAD_DIR}/${TAR_NAME}
    mv ${DIR_NAME}/* .
    rmdir ${DIR_NAME}
    popd

    # Cleanup
    rm -rf ${DOWNLOAD_DIR}
}

ensure_toolchain() {
    if [ -d "${TOOLCHAIN_DIR}" ]; then
        echo "toolchain already installed"
    else
        download_toolchain
    fi
}

ensure_toolchain
