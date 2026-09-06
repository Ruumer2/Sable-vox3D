#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VOX3D_DIR="${SCRIPT_DIR}/vox3D"
RESOURCES_DIR="${SCRIPT_DIR}/../common/src/main/resources/natives/sable_vox3d"

# Ensure Zig is installed
if command -v zig >/dev/null 2>&1; then
    ZIG="zig"
elif [ -x "/opt/zig/zig" ]; then
    ZIG="/opt/zig/zig"
else
    echo "Installing Zig 0.13.0 to /opt/zig..."
    mkdir -p /opt/zig
    curl -sL https://ziglang.org/download/0.13.0/zig-linux-x86_64-0.13.0.tar.xz | tar -xJ --strip-components=1 -C /opt/zig
    ZIG="/opt/zig/zig"
fi

echo "Using Zig: $($ZIG version)"

# JNI includes
JNI_INCLUDE=""
if [ -d "/usr/lib/jvm/java-21-openjdk-amd64/include" ]; then
    JNI_INCLUDE="-I/usr/lib/jvm/java-21-openjdk-amd64/include -I/usr/lib/jvm/java-21-openjdk-amd64/include/linux"
elif [ -n "${JAVA_HOME:-}" ] && [ -d "${JAVA_HOME}/include" ]; then
    JNI_INCLUDE="-I${JAVA_HOME}/include -I${JAVA_HOME}/include/linux"
fi

COMMON_INCLUDES=(
    -I"${SCRIPT_DIR}/src"
    -I"${VOX3D_DIR}/include"
    -I"${VOX3D_DIR}/src"
    ${JNI_INCLUDE}
)

COMMON_DEFINES=(
    -DBOX3D_DOUBLE_PRECISION
)

COMMON_FLAGS=(
    -O3
    -fPIC
    -Wall
    -Wno-unused-function
    -Wno-unused-variable
)

SOURCES=(
    "${SCRIPT_DIR}/src/sable_vox3d.c"
    "${VOX3D_DIR}/src/"*.c
)

build_target() {
    local target="$1"
    local os="$2"
    local arch="$3"
    local out_name="$4"
    local extra_flags="${5:-}"

    local out_dir="${RESOURCES_DIR}/${os}/${arch}"
    mkdir -p "${out_dir}"
    local out_file="${out_dir}/${out_name}"

    echo "=== Building ${os}/${arch} (${target}) -> ${out_name} ==="
    $ZIG cc -target "${target}" \
        -shared \
        "${COMMON_FLAGS[@]}" \
        "${COMMON_DEFINES[@]}" \
        "${COMMON_INCLUDES[@]}" \
        ${extra_flags} \
        "${SOURCES[@]}" \
        -o "${out_file}"

    echo "Built: ${out_file}"
    ls -lh "${out_file}"
    file "${out_file}" || true
    echo ""
}

# 1. Linux x86_64
build_target "x86_64-linux-gnu.2.28" "linux" "x86_64" "libsable_vox3d.so" "-lm -s"

# 2. Linux aarch64
build_target "aarch64-linux-gnu.2.28" "linux" "aarch64" "libsable_vox3d.so" "-lm -s"

# 3. macOS x86_64
build_target "x86_64-macos" "macos" "x86_64" "libsable_vox3d.dylib" ""

# 4. macOS aarch64 (Apple Silicon M1/M2/M3/M4)
build_target "aarch64-macos" "macos" "aarch64" "libsable_vox3d.dylib" ""

# 5. Windows aarch64 (ARM64)
build_target "aarch64-windows-gnu" "windows" "aarch64" "sable_vox3d.dll" "-s"
rm -f "${RESOURCES_DIR}/windows/aarch64/sable_vox3d.lib"

echo "All multi-platform binaries built successfully!"