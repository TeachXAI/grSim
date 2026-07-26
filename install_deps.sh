#!/usr/bin/env bash
#
# install_deps.sh - Install dependencies and build the headless grSim library
#
# The default product is a network-free, GUI-free C++ library (libgrsim)
# for in-process ML/RL training and control. No Qt, protobuf, VarTypes,
# or OpenGL are required.
#
# Usage:
#   ./install_deps.sh [options]
#
# Options:
#   --prefix DIR       Install prefix (default: /usr/local)
#   --build-dir DIR    Build directory (default: <repo>/build)
#   --jobs N           Parallel build jobs (default: nproc)
#   --build-type TYPE  CMake build type: Release|Debug (default: Release)
#   --skip-deps        Do not install system packages
#   --skip-install     Build only; do not run 'cmake --install'
#   --skip-tests       Configure with -DGRSIM_BUILD_TESTS=OFF
#   --clean            Remove the build directory before configuring
#   -h, --help         Show this help
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

PREFIX="/usr/local"
BUILD_DIR="${SCRIPT_DIR}/build"
JOBS="$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)"
BUILD_TYPE="Release"
SKIP_DEPS=0
SKIP_INSTALL=0
SKIP_TESTS=0
CLEAN=0
BINARY_PATH=""

if [[ -t 1 ]]; then
  C_RESET=$'\033[0m'
  C_BOLD=$'\033[1m'
  C_GREEN=$'\033[32m'
  C_YELLOW=$'\033[33m'
  C_RED=$'\033[31m'
  C_BLUE=$'\033[34m'
else
  C_RESET= C_BOLD= C_GREEN= C_YELLOW= C_RED= C_BLUE=
fi

log()  { echo "${C_BLUE}[install]${C_RESET} $*"; }
ok()   { echo "${C_GREEN}[ok]${C_RESET} $*"; }
warn() { echo "${C_YELLOW}[warn]${C_RESET} $*" >&2; }
die()  { echo "${C_RED}[error]${C_RESET} $*" >&2; exit 1; }

run_as_root() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  elif command -v sudo >/dev/null 2>&1; then
    sudo "$@"
  else
    die "Need root privileges to run: $* (install sudo or re-run as root)"
  fi
}

usage() {
  sed -n '2,28p' "$0" | sed 's/^# \?//'
  exit 0
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --prefix)       PREFIX="${2:?}"; shift 2 ;;
    --build-dir)    BUILD_DIR="${2:?}"; shift 2 ;;
    --jobs|-j)      JOBS="${2:?}"; shift 2 ;;
    --build-type)   BUILD_TYPE="${2:?}"; shift 2 ;;
    --skip-deps)    SKIP_DEPS=1; shift ;;
    --skip-install) SKIP_INSTALL=1; shift ;;
    --skip-tests)   SKIP_TESTS=1; shift ;;
    --clean)        CLEAN=1; shift ;;
    -h|--help)      usage ;;
    *)              die "Unknown option: $1 (try --help)" ;;
  esac
done

detect_os() {
  if [[ -f /etc/os-release ]]; then
    # shellcheck source=/dev/null
    . /etc/os-release
    OS_ID="${ID:-unknown}"
    OS_LIKE="${ID_LIKE:-}"
    OS_VERSION="${VERSION_ID:-}"
  elif [[ "$(uname -s)" == "Darwin" ]]; then
    OS_ID="macos"
    OS_LIKE=""
    OS_VERSION="$(sw_vers -productVersion 2>/dev/null || true)"
  else
    OS_ID="unknown"
    OS_LIKE=""
    OS_VERSION=""
  fi
  log "Detected OS: ${OS_ID} ${OS_VERSION} (like: ${OS_LIKE:-n/a})"
}

is_debian_like() {
  [[ "${OS_ID}" == "ubuntu" || "${OS_ID}" == "debian" || "${OS_LIKE}" == *debian* || "${OS_LIKE}" == *ubuntu* ]]
}

is_arch_like() {
  [[ "${OS_ID}" == "arch" || "${OS_ID}" == "manjaro" || "${OS_LIKE}" == *arch* ]]
}

install_deps_debian() {
  log "Installing headless build dependencies via apt..."
  export DEBIAN_FRONTEND=noninteractive
  run_as_root apt-get update -y
  local packages=(
    git
    ca-certificates
    build-essential
    cmake
    pkg-config
    libode-dev
    libyaml-cpp-dev
    python3
    python3-numpy
    python3-pil
    ffmpeg
  )
  run_as_root apt-get install -y --no-install-recommends "${packages[@]}"
  ok "apt packages installed"
}

install_deps_arch() {
  log "Installing headless build dependencies via pacman..."
  run_as_root pacman -Sy --needed --noconfirm \
    base-devel cmake git ode yaml-cpp python python-numpy python-pillow ffmpeg
  ok "pacman packages installed"
}

install_deps_macos() {
  if ! command -v brew >/dev/null 2>&1; then
    die "Homebrew is required on macOS. Install from https://brew.sh/"
  fi
  log "Installing headless build dependencies via Homebrew..."
  brew update || true
  brew tap robotology/formulae || true
  brew install cmake pkg-config robotology/formulae/ode yaml-cpp || \
    brew install cmake pkg-config ode yaml-cpp
  ok "Homebrew packages installed"
}

install_deps() {
  if [[ "${SKIP_DEPS}" -eq 1 ]]; then
    warn "Skipping dependency installation (--skip-deps)"
    return
  fi

  if is_debian_like; then
    install_deps_debian
  elif is_arch_like; then
    install_deps_arch
  elif [[ "${OS_ID}" == "macos" ]]; then
    install_deps_macos
  else
    die "Unsupported OS '${OS_ID}'. Install deps manually (see INSTALL.md) and re-run with --skip-deps"
  fi

  command -v cmake >/dev/null 2>&1 || die "cmake not found after installing dependencies"
  command -v g++ >/dev/null 2>&1 || command -v clang++ >/dev/null 2>&1 || die "No C++ compiler found"
  command -v pkg-config >/dev/null 2>&1 || die "pkg-config not found"
}

configure_and_build() {
  if [[ "${CLEAN}" -eq 1 && -d "${BUILD_DIR}" ]]; then
    log "Cleaning build directory: ${BUILD_DIR}"
    rm -rf "${BUILD_DIR}"
  fi

  mkdir -p "${BUILD_DIR}"

  local tests_flag=ON
  if [[ "${SKIP_TESTS}" -eq 1 ]]; then
    tests_flag=OFF
  fi

  log "Configuring CMake (type=${BUILD_TYPE}, prefix=${PREFIX}, tests=${tests_flag})..."
  cmake -S "${SCRIPT_DIR}" -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    -DGRSIM_BUILD_TESTS="${tests_flag}" \
    -DGRSIM_BUILD_APPS=ON

  log "Building with ${JOBS} job(s)..."
  cmake --build "${BUILD_DIR}" --parallel "${JOBS}"
  ok "Build finished"

  local bin_path="${BUILD_DIR}/libgrsim/grsim_run"
  if [[ ! -x "${bin_path}" ]]; then
    bin_path="${BUILD_DIR}/grsim_run"
  fi
  if [[ -x "${bin_path}" ]]; then
    ok "Binary: ${bin_path}"
    BINARY_PATH="${bin_path}"
  else
    warn "Could not locate grsim_run after build"
  fi

  if [[ "${tests_flag}" == "ON" ]]; then
    log "Running tests..."
    if [[ -d "${BUILD_DIR}/libgrsim" ]]; then
      ctest --test-dir "${BUILD_DIR}/libgrsim" --output-on-failure
    else
      ctest --test-dir "${BUILD_DIR}" --output-on-failure
    fi
    ok "Tests passed"
  fi
}

do_install() {
  if [[ "${SKIP_INSTALL}" -eq 1 ]]; then
    warn "Skipping install (--skip-install). Binary: ${BINARY_PATH:-${BUILD_DIR}/libgrsim/grsim_run}"
    return
  fi

  log "Installing to ${PREFIX}..."
  if [[ -w "${PREFIX}" ]] || [[ "$(id -u)" -eq 0 ]]; then
    cmake --install "${BUILD_DIR}"
  else
    run_as_root cmake --install "${BUILD_DIR}"
  fi
  ok "Installed to ${PREFIX}"
}

print_summary() {
  local bin="${BINARY_PATH:-${BUILD_DIR}/libgrsim/grsim_run}"
  cat <<SUM

${C_BOLD}========================================${C_RESET}
${C_BOLD} grSim headless library ready${C_RESET}
${C_BOLD}========================================${C_RESET}

  Binary:  ${bin}
  Prefix:  ${PREFIX}
  Build:   ${BUILD_DIR}

${C_BOLD}Run a demo (in-process, no network/GUI):${C_RESET}

  ${bin} --config config/default.yaml --duration 8 --mode sync --behavior circle

${C_BOLD}Dependencies (headless only):${C_RESET}
  CMake >= 3.14, C++17, libode, yaml-cpp

${C_BOLD}ML / RL:${C_RESET}
  #include "grsim/env.h"
  Link: -lgrsim -lode -lyaml-cpp -lpthread

SUM
}

main() {
  log "grSim headless install starting (repo: ${SCRIPT_DIR})"
  detect_os
  install_deps
  configure_and_build
  do_install
  print_summary
}

main "$@"
