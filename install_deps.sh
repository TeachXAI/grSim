#!/usr/bin/env bash
#
# install.sh - Setup, build, and install grSim (RoboCup SSL simulator)
#
# Designed for machines like Ubuntu/Debian (also supports Arch / macOS).
# After install, run headless (no GUI) with:
#   grSim --headless -platform offscreen
#
# Usage:
#   ./install.sh [options]
#
# Options:
#   --prefix DIR       Install prefix (default: /usr/local)
#   --build-dir DIR    Build directory (default: <repo>/build)
#   --jobs N           Parallel build jobs (default: nproc)
#   --build-type TYPE  CMake build type: Release|Debug (default: Release)
#   --build-ode        Force building ODE from source (-DBUILD_ODE=ON)
#   --no-clients       Skip building the example Qt client
#   --skip-deps        Do not install system packages
#   --skip-install     Build only; do not run 'cmake --install'
#   --clean            Remove the build directory before configuring
#   --no-ldconfig      Skip ldconfig after install
#   -h, --help         Show this help
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

# Defaults
PREFIX="/usr/local"
BUILD_DIR="${SCRIPT_DIR}/build"
JOBS="$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)"
BUILD_TYPE="Release"
BUILD_ODE="OFF"
BUILD_CLIENTS="ON"
SKIP_DEPS=0
SKIP_INSTALL=0
CLEAN=0
RUN_LDCONFIG=1
BINARY_PATH=""

# Known-good mirror for the protobuf tarball CMake ExternalProject downloads when
# system protobuf is missing or >= 3.21 (incompatible with this project).
PROTOBUF_VERSION="3.6.1"
PROTOBUF_TARBALL="protobuf-cpp-${PROTOBUF_VERSION}.tar.gz"
PROTOBUF_SHA256="b3732e471a9bb7950f090fd0457ebd2536a9ba0891b7f3785919c654fe2a2529"
PROTOBUF_URL_GITHUB="https://github.com/protocolbuffers/protobuf/releases/download/v${PROTOBUF_VERSION}/${PROTOBUF_TARBALL}"
PROTOBUF_URL_ERLANGEN="http://www.robotics-erlangen.de/downloads/libraries/${PROTOBUF_TARBALL}"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
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

# Run a command as root when needed (works as root without sudo)
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

# ---------------------------------------------------------------------------
# Parse args
# ---------------------------------------------------------------------------
while [[ $# -gt 0 ]]; do
  case "$1" in
    --prefix)       PREFIX="${2:?}"; shift 2 ;;
    --build-dir)    BUILD_DIR="${2:?}"; shift 2 ;;
    --jobs|-j)      JOBS="${2:?}"; shift 2 ;;
    --build-type)   BUILD_TYPE="${2:?}"; shift 2 ;;
    --build-ode)    BUILD_ODE="ON"; shift ;;
    --no-clients)   BUILD_CLIENTS="OFF"; shift ;;
    --skip-deps)    SKIP_DEPS=1; shift ;;
    --skip-install) SKIP_INSTALL=1; shift ;;
    --clean)        CLEAN=1; shift ;;
    --no-ldconfig)  RUN_LDCONFIG=0; shift ;;
    -h|--help)      usage ;;
    *)              die "Unknown option: $1 (try --help)" ;;
  esac
done

# ---------------------------------------------------------------------------
# Detect OS
# ---------------------------------------------------------------------------
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

# ---------------------------------------------------------------------------
# Install dependencies
# ---------------------------------------------------------------------------
install_deps_debian() {
  log "Installing build dependencies via apt..."
  export DEBIAN_FRONTEND=noninteractive

  run_as_root apt-get update -y

  # Build + runtime deps for grSim and headless/offscreen Qt.
  # Note: system protobuf on Ubuntu 22.04+/24.04 is often >= 3.21, which this
  # project cannot use; CMake will automatically download & build protobuf 3.6.1.
  local packages=(
    git
    ca-certificates
    curl
    build-essential
    cmake
    pkg-config
    # Qt5
    qtbase5-dev
    libqt5opengl5-dev
    # OpenGL / Mesa (needed to link and for offscreen GL)
    libgl1-mesa-dev
    libglu1-mesa-dev
    libegl1-mesa-dev
    # ODE physics (double precision on Ubuntu packages)
    libode-dev
    # Boost (VarTypes)
    libboost-dev
    # Protobuf headers/compiler (may be unused if CMake builds its own copy)
    libprotobuf-dev
    protobuf-compiler
    # Headless / offscreen runtime helpers
    libqt5opengl5
    libgl1
    libegl1
    # Optional: virtual framebuffer (Docker-style VNC / display-less helpers)
    xvfb
  )

  run_as_root apt-get install -y --no-install-recommends "${packages[@]}"
  ok "apt packages installed"
}

install_deps_arch() {
  log "Installing build dependencies via pacman..."
  run_as_root pacman -Sy --needed --noconfirm \
    base-devel boost hicolor-icon-theme \
    mesa ode protobuf qt5-base cmake git curl
  ok "pacman packages installed"
}

install_deps_macos() {
  if ! command -v brew >/dev/null 2>&1; then
    die "Homebrew is required on macOS. Install from https://brew.sh/"
  fi
  log "Installing build dependencies via Homebrew..."
  brew update || true
  brew tap robotology/formulae || true
  brew install cmake pkg-config robotology/formulae/ode qt@5 protobuf@21 curl || \
    brew install cmake pkg-config ode qt@5 protobuf curl
  ok "Homebrew packages installed"
  warn "On macOS you may need to export Qt paths before building, e.g.:"
  warn "  export PATH=\"\$(brew --prefix qt@5)/bin:\$PATH\""
  warn "  export CMAKE_PREFIX_PATH=\"\$(brew --prefix qt@5):\${CMAKE_PREFIX_PATH:-}\""
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
  command -v git >/dev/null 2>&1 || die "git not found (needed to fetch VarTypes / optional protobuf)"
  command -v pkg-config >/dev/null 2>&1 || die "pkg-config not found"
  command -v curl >/dev/null 2>&1 || die "curl not found (needed to fetch protobuf if required)"
}

# ---------------------------------------------------------------------------
# Ensure protobuf ExternalProject can download (mirror fallback)
# ---------------------------------------------------------------------------
# CMake builds protobuf 3.6.1 from source when system protobuf is missing or
# >= 3.21. The default URL (robotics-erlangen.de) is sometimes blocked; we
# rewrite BuildProtobuf.cmake to prefer the official GitHub release (same SHA256).
ensure_protobuf_download_url() {
  local cmake_file="${SCRIPT_DIR}/cmake/modules/BuildProtobuf.cmake"
  [[ -f "${cmake_file}" ]] || return 0

  if grep -q "robotics-erlangen.de/downloads/libraries/${PROTOBUF_TARBALL}" "${cmake_file}"; then
    log "Pointing protobuf ExternalProject at GitHub release (reliable mirror)..."
    # In-place rewrite (no .bak left in the tree). URL hash is unchanged.
    local tmp
    tmp="$(mktemp)"
    sed "s|http://www.robotics-erlangen.de/downloads/libraries/${PROTOBUF_TARBALL}|${PROTOBUF_URL_GITHUB}|g" \
      "${cmake_file}" > "${tmp}"
    cat "${tmp}" > "${cmake_file}"
    rm -f "${tmp}"
    ok "BuildProtobuf.cmake now uses: ${PROTOBUF_URL_GITHUB}"
  fi
}

# Optionally pre-seed the ExternalProject download cache so rebuilds are offline-friendly
preseed_protobuf_tarball() {
  local dest_dir="${BUILD_DIR}/protobuf_external-prefix/src"
  local dest="${dest_dir}/${PROTOBUF_TARBALL}"
  mkdir -p "${dest_dir}"

  if [[ -f "${dest}" ]]; then
    local have
    have="$(sha256sum "${dest}" 2>/dev/null | awk '{print $1}')"
    if [[ "${have}" == "${PROTOBUF_SHA256}" ]]; then
      ok "Protobuf tarball already present with correct checksum"
      return
    fi
    warn "Existing protobuf tarball has wrong checksum; re-downloading"
    rm -f "${dest}"
  fi

  log "Pre-downloading protobuf ${PROTOBUF_VERSION}..."
  if curl -fL --retry 3 --retry-delay 2 -o "${dest}" "${PROTOBUF_URL_GITHUB}"; then
    local have
    have="$(sha256sum "${dest}" | awk '{print $1}')"
    if [[ "${have}" != "${PROTOBUF_SHA256}" ]]; then
      rm -f "${dest}"
      die "Protobuf tarball checksum mismatch (got ${have}, expected ${PROTOBUF_SHA256})"
    fi
    ok "Downloaded ${PROTOBUF_TARBALL}"
  else
    warn "Could not pre-download protobuf from GitHub; CMake ExternalProject will try its configured URL"
    rm -f "${dest}"
  fi
}

# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
configure_and_build() {
  if [[ "${CLEAN}" -eq 1 && -d "${BUILD_DIR}" ]]; then
    log "Cleaning build directory: ${BUILD_DIR}"
    rm -rf "${BUILD_DIR}"
  fi

  mkdir -p "${BUILD_DIR}"

  ensure_protobuf_download_url
  preseed_protobuf_tarball

  log "Configuring CMake (type=${BUILD_TYPE}, prefix=${PREFIX}, BUILD_ODE=${BUILD_ODE}, clients=${BUILD_CLIENTS})..."
  # VarTypes is fetched automatically if missing.
  # Protobuf is auto-built from source when system version is missing or >= 3.21.
  # ODE can be forced with --build-ode if the system package misbehaves.
  cmake -S "${SCRIPT_DIR}" -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    -DBUILD_ODE="${BUILD_ODE}" \
    -DBUILD_CLIENTS="${BUILD_CLIENTS}"

  log "Building with ${JOBS} job(s)..."
  # First-time builds may download VarTypes (git) and/or protobuf (tarball)
  cmake --build "${BUILD_DIR}" --parallel "${JOBS}"
  ok "Build finished"

  local bin_path="${SCRIPT_DIR}/bin/grSim"
  if [[ ! -x "${bin_path}" ]]; then
    if [[ -x "${BUILD_DIR}/bin/grSim" ]]; then
      bin_path="${BUILD_DIR}/bin/grSim"
    elif [[ -x "${BUILD_DIR}/grSim" ]]; then
      bin_path="${BUILD_DIR}/grSim"
    else
      warn "Could not locate grSim binary after build (expected under bin/)"
      return
    fi
  fi
  ok "Binary: ${bin_path}"
  BINARY_PATH="${bin_path}"
}

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
do_install() {
  if [[ "${SKIP_INSTALL}" -eq 1 ]]; then
    warn "Skipping install (--skip-install). Binary is at: ${BINARY_PATH:-${SCRIPT_DIR}/bin/grSim}"
    return
  fi

  log "Installing to ${PREFIX}..."
  if [[ -w "${PREFIX}" ]] || [[ "$(id -u)" -eq 0 ]]; then
    cmake --install "${BUILD_DIR}"
  else
    run_as_root cmake --install "${BUILD_DIR}"
  fi

  # Ensure libraries installed under prefix/lib are found at runtime
  if [[ "${RUN_LDCONFIG}" -eq 1 ]] && command -v ldconfig >/dev/null 2>&1; then
    if [[ -d "${PREFIX}/lib" || -d "${PREFIX}/lib64" ]]; then
      log "Running ldconfig..."
      run_as_root ldconfig || warn "ldconfig failed (non-fatal)"
    fi
  fi

  local installed_bin="${PREFIX}/bin/grSim"
  if [[ -x "${installed_bin}" ]]; then
    ok "Installed: ${installed_bin}"
    BINARY_PATH="${installed_bin}"
  else
    warn "Install completed but ${installed_bin} not found"
  fi
}

# ---------------------------------------------------------------------------
# Headless smoke check (optional, non-fatal)
# ---------------------------------------------------------------------------
smoke_test_headless() {
  local bin="${BINARY_PATH:-${PREFIX}/bin/grSim}"
  if [[ ! -x "${bin}" ]]; then
    bin="${SCRIPT_DIR}/bin/grSim"
  fi
  if [[ ! -x "${bin}" ]]; then
    warn "Skipping headless smoke test (binary not found)"
    return
  fi

  log "Smoke-testing headless launch (3s)..."
  # --headless / -H : hide UI and disable GL widget rendering path in grSim
  # -platform offscreen : Qt platform plugin that needs no real X11 display
  set +e
  if command -v timeout >/dev/null 2>&1; then
    timeout 3s "${bin}" --headless -platform offscreen >/tmp/grsim-smoke.log 2>&1
  else
    "${bin}" --headless -platform offscreen >/tmp/grsim-smoke.log 2>&1 &
    local pid=$!
    sleep 3
    kill "${pid}" 2>/dev/null
    wait "${pid}" 2>/dev/null
  fi
  local rc=$?
  set -e
  # timeout returns 124 when the process is still running after the limit (success)
  if [[ ${rc} -eq 124 || ${rc} -eq 0 || ${rc} -eq 143 ]]; then
    ok "Headless launch looks healthy (exit ${rc})"
  else
    warn "Headless smoke test exited with code ${rc}. See /tmp/grsim-smoke.log"
    if [[ -f /tmp/grsim-smoke.log ]]; then
      warn "---- last 30 lines of smoke log ----"
      tail -n 30 /tmp/grsim-smoke.log >&2 || true
    fi
    warn "You may still run with a display, or install mesa/egl packages and retry."
  fi
}

print_summary() {
  local bin="${BINARY_PATH:-${PREFIX}/bin/grSim}"
  cat <<EOF

${C_BOLD}========================================${C_RESET}
${C_BOLD} grSim setup complete${C_RESET}
${C_BOLD}========================================${C_RESET}

  Binary:  ${bin}
  Prefix:  ${PREFIX}
  Build:   ${BUILD_DIR}

${C_BOLD}Run without GUI (headless):${C_RESET}

  ${bin} --headless -platform offscreen

  Short form:
  ${bin} -H -platform offscreen

${C_BOLD}What those flags do:${C_RESET}
  --headless / -H     grSim hides its window and disables the OpenGL view
  -platform offscreen Qt runs with no X11/Wayland display (server-friendly)

${C_BOLD}Typical network ports${C_RESET} (defaults; override in the config UI / grsim.xml):
  Command listen .............. 20011
  Vision / SSL-Vision style ... see VisionMulticastPort in settings
  Blue / Yellow status ........ BlueStatusSendPort / YellowStatusSendPort
  Sim control ................. SimControlListenPort

${C_BOLD}Troubleshooting:${C_RESET}
  • ODE precision crash → re-run:  ./install.sh --clean --build-ode
  • Stuck config after upgrade → remove ~/.config/Parsian/grsim.xml
  • Rebuild only (deps already installed): ./install.sh --skip-deps
  • Build without installing: ./install.sh --skip-install

EOF
}

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
main() {
  log "grSim install starting (repo: ${SCRIPT_DIR})"
  detect_os
  install_deps
  configure_and_build
  do_install
  smoke_test_headless
  print_summary
}

main "$@"
