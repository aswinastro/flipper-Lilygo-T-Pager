cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "components")

# ---------------------------------------------------------------------------
# Optional patch for libnet80211.a to allow raw TX of deauth/disassoc management frames.
# Skipped on IDF 5.5+ to prevent relocation decoding faults with newer toolchains.
# ---------------------------------------------------------------------------

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(furi_esp32)
