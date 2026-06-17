#pragma once

#include <cstdint>
#include <cstddef>

// ---------------------------------------------------------------------------
// http_pages.hpp
//
// All HTML / CSS / JS page content is compiled into flash (rodata) as plain
// C-string literals defined in http_pages.cpp.  Nothing is allocated on the
// heap at runtime.  Every page exposes:
//   extern const char   kPage<Name>[];   // NUL-terminated string in flash
//   extern const size_t kPage<Name>Len;  // strlen equivalent, no NUL
//
// The motor-dashboard page embeds the firmware version at a single injection
// point marked by the token HTTP_PAGES_VERSION_TOKEN, which http_server.cpp
// replaces with a statically-known pointer (FirmwareVersion::get_version_string()).
// Because the version string is short (<32 bytes) and known at build time the
// server sends the page in two fixed iovec-style chunks rather than doing any
// heap allocation.
// ---------------------------------------------------------------------------

// Token that marks where the firmware-version string should be injected.
// Must be unique within each page and short enough to fit in a single
// httpd_resp_send_chunk call budget.
#define HTTP_PAGES_VERSION_TOKEN "{{FW_VER}}"

// Root / dashboard page
extern const char   kPageRoot[];
extern const size_t kPageRootLen;

// Motor-control dashboard (served dynamically; version injected at runtime)
// Split into a prefix (before version token) and suffix (after).
extern const char   kPageMotorPrefix[];
extern const size_t kPageMotorPrefixLen;
extern const char   kPageMotorSuffix[];
extern const size_t kPageMotorSuffixLen;
