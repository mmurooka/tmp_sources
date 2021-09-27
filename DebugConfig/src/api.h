#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define DebugConfig_DLLIMPORT __declspec(dllimport)
#  define DebugConfig_DLLEXPORT __declspec(dllexport)
#  define DebugConfig_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define DebugConfig_DLLIMPORT __attribute__((visibility("default")))
#    define DebugConfig_DLLEXPORT __attribute__((visibility("default")))
#    define DebugConfig_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define DebugConfig_DLLIMPORT
#    define DebugConfig_DLLEXPORT
#    define DebugConfig_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef DebugConfig_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define DebugConfig_DLLAPI
#  define DebugConfig_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef DebugConfig_EXPORTS
#    define DebugConfig_DLLAPI DebugConfig_DLLEXPORT
#  else
#    define DebugConfig_DLLAPI DebugConfig_DLLIMPORT
#  endif // DebugConfig_EXPORTS
#  define DebugConfig_LOCAL DebugConfig_DLLLOCAL
#endif // DebugConfig_STATIC