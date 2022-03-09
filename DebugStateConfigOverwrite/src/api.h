#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define DebugStateConfigOverwrite_DLLIMPORT __declspec(dllimport)
#  define DebugStateConfigOverwrite_DLLEXPORT __declspec(dllexport)
#  define DebugStateConfigOverwrite_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define DebugStateConfigOverwrite_DLLIMPORT __attribute__((visibility("default")))
#    define DebugStateConfigOverwrite_DLLEXPORT __attribute__((visibility("default")))
#    define DebugStateConfigOverwrite_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define DebugStateConfigOverwrite_DLLIMPORT
#    define DebugStateConfigOverwrite_DLLEXPORT
#    define DebugStateConfigOverwrite_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef DebugStateConfigOverwrite_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define DebugStateConfigOverwrite_DLLAPI
#  define DebugStateConfigOverwrite_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef DebugStateConfigOverwrite_EXPORTS
#    define DebugStateConfigOverwrite_DLLAPI DebugStateConfigOverwrite_DLLEXPORT
#  else
#    define DebugStateConfigOverwrite_DLLAPI DebugStateConfigOverwrite_DLLIMPORT
#  endif // DebugStateConfigOverwrite_EXPORTS
#  define DebugStateConfigOverwrite_LOCAL DebugStateConfigOverwrite_DLLLOCAL
#endif // DebugStateConfigOverwrite_STATIC