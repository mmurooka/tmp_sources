#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define FrameVisualizationController_DLLIMPORT __declspec(dllimport)
#  define FrameVisualizationController_DLLEXPORT __declspec(dllexport)
#  define FrameVisualizationController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define FrameVisualizationController_DLLIMPORT __attribute__((visibility("default")))
#    define FrameVisualizationController_DLLEXPORT __attribute__((visibility("default")))
#    define FrameVisualizationController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define FrameVisualizationController_DLLIMPORT
#    define FrameVisualizationController_DLLEXPORT
#    define FrameVisualizationController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef FrameVisualizationController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define FrameVisualizationController_DLLAPI
#  define FrameVisualizationController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef FrameVisualizationController_EXPORTS
#    define FrameVisualizationController_DLLAPI FrameVisualizationController_DLLEXPORT
#  else
#    define FrameVisualizationController_DLLAPI FrameVisualizationController_DLLIMPORT
#  endif // FrameVisualizationController_EXPORTS
#  define FrameVisualizationController_LOCAL FrameVisualizationController_DLLLOCAL
#endif // FrameVisualizationController_STATIC