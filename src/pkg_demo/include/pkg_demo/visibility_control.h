#ifndef PKG_DEMO__VISIBILITY_CONTROL_H_
#define PKG_DEMO__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PKG_DEMO_EXPORT __attribute__ ((dllexport))
    #define PKG_DEMO_IMPORT __attribute__ ((dllimport))
  #else
    #define PKG_DEMO_EXPORT __declspec(dllexport)
    #define PKG_DEMO_IMPORT __declspec(dllimport)
  #endif
  #ifdef PKG_DEMO_BUILDING_LIBRARY
    #define PKG_DEMO_PUBLIC PKG_DEMO_EXPORT
  #else
    #define PKG_DEMO_PUBLIC PKG_DEMO_IMPORT
  #endif
  #define PKG_DEMO_PUBLIC_TYPE PKG_DEMO_PUBLIC
  #define PKG_DEMO_LOCAL
#else
  #define PKG_DEMO_EXPORT __attribute__ ((visibility("default")))
  #define PKG_DEMO_IMPORT
  #if __GNUC__ >= 4
    #define PKG_DEMO_PUBLIC __attribute__ ((visibility("default")))
    #define PKG_DEMO_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PKG_DEMO_PUBLIC
    #define PKG_DEMO_LOCAL
  #endif
  #define PKG_DEMO_PUBLIC_TYPE
#endif

#endif  // PKG_DEMO__VISIBILITY_CONTROL_H_
