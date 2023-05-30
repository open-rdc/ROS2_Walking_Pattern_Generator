#ifndef ROBOT_MANAGER__VISIBILITY_CONTROL_H_
#define ROBOT_MANAGER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROBOT_MANAGER_EXPORT __attribute__ ((dllexport))
    #define ROBOT_MANAGER_IMPORT __attribute__ ((dllimport))
  #else
    #define ROBOT_MANAGER_EXPORT __declspec(dllexport)
    #define ROBOT_MANAGER_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROBOT_MANAGER_BUILDING_LIBRARY
    #define ROBOT_MANAGER_PUBLIC ROBOT_MANAGER_EXPORT
  #else
    #define ROBOT_MANAGER_PUBLIC ROBOT_MANAGER_IMPORT
  #endif
  #define ROBOT_MANAGER_PUBLIC_TYPE ROBOT_MANAGER_PUBLIC
  #define ROBOT_MANAGER_LOCAL
#else
  #define ROBOT_MANAGER_EXPORT __attribute__ ((visibility("default")))
  #define ROBOT_MANAGER_IMPORT
  #if __GNUC__ >= 4
    #define ROBOT_MANAGER_PUBLIC __attribute__ ((visibility("default")))
    #define ROBOT_MANAGER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROBOT_MANAGER_PUBLIC
    #define ROBOT_MANAGER_LOCAL
  #endif
  #define ROBOT_MANAGER_PUBLIC_TYPE
#endif

#endif  // ROBOT_MANAGER__VISIBILITY_CONTROL_H_
