#ifndef NET_FT_DRIVER__VISIBILITY_CONTROL_H_
#define NET_FT_DRIVER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define NET_FT_DRIVER_EXPORT __attribute__((dllexport))
#define NET_FT_DRIVER_IMPORT __attribute__((dllimport))
#else
#define NET_FT_DRIVER_EXPORT __declspec(dllexport)
#define NET_FT_DRIVER_IMPORT __declspec(dllimport)
#endif
#ifdef NET_FT_DRIVER_BUILDING_DLL
#define NET_FT_DRIVER_PUBLIC NET_FT_DRIVER_EXPORT
#else
#define NET_FT_DRIVER_PUBLIC NET_FT_DRIVER_IMPORT
#endif
#define NET_FT_DRIVER_PUBLIC_TYPE NET_FT_DRIVER_PUBLIC
#define NET_FT_DRIVER_LOCAL
#else
#define NET_FT_DRIVER_EXPORT __attribute__((visibility("default")))
#define NET_FT_DRIVER_IMPORT
#if __GNUC__ >= 4
#define NET_FT_DRIVER_PUBLIC __attribute__((visibility("default")))
#define NET_FT_DRIVER_LOCAL __attribute__((visibility("hidden")))
#else
#define NET_FT_DRIVER_PUBLIC
#define NET_FT_DRIVER_LOCAL
#endif
#define NET_FT_DRIVER_PUBLIC_TYPE
#endif

#endif  // NET_FT_DRIVER__VISIBILITY_CONTROL_H_
