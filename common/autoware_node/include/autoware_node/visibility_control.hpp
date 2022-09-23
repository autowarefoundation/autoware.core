#ifndef AUTOWARE_NODE__VISIBILITY_CONTROL_HPP_
#define AUTOWARE_NODE__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define AUTOWARE_NODE_EXPORT __attribute__((dllexport))
#define AUTOWARE_NODE_IMPORT __attribute__((dllimport))
#else
#define AUTOWARE_NODE_EXPORT __declspec(dllexport)
#define AUTOWARE_NODE_IMPORT __declspec(dllimport)
#endif
#ifdef AUTOWARE_NODE_BUILDING_LIBRARY
#define AUTOWARE_NODE_PUBLIC AUTOWARE_NODE_EXPORT
#else
#define AUTOWARE_NODE_PUBLIC AUTOWARE_NODE_IMPORT
#endif
#define AUTOWARE_NODE_PUBLIC_TYPE AUTOWARE_NODE_PUBLIC
#define AUTOWARE_NODE_LOCAL
#else
#define AUTOWARE_NODE_EXPORT __attribute__((visibility("default")))
#define AUTOWARE_NODE_IMPORT
#if __GNUC__ >= 4
#define AUTOWARE_NODE_PUBLIC __attribute__((visibility("default")))
#define AUTOWARE_NODE_LOCAL __attribute__((visibility("hidden")))
#else
#define AUTOWARE_NODE_PUBLIC
#define AUTOWARE_NODE_LOCAL
#endif
#define AUTOWARE_NODE_PUBLIC_TYPE
#endif

#endif  // AUTOWARE_NODE__VISIBILITY_CONTROL_HPP_
