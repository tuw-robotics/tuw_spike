#ifndef TUW_SPIKE_CONTROL__VISIBILITY_CONTROL_H_
#define TUW_SPIKE_CONTROL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define TUW_SPIKE_CONTROL_EXPORT __attribute__((dllexport))
#define TUW_SPIKE_CONTROL_IMPORT __attribute__((dllimport))
#else
#define TUW_SPIKE_CONTROL_EXPORT __declspec(dllexport)
#define TUW_SPIKE_CONTROL_IMPORT __declspec(dllimport)
#endif
#ifdef TUW_SPIKE_CONTROL_BUILDING_LIBRARY
#define TUW_SPIKE_CONTROL_PUBLIC TUW_SPIKE_CONTROL_EXPORT
#else
#define TUW_SPIKE_CONTROL_PUBLIC TUW_SPIKE_CONTROL_IMPORT
#endif
#define TUW_SPIKE_CONTROL_PUBLIC_TYPE TUW_SPIKE_CONTROL_PUBLIC
#define TUW_SPIKE_CONTROL_LOCAL
#else
#define TUW_SPIKE_CONTROL_EXPORT __attribute__((visibility("default")))
#define TUW_SPIKE_CONTROL_IMPORT
#if __GNUC__ >= 4
#define TUW_SPIKE_CONTROL_PUBLIC __attribute__((visibility("default")))
#define TUW_SPIKE_CONTROL_LOCAL __attribute__((visibility("hidden")))
#else
#define TUW_SPIKE_CONTROL_PUBLIC
#define TUW_SPIKE_CONTROL_LOCAL
#endif
#define TUW_SPIKE_CONTROL_PUBLIC_TYPE
#endif

#endif // TUW_SPIKE_CONTROL__VISIBILITY_CONTROL_H_
