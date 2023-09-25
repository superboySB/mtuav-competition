#ifndef MTUAV_SDK_EXPORT_H
#define MTUAV_SDK_EXPORT_H

#if defined(__linux__)
#define OS_LINUX 1
#elif defined(_WIN32)
#define OS_WIN 1
#endif

#if defined(OS_WIN)
#if defined(MTUAV_SDK_IMPLEMENTATION)
#define SDK_EXPORT __declspec(dllexport)
#else
#define SDK_EXPORT __declspec(dllimport)
#endif  // defined(MTUAV_SDK_IMPLEMENTATION)
#else  // defined(OS_WIN)
#if defined(MTUAV_SDK_IMPLEMENTATION)
#define SDK_EXPORT __attribute__((visibility("default")))
#else
#define SDK_EXPORT
#endif  // defined(MTUAV_SDK_IMPLEMENTATION)
#endif

#endif //MTUAV_SDK_EXPORT_H
