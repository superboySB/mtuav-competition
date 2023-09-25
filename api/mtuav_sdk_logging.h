#ifndef MTUAV_SDK_LOGGING_H
#define MTUAV_SDK_LOGGING_H

#include "mtuav_sdk_export.h"

#if OS_LINUX
#define MT_LOG(format, ...) ::mtuav::MLog(__FILE__, __LINE__, format, __VA_ARGS__)
#endif

namespace mtuav {
SDK_EXPORT void MLog(const char* filename, int line, const char* format, ...);
}

#endif //MTUAV_SDK_LOGGING_H
