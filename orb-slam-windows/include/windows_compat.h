#ifndef WINDOWS_COMPAT_H
#define WINDOWS_COMPAT_H

#ifdef _WIN32

// Windows-specific includes
#include <windows.h>
#include <io.h>
#include <chrono>
#include <thread>
#include <process.h>

// Define usleep for Windows (microseconds sleep)
inline void usleep(unsigned int microseconds) {
    std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
}

// Map monotonic_clock to steady_clock (they're equivalent)
namespace std {
    namespace chrono {
        using monotonic_clock = steady_clock;
    }
}

// Provide dummy unistd.h functionality if needed
#define STDIN_FILENO 0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2

// Additional timing functions that might be needed
inline void sleep(unsigned int seconds) {
    std::this_thread::sleep_for(std::chrono::seconds(seconds));
}

#else
// On Unix/Linux, just include the normal header
#include <unistd.h>
#endif

#endif // WINDOWS_COMPAT_H