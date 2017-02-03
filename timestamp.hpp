#pragma once

#ifndef DEBUG
#define DEBUG_DURATION( msg ) {}
#define DEBUG_MESSAGE( msg ) {}
#else

#if __cplusplus < 201103L
#error You need a compiller which supports atleast c++11, fully.
#endif

#include <chrono>
#include <cstdint>
#include <iostream>

#include "pathfinder_version.h"

// class meant to print duration of code execution
class TimeStamp {
public:
    inline TimeStamp() : m_start( std::chrono::system_clock::now() ) {}

    inline ~TimeStamp()
    {
        const auto duration = std::chrono::system_clock::now() - m_start;
        std::string unit;
        uint64_t value = std::chrono::duration_cast<std::chrono::microseconds>( duration ).count();
        if ( value < 1000 ) {
            unit = "\u00B5s";
        } else if ( value >= 1000 && value < 5000000 ) {
            value = std::chrono::duration_cast<std::chrono::milliseconds>( duration ).count();
            unit = "ms";
        } else {
            value = std::chrono::duration_cast<std::chrono::seconds>( duration ).count();
            unit = "s";
        }
        std::cout << value << unit << std::endl;
    }
protected:
    std::chrono::time_point<std::chrono::system_clock> m_start;
};

#define DEBUG_DURATION( msg ) std::cout << msg << " " << std::flush; TimeStamp timeStamp
#define DEBUG_MESSAGE( msg ) std::cout << msg

#endif
