/************************************************************************************
*************************************************************************************
**
** MIT License
**
** Copyright (c) 2017 Maciej Latocha <latocha.maciek@gmail.com>
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions:
**
** The above copyright notice and this permission notice shall be included in all
** copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
** OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
** SOFTWARE.
**
*************************************************************************************
************************************************************************************/

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
