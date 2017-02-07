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
#if __cplusplus < 201103L
#error You need a compiller which supports atleast c++11, fully.
#endif

#include <memory>
#include <limits>
#include <vector>

#include "pathfinder_version.h"

template<typename T>
class PathInfo {
public:
    typedef typename std::vector<std::shared_ptr<T>>::iterator iterator;
    typedef typename std::vector<std::shared_ptr<T>>::const_iterator const_iterator;
    enum Enum { Invalid, Found, NotFound };

    bool m_isShortest;
    std::shared_ptr<T> m_endPoint;
    std::vector<std::shared_ptr<T>> m_path;

    PathInfo( const std::shared_ptr<T>& p = nullptr ) : m_isShortest( false ), m_endPoint( p ) {};

    operator Enum() const
    {
        if ( !m_endPoint ) {
            return Invalid;
        } else if ( m_path.empty() ) {
            return NotFound;
        } else {
            return Found;
        }
    }

    // NOTE: 0 as result means PathInfo is invalid or path doesn't exists,
    // use cast operator to say which one
    uint64_t score() const
    {
        return ( m_endPoint && !m_path.empty() ) ? std::numeric_limits<uint64_t>::max() - m_path.size() : 0;
    }

    uint64_t size() const
    {
        return m_path.size();
    }

    bool operator < ( const PathInfo& v ) const
    {
        if ( !m_endPoint || !v.m_endPoint ) {
            return false;
        }
        return *m_endPoint < *v.m_endPoint;
    }

    void setPath( const std::vector<std::shared_ptr<T>>& p )
    {
        m_path = p;
    }

    // tells if path is best possible path, equal to manhattan distance from start to end
    bool isShortest() const
    {
        return m_isShortest;
    }

    void setShortest( bool b ) {
        m_isShortest = b;
    }

    iterator begin()
    {
        return m_path.begin();
    }

    iterator end()
    {
        return m_path.end();
    }

    const_iterator begin() const
    {
        return m_path.cbegin();
    }

    const_iterator end() const
    {
        return m_path.cend();
    }
};
