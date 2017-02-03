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
};
