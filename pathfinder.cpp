#if __cplusplus < 201103L
#error You need a compiller which supports atleast c++11, fully.
#endif

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <utility>
#include <vector>




#include "node.hpp"



class Point {
protected:
    int64_t m_x, m_y, m_index;

public:
    Point( int64_t x, int64_t y, int64_t index = -1 );
    inline int64_t x() const { return m_x; }
    inline int64_t y() const { return m_y; }
    inline int64_t index() const { return m_index; }
    bool operator == ( const Point& p ) const;
    bool operator < ( const Point& p ) const;

    static int64_t manhattan( const Point& a, const Point& b );
};

Point::Point( int64_t x, int64_t y, int64_t index ) :
    m_x( x ),
    m_y( y ),
    m_index( index )
{
}

bool Point::operator == ( const Point& p ) const
{
    return m_x == p.m_x && m_y == p.m_y;
}

bool Point::operator < ( const Point& p ) const
{
    if ( m_x < p.m_x ) {
        return true;
    } else if ( p.m_x < m_x ) {
        return false;
    } else {
        return m_y < p.m_y;
    }
}

int64_t Point::manhattan( const Point& a, const Point& b )
{
    return abs( a.m_x - b.m_x ) + abs( a.m_y - b.m_y );
}



// class meant to print duration of code execution
class TimeStamp {
public:
    TimeStamp();
    ~TimeStamp();
protected:
    std::chrono::time_point<std::chrono::system_clock> m_start;
};

TimeStamp::TimeStamp() : m_start( std::chrono::system_clock::now() ) {}
TimeStamp::~TimeStamp()
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



typedef Node<Point> NodePoint;

class PathFinder {
protected:
    std::mutex m_mutex;
    std::set<NodePoint::Ptr, std::function<bool( const NodePoint::Ptr&, const NodePoint::Ptr& )> > m_waypointSet;

    int64_t m_mapWidth;
    int64_t m_mapHeight;

public:
    PathFinder( const std::vector<bool>* waypoints, int64_t mapWidth, int64_t mapHeight );
    ~PathFinder();

    // returns path information, including start point
    NodePoint::PathInfo findPath( const Point& start, const Point& end );

};

PathFinder::~PathFinder()
{
    std::cout << "Releasig node resources... ";
    TimeStamp ts;
    for ( auto& n : m_waypointSet ) {
        n->reset();
    }
}

static Point iToPoint( int64_t i, int64_t w )
{
    assert( i > -1 );
    assert( w > 0 );
    return Point( i % w, i / w, i );
}

static bool isObstacle( const std::vector<bool>* waypoints, const Point& p )
{
    assert( waypoints );
    assert( p.index() > -1 );
    if ( static_cast<uint64_t>( p.index() ) >= waypoints->size() ) {
        return true;
    }
    return !waypoints->at( p.index() );
}

PathFinder::PathFinder( const std::vector<bool>* waypoints, int64_t mapWidth, int64_t mapHeight ) :
    m_waypointSet( &NodePoint::cmpLT ),
    m_mapWidth( mapWidth ),
    m_mapHeight( mapHeight )
{
    assert( waypoints );
    assert( waypoints->size() == static_cast<uint64_t>( mapWidth * mapHeight ) );

    // convert grid with obstacles into waypoint nodes
    {
        std::cout << "Converting data into nodes... ";
        TimeStamp ts;
        for ( size_t i = 0; i < waypoints->size(); i++ ) {
            const Point p = iToPoint( i, mapWidth );
            if ( !isObstacle( waypoints, p ) ) {
                NodePoint::Ptr ptr = std::make_shared<NodePoint>( p );
                ptr->setDistanceFunction( &Point::manhattan );
                m_waypointSet.insert( ptr );
            }
        }
    }

    // connect waypoint nodes
    {
        std::cout << "Generating connections... ";
        TimeStamp ts;
        for ( int64_t x = 0; x < mapWidth; x++ ) {
        for ( int64_t y = 0; y < mapHeight; y++ ) {
            auto node = m_waypointSet.find( std::make_shared<NodePoint>( Point( x, y ) ) );
            if ( node == m_waypointSet.end() ) {
                continue;
            }
    // explicit manifestation of laziness for writting same code with decorative changes
    #define ADD_ADJECENT_NODE( x, y ) {\
        auto nodeToAdd = m_waypointSet.find( std::make_shared<NodePoint>( Point( x, y ) ) ); \
        if ( nodeToAdd != m_waypointSet.end() ) { (*node)->addAdjecentNode( *nodeToAdd ); } \
    }
            ADD_ADJECENT_NODE( x, y - 1 );
            ADD_ADJECENT_NODE( x, y + 1 );
            ADD_ADJECENT_NODE( x - 1, y );
            ADD_ADJECENT_NODE( x + 1, y );
    #undef ADD_ADJECENT_NODE
        }
        }
    }
}

NodePoint::PathInfo PathFinder::findPath( const Point& start, const Point& end )
{
    std::lock_guard<std::mutex> lockGuard( m_mutex );

    auto startNode = m_waypointSet.find( std::make_shared<NodePoint>( start ) );
    if ( startNode == m_waypointSet.end() ) {
        std::cerr << "Start position is outside of map boundaries or is an obstacle... ";
        return NodePoint::PathInfo();
    }

    auto endNode = m_waypointSet.find( std::make_shared<NodePoint>( end ) );
    if ( endNode == m_waypointSet.end() ) {
        std::cerr << "End position is outside of map boundaries or is an obstacle" << std::endl;
        return NodePoint::PathInfo( std::make_shared<Point>( end ) );
    }

    // peeking at the cache
    {
        std::cout << "Browsing cache... ";
        TimeStamp ts;
        const NodePoint::PathInfo pi = (*startNode)->cacheLookup( std::make_shared<Point>( end ) );
        switch ( pi ) {
            case NodePoint::PathInfo::Invalid:
                std::cout << "not found ";
                break;
            case NodePoint::PathInfo::Found:
                std::cout << "found, skipping search ";
                return pi;
            case NodePoint::PathInfo::NotFound:
                std::cout << "path is known to not exists, skipping search ";
                return pi;
            default:
                assert( !"Unhandled enum" );
        }
    }

    // checking if the node can be reached from starting position
    {
        std::cout << "Checking if destination is reachable... ";
        TimeStamp ts;
        if ( (*startNode)->isReachable( *endNode ) ) {
            std::cout << "yes ";
        } else {
            std::cout << "no, skipping search ";
            return NodePoint::PathInfo( (*endNode)->data() );
        }
    }

    NodePoint::PathInfo path;
    {
        std::cout << "Searching... ";
        TimeStamp ts;
        path = (*startNode)->findData( *endNode );
    }

    // finalize searching, clearing minimum distance helper variable
    {
        std::cout << "Postsearch activity... ";
        TimeStamp ts;
        for ( auto& n : m_waypointSet ) {
            n->completeSearch();
        }
    }

    return path;
}







static void printPathInfo( const NodePoint::PathInfo& path )
{
    std::cout << std::endl;
    switch ( path ) {
        case NodePoint::PathInfo::Invalid:
            std::cout << "Path is not valid for some reason" << std::endl;
            break;
        case NodePoint::PathInfo::NotFound:
            std::cout << "Path could not be found" << std::endl;
            break;
        case NodePoint::PathInfo::Found:
            if ( path.size() == 1 ) {
                std::cout << "Starting point is also destination point." << std::endl;
                break;
            }
            std::cout << "Number of waypoints (including starting position):\t" << path.size() << std::endl;
            std::cout << "\nWaypoint list (x:y):\n\t";
            for ( const auto& p : path.m_path ) {
                assert( p );
                std::cout << p->x() << ":" << p->y() << ",  ";
            }
            std::cout << std::endl;

            std::cout << "\nWaypoint list (just index):\n\t";
            for ( const auto& p : path.m_path ) {
                assert( p );
                std::cout << p->index() << ", ";
            }
            break;
        default:
            assert( !"Unhandled enum" );
    }
    std::cout << std::endl;
}



int main( int argc, char** argv )
{
    const Point startPoint( 0, 0 ), endPoint( 15, 9 );
#if 0
    {
        const std::vector<bool> data {
            1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
            1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1,
            1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1,
            1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1,
            1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1,
            1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1
        };
        PathFinder pathFinder( &data, 16, 10 );
        printPathInfo( pathFinder.findPath( startPoint, endPoint ) );
    }
#endif

// extreme stress test?
#if 0
    {
        // may get a crash due to too deep recursion if exceeded stack limit
        // try: ulimit -s unlimited
        const int64_t size = 3000;
        std::vector<bool> data( size * size, 1 );

        PathFinder pathFinder( &data, size, size );
        printPathInfo( pathFinder.findPath( startPoint, endPoint ) );
    }
#endif

// example 1 from requirments
#if 1
    {
        const std::vector<bool> data {
            1, 1, 1, 1,
            0, 1, 0, 1,
            0, 1, 1, 1
        };
        PathFinder pathFinder( &data, 4, 3 );
        printPathInfo( pathFinder.findPath( Point( 0, 0 ), Point( 1, 2 ) ) );
    }
#endif

// example 2 from requirments
#if 0
    {
        const std::vector<bool> data {
            0, 0, 1,
            0, 1, 1,
            1, 0, 1
        };
        PathFinder pathFinder( &data, 3, 3 );
        printPathInfo( pathFinder.findPath( Point( 2, 0 ), Point( 0, 2 ) ) );
    }
#endif

    return 0;
}
