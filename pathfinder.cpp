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




#include "pathfinder.hpp"



class Point {
protected:
    int32_t m_x, m_y, m_index;

public:
    Point( int32_t x, int32_t y, int32_t index = -1 );
    inline int32_t x() const { return m_x; }
    inline int32_t y() const { return m_y; }
    inline int32_t index() const { return m_index; }
    bool operator == ( const Point& p ) const;
    bool operator < ( const Point& p ) const;

    static Point fromIndex( int32_t i, int32_t w );
    static int64_t manhattan( const Point& a, const Point& b );
};

Point::Point( int32_t x, int32_t y, int32_t index ) :
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

Point Point::fromIndex( int32_t index, int32_t width )
{
    assert( index > -1 );
    assert( width > 0 );
    return Point( index % width, index / width, index );
}

int64_t Point::manhattan( const Point& a, const Point& b )
{
    return abs( a.m_x - b.m_x ) + abs( a.m_y - b.m_y );
}



static bool isObstacle( const unsigned char* waypoints, int64_t waypointsSize, const Point& p )
{
    assert( waypoints );
    assert( p.index() > -1 );
    if ( p.index() >= waypointsSize ) {
        return true;
    }
    return !waypoints[ p.index() ];
}

bool sortCmp( const std::shared_ptr<Point>& a, const std::shared_ptr<Point>& b )
{
    assert( a );
    assert( b );
    return *a < *b;
}

static std::vector< std::shared_ptr<Point> > rawDataToPoints( const unsigned char* waypoints, int64_t waypointsSize, int64_t width )
{
    DEBUG_DURATION( "Converting raw data into something usable..." );
    assert( waypoints );
    assert( waypointsSize > 0 );
    assert( width > 0 );
    std::vector< std::shared_ptr<Point> > points;
    int64_t i = 0;
    while ( i < waypointsSize ) {
        const Point p( Point::fromIndex( i++, width ) );
        if ( !isObstacle( waypoints, waypointsSize, p ) ) {
            points.push_back( std::make_shared<Point>( p ) );
        }
    }
    std::sort( points.begin(), points.end(), &sortCmp );
    return points;
}



// do you have a better name?
typedef std::vector< std::shared_ptr<Point> > PointHoarder;
static PointHoarder adjecentFor( const std::shared_ptr<Point>& pointPtr, const PointHoarder& almightyPointHoarder )
{
    assert( pointPtr );
    if ( !std::binary_search( almightyPointHoarder.begin(), almightyPointHoarder.end(), pointPtr, &sortCmp ) ) {
        return PointHoarder();
    }

    PointHoarder adjecent;
// explicit manifestation of laziness for writting same code with decorative changes
#define ADD_ADJECENT_NODE( x_, y_ ) { \
    std::shared_ptr<Point> ptr = std::make_shared<Point>( pointPtr->x() + x_, pointPtr->y() + y_ ); \
    if ( std::binary_search( almightyPointHoarder.begin(), almightyPointHoarder.end(), ptr, &sortCmp ) ) { \
        adjecent.push_back( *std::lower_bound( almightyPointHoarder.cbegin(), almightyPointHoarder.cend(), ptr, &sortCmp ) ); \
    } \
}
    ADD_ADJECENT_NODE( 0, -1 );
    ADD_ADJECENT_NODE( 0, 1 );
    ADD_ADJECENT_NODE( -1, 0 );
    ADD_ADJECENT_NODE( 1, 0 );
#undef ADD_ADJECENT_NODE
    return adjecent;
}





static void printPathInfo( const PathInfo<Point>& path )
{
    std::cout << std::endl;
    switch ( path ) {
        case PathInfo<Point>::Invalid:
            std::cout << "Path is not valid for some reason" << std::endl;
            break;
        case PathInfo<Point>::NotFound:
            std::cout << "Path could not be found" << std::endl;
            break;
        case PathInfo<Point>::Found:
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
#if 0
    {
        const std::vector<unsigned char> rawData {
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

        const std::vector< std::shared_ptr<Point> > data = rawDataToPoints( &rawData[0], rawData.size(), 16 );
        PathFinder<Point> pathFinder( data, std::bind( &adjecentFor, std::placeholders::_1, data ), &Point::manhattan );
        PathInfo<Point> pi = pathFinder.findPath( Point( 0, 0 ), Point( 15, 9 ) );
        printPathInfo( pi );
    }
#endif

// extreme stress test?
#if 1
    {
        // may get a crash due to too deep recursion if exceeded stack limit
        // try: ulimit -s unlimited
        const int64_t size = 3000;
        std::vector<unsigned char> rawData( size * size, 1 );

        const std::vector< std::shared_ptr<Point> > data = rawDataToPoints( &rawData[0], rawData.size(), size );
        PathFinder<Point> pathFinder( data, std::bind( &adjecentFor, std::placeholders::_1, data ), &Point::manhattan );
        PathInfo<Point> pi = pathFinder.findPath( Point( 0, 0 ), Point( 15, 9 ) );
        printPathInfo( pi );
    }
#endif

// example 1 from requirments
#if 0
    {
        const std::vector<unsigned char> rawData {
            1, 1, 1, 1,
            0, 1, 0, 1,
            0, 1, 1, 1
        };
        const std::vector< std::shared_ptr<Point> > data = rawDataToPoints( &rawData[0], rawData.size(), 4 );
        PathFinder<Point> pathFinder( data, std::bind( &adjecentFor, std::placeholders::_1, data ), &Point::manhattan );
        PathInfo<Point> pi = pathFinder.findPath( Point( 0, 0 ), Point( 3, 2 ) );
        printPathInfo( pi );
    }
#endif

// example 2 from requirments
#if 0
    {
        const std::vector<unsigned char> rawData {
            0, 0, 1,
            0, 1, 1,
            1, 0, 1
        };
        const std::vector< std::shared_ptr<Point> > data = rawDataToPoints( &rawData[0], rawData.size(), 3 );
        PathFinder<Point> pathFinder( data, std::bind( &adjecentFor, std::placeholders::_1, data ), &Point::manhattan );
        PathInfo<Point> pi = pathFinder.findPath( Point( 2, 0 ), Point( 2, 2 ) );
        printPathInfo( pi );
    }
#endif
    return 0;
}
