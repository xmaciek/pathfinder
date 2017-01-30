#if __cplusplus < 201103L
#error You need a compiller which supports atleast c++11.
#endif

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <utility>
#include <vector>


// class meant to automatically release lock when leaving scope
class TravelLock {
public:
    inline TravelLock( bool& b ) : m_b( b ) { m_b = true; }
    inline ~TravelLock() { m_b = false; }
protected:
    bool& m_b;
};

template<typename T>
class Node {
public:
    typedef std::shared_ptr<Node> Ptr;
    typedef std::shared_ptr<T> DataPtr;
    typedef std::vector<DataPtr> Path;
    typedef std::function<int64_t( const T&, const T& )> DistanceFunction;

protected:
    bool m_isLocked;
    DataPtr m_dataPtr;
    std::vector<Ptr> m_adjecentNodeVector;

    // m_shortestPath is set only when distance is equal to manhattan distance,
    // this works as cache so we do not have to traverse again this node.
    Path m_shortestPath;
    int64_t m_minDistance;

public:
    Node( const T& p ) :
        m_isLocked( false ),
        // copy of data is explicitly stored as shared pointer to keep result values valid after deletion of Node
        m_dataPtr( std::make_shared<T>( p ) ),
        m_minDistance( -1 )
    {}


    void addAdjecentNode( Ptr n )
    {
        assert( n );
        m_adjecentNodeVector.push_back( n );
    };

    // NOTE: meant to be called in ~PathFinder(), otherwise we get memory leak.
    // this is due to Nodes beign held as shared pointers by PathFiner, and they hold
    // same shared pointers to each other as adjecent nodes.
    void reset()
    {
        m_minDistance = -1;
        m_adjecentNodeVector.clear();
    }

    Path findData( Ptr p )
    {
        assert( p );
        assert( m_dataPtr );
        assert( m_minDistance > -1 );

        if ( !m_shortestPath.empty() ) {
            return m_shortestPath;
        }

        if ( *m_dataPtr == *(p->m_dataPtr) ) {
            m_shortestPath = { m_dataPtr };
            return m_shortestPath;
        }

        if ( m_isLocked ) {
            return Path();
        }

        TravelLock lock( m_isLocked );

        Path adjecentPath;
        bool foundShortest = false;
        for ( auto& direction : m_adjecentNodeVector ) {
            Path tmp = direction->findData( p );
            if ( tmp.empty() ) {
                continue;
            }
            if ( ( adjecentPath.empty() ) || tmp.size() < adjecentPath.size() ) {
                std::swap( tmp, adjecentPath );
            }
            if ( adjecentPath.size() == m_minDistance ) {
                foundShortest = true;
                break;
            }
        }

        if ( adjecentPath.empty() ) {
            return Path();
        }


        Path path = { m_dataPtr };
        path.insert( path.end(), adjecentPath.begin(), adjecentPath.end() );
        if ( foundShortest ) {
            m_shortestPath = path;
        }
        return path;
    }

    void prepareSearch( const T& t, DistanceFunction distanceFunction )
    {
        assert( m_dataPtr );
        assert( distanceFunction );
        m_minDistance = distanceFunction( *m_dataPtr, t );
        std::sort( m_adjecentNodeVector.begin(), m_adjecentNodeVector.end(), [t, distanceFunction]( auto& a, auto& b )
        {
            assert( a );
            assert( b );
            assert( a->m_dataPtr );
            assert( b->m_dataPtr );
            assert( distanceFunction );
            return distanceFunction( *(a->m_dataPtr), t ) < distanceFunction( *(b->m_dataPtr), t );
        } );
    }

    void completeSearch()
    {
        m_minDistance = -1;
        m_shortestPath.clear();
    }

    // comarsion function for std::set
    static bool cmpLT( const Node<T>::Ptr& a, const Node<T>::Ptr& b )
    {
        assert( a );
        assert( a->m_dataPtr );
        assert( b );
        assert( b->m_dataPtr );
        return *(a->m_dataPtr) < *(b->m_dataPtr);
    }

    static bool cmpLT2( Node<T>::Ptr a, Node<T>::Ptr, const T& b )
    {
        assert( a );
        assert( a->m_dataPtr );
        return *(a->m_dataPtr) < b;
    }
};







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
    typedef std::vector< std::shared_ptr<Point> > Path;
    int64_t findPath( const Point& start, const Point& end, Path* pathOut );

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
    assert( w );
    return Point( i % w, i / w, i );
}

static bool isPointOnMap( const Point& p, int64_t w, int64_t h )
{
    return w > p.x() && p.x() >= 0 &&
           h > p.y() && p.y() >= 0;
}

static bool isObstacle( const std::vector<bool>* waypoints, const Point& p, int64_t w, int64_t h )
{
    assert( waypoints );
    if ( !isPointOnMap( p, w, h ) ) {
        return true;
    }
    return !waypoints->at( p.x() + w * p.y() );
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
            if ( !isObstacle( waypoints, p, mapWidth, mapHeight ) ) {
                m_waypointSet.insert( std::make_shared<NodePoint>( p ) );
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



int64_t PathFinder::findPath( const Point& start, const Point& end, PathFinder::Path* pathOut )
{
    assert( pathOut );

    std::lock_guard<std::mutex> lockGuard( m_mutex );

    if ( !isPointOnMap( start, m_mapWidth, m_mapHeight ) ) {
        std::cerr << "Start position is outside of map boundaries or is an obstacle... ";
        return -1;
    }
    if ( !isPointOnMap( end, m_mapWidth, m_mapHeight ) ) {
        std::cerr << "End position is outside of map boundaries or is an obstacle" << std::endl;
        return -1;
    }

    auto node = m_waypointSet.find( std::make_shared<NodePoint>( start ) );
    if ( node == m_waypointSet.end() ) {
        return -1;
    }

    // to align search order by shortest manhattan distance
    {
        std::cout << "Preparing search... ";
        TimeStamp ts;
        for ( auto& n : m_waypointSet ) {
            n->prepareSearch( end, &Point::manhattan );
        }
    }

    {
        std::cout << "Searching... ";
        TimeStamp ts;
        *pathOut = (*node)->findData( std::make_shared<NodePoint>( end ) );
    }

    // finalize searching, clearing up caches
    {
        std::cout << "Postsearch cleanup... ";
        TimeStamp ts;
        for ( auto& n : m_waypointSet ) {
            n->completeSearch();
        }
    }
    if ( pathOut->empty() ) {
        return -1;
    }

    // we do not want the starting point to be included, right?
    pathOut->erase( pathOut->begin() );

    return pathOut->size();
}







static void printPathInfo( int64_t pathLength, const PathFinder::Path& path )
{
    std::cout << std::endl;
    if ( pathLength == -1 ) {
        std::cout << "Path could not be found." << std::endl;
        return;
    }

    if ( pathLength == 0 ) {
        std::cout << "Starting point is also destination point." << std::endl;
        return;
    }

    std::cout << "Number of waypoints (excluding starting position):\t" << pathLength << std::endl;
    std::cout << "\nWaypoint list (index_x:y):\n\t";
    for ( const auto& p : path ) {
        assert( p );
        std::cout << p->index() << "_" << p->x() << ":" << p->y() << ", ";
    }
    std::cout << std::endl;

    std::cout << "\nWaypoint list (just index):\n\t";
    for ( const auto& p : path ) {
        assert( p );
        std::cout << p->index() << ", ";
    }
    std::cout << std::endl;
}

int main( int argc, char** argv )
{

    PathFinder::Path path;
    int64_t pathLength;

    {
        const std::vector<bool> data = {
            1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1,
            1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1,
            1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1,
            1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1,
            1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1,
            1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1
        };
        PathFinder pathFinder( &data, 16, 10 );
        pathLength = pathFinder.findPath( Point( 0, 0 ), Point( 15, 9 ), &path );
    }
//     {
//         const int64_t size = 3000;
//         std::vector<bool> data2( size * size );
//         std::fill( data2.begin(), data2.end(), 1 );

//         PathFinder pathFinder( &data2, size, size );
//         pathLength = pathFinder.findPath( Point( 0, 0 ), Point( size-1, size-1 ), &path );
//     }


    std::cout << "Input data and exact path data will be released at the program exit." << std::endl;
    printPathInfo( pathLength, path );

    return 0;
}
