#if __cplusplus < 201103L
#error You need a compiller which supports atleast c++11.
#endif

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <functional>
#include <iostream>
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

protected:
    DataPtr m_dataPtr;
    bool m_isLocked;
    std::vector<Ptr> m_adjecentNodeVector;

public:
    Node( const T& p ) :
        // the data is explicitly a shared pointer to keep result values valid after deletion of Node
        m_dataPtr( std::make_shared<T>( p ) ),
        m_isLocked( false )
    {}

    void addAdjecentNode( Ptr n )
    {
        assert( n );
        m_adjecentNodeVector.push_back( n );
    };

    // NOTE: meant to be called in ~PathFinder()
    void detachAll()
    {
        m_adjecentNodeVector.clear();
    }

    Path findData( Ptr p )
    {
        assert( p );
        assert( m_dataPtr );

        if ( *m_dataPtr == *(p->m_dataPtr) ) {
            return { m_dataPtr };
        }
        if ( m_isLocked ) {
            return Path();
        }

        TravelLock lock( m_isLocked );

        Path adjecentPath;
        for ( Ptr direction : m_adjecentNodeVector ) {
            Path tmp = direction->findData( p );
            if ( tmp.empty() ) {
                continue;
            }
            if ( ( adjecentPath.empty() ) || tmp.size() < adjecentPath.size() ) {
                std::swap( tmp, adjecentPath );
            }
        }

        if ( adjecentPath.empty() ) {
            return Path();
        }

        Path path = { m_dataPtr };
        path.insert( path.end(), adjecentPath.begin(), adjecentPath.end() );
        return path;
    }

    // comarsion function for std::set
    static bool cmpLT( const Node<T>::Ptr a, const Node<T>::Ptr b )
    {
        assert( a );
        assert( a->m_dataPtr );
        assert( b );
        assert( b->m_dataPtr );
        return *(a->m_dataPtr) < *(b->m_dataPtr);
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



typedef Node<Point> NodePoint;

class PathFinder {
protected:
    std::mutex m_mutex;
    std::set<NodePoint::Ptr, std::function<bool(const NodePoint::Ptr, const NodePoint::Ptr)> > m_waypointSet;

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
    for ( auto& n : m_waypointSet ) {
        n->detachAll();
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

// explicit manifestation of laziness for writting same code with decorative changes
#define ADD_ADJECENT_NODE( x, y ) {\
    auto nodeToAdd = m_waypointSet.find( std::make_shared<NodePoint>( Point( x, y ) ) ); \
    if ( nodeToAdd != m_waypointSet.end() ) { (*node)->addAdjecentNode( *nodeToAdd ); } \
}

PathFinder::PathFinder( const std::vector<bool>* waypoints, int64_t mapWidth, int64_t mapHeight ) :
    m_waypointSet( &NodePoint::cmpLT ),
    m_mapWidth( mapWidth ),
    m_mapHeight( mapHeight )
{
    assert( waypoints );
    assert( waypoints->size() == static_cast<uint64_t>( mapWidth * mapHeight ) );

    // convert grid with obstacles into waypoint nodes
    for ( size_t i = 0; i < waypoints->size(); i++ ) {
        const Point p = iToPoint( i, mapWidth );
        if ( !isObstacle( waypoints, p, mapWidth, mapHeight ) ) {
            m_waypointSet.insert( std::make_shared<NodePoint>( p ) );
        }
    }

    // connect waypoint nodes
    for ( int64_t x = 0; x < mapWidth; x++ ) {
    for ( int64_t y = 0; y < mapHeight; y++ ) {
        NodePoint::Ptr p = std::make_shared<NodePoint>( Point( x, y ) );
        auto node = m_waypointSet.find( p );
        if ( node == m_waypointSet.end() ) {
            continue;
        }
        ADD_ADJECENT_NODE( x, y - 1 );
        ADD_ADJECENT_NODE( x, y + 1 );
        ADD_ADJECENT_NODE( x - 1, y );
        ADD_ADJECENT_NODE( x + 1, y );
    }
    }
}

#undef ADD_ADJECENT_NODE


int64_t PathFinder::findPath( const Point& start, const Point& end, PathFinder::Path* pathOut )
{
    assert( pathOut );

    std::lock_guard<std::mutex> lockGuard( m_mutex );
    if ( !isPointOnMap( start, m_mapWidth, m_mapHeight ) ) {
        std::cerr << "Start position is outside of map boundaries or is an obstacle" << std::endl;
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

    *pathOut = (*node)->findData( std::make_shared<NodePoint>( end ) );
    if ( pathOut->empty() ) {
        return -1;
    }

    // we do not want the starting point to be included, right?
    pathOut->erase( pathOut->begin() );
    return pathOut->size();
}

static void printPathInfo( int64_t pathLength, const PathFinder::Path& path )
{
    if ( pathLength == -1 ) {
        std::cout << "Path could not be found." << std::endl;
        return;
    }

    if ( pathLength == 0 ) {
        std::cout << "Starting point is also destination point." << std::endl;
        return;
    }

    std::cout << "Number of waypoints (excluding starting position):\t" << pathLength << std::endl;
    std::cout << "Waypoint list (index_x:y):\t";
    for ( const auto& p : path ) {
        assert( p );
        std::cout << p->index() << "_" << p->x() << ":" << p->y() << ", ";
    }
    std::cout << std::endl;

    std::cout << "Waypoint list (just index):\t";
    for ( const auto& p : path ) {
        assert( p );
        std::cout << p->index() << ", ";
    }
    std::cout << std::endl;
}

int main( int argc, char** argv )
{

    const std::vector<bool> data = { 1, 1, 1, 1, 1, 1, 1, 1,
                                     1, 0, 1, 0, 0, 0, 1, 1,
                                     1, 1, 1, 1, 1, 1, 1, 1,
                                     1, 0, 1, 0, 0, 1, 0, 1,
                                     1, 1, 1, 1, 0, 1, 1, 1 };
    int size = 400;
    std::vector<bool> data2( size * size );
    std::fill( data2.begin(), data2.end(), 1 );
//     PathFinder pathFinder( &data, 8, 5 );
    PathFinder pathFinder( &data2, size, size );
    PathFinder::Path path;
    const int64_t pathLength = pathFinder.findPath( Point( 0, 0 ), Point( 7, 4 ), &path );
    printPathInfo( pathLength, path );

    // thread safety: if multiple threads needs to sieve shared data,
    // use thread_local storage specifier on PathFinder and related return values as needed
    return 0;
}
