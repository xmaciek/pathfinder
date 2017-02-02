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


// helper class meant to automatically release lock when leaving scope
class TravelLock {
public:
    TravelLock( bool& b );
    ~TravelLock();

protected:
    bool& m_lock;

private:
    TravelLock( const TravelLock& );
    TravelLock& operator = ( const TravelLock& );
};

TravelLock::TravelLock( bool& b ) :
    m_lock( b )
{
    b = true;
}

TravelLock::~TravelLock()
{
    m_lock = false;
}

template<typename T>
class Node {
public:
    typedef std::shared_ptr<Node<T>> Ptr;
    typedef std::shared_ptr<T> DataPtr;
    typedef std::vector<DataPtr> Path;
    typedef std::function<int64_t( const T&, const T& )> DistanceFunction;

    class PathInfo {
    public:
        enum Enum { Invalid, Found, NotFound };

        bool m_isShortest;
        DataPtr m_endPoint;
        Path m_path;

        PathInfo( const DataPtr& p = nullptr ) : m_isShortest( false ), m_endPoint( p ) {};

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

        void setPath( const Path& p )
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

protected:
    bool m_isLocked;
    DataPtr m_dataPtr;
    std::vector<Node<T>::Ptr> m_adjecentNodeVector;
    DistanceFunction m_distanceFunction;

    int64_t m_minDistance;

    // basically a cache of found paths, so we do not have to traverse again this node
    std::set<Node<T>::PathInfo> m_traveledPathSet;

    // NOTE: appends vector of nodes which can be traversed,
    // needed to predict if destination point can be founded.
    // this function has to be used within isReachable();
    void getTraversableScope( std::vector<Node<T>*>* s )
    {
        assert( s );
        if ( isLocked() ) {
            return;
        }
        lock();

        s->push_back( this );
        for ( auto& adj : m_adjecentNodeVector ) {
            assert( adj );
            adj->getTraversableScope( s );
        }
    }

    inline void lock() { m_isLocked = true; }
    inline void unlock() { m_isLocked = false; }
    inline bool isLocked() const { return m_isLocked; }

    // because cannot capture 'this' in lambda functions in c++ < c++14
    bool distancePrepareSearch( const Node<T>::Ptr& a, const Node<T>::Ptr& b, const T& t ) const
    {
        assert( a );
        assert( b );
        assert( a->m_dataPtr );
        assert( b->m_dataPtr );
        assert( m_distanceFunction );
        return m_distanceFunction( *(a->m_dataPtr), t ) < m_distanceFunction( *(b->m_dataPtr), t );
    }

    // align search order by shortest manhattan distance
    void prepareSearch( const T& t )
    {
        assert( m_dataPtr );
        assert( m_distanceFunction );

        // already aligned
        if ( m_minDistance != -1 ) {
            return;
        }
        m_minDistance = m_distanceFunction( *m_dataPtr, t );
        std::sort( m_adjecentNodeVector.begin(), m_adjecentNodeVector.end(),
                   std::bind( &Node<T>::distancePrepareSearch, this, std::placeholders::_1, std::placeholders::_2, t ) );
    }

public:
    Node( const T& p ) :
        m_isLocked( false ),
        // copy of data is explicitly stored as shared pointer to keep result values valid after deletion of Node
        m_dataPtr( std::make_shared<T>( p ) ),
        m_minDistance( -1 )
    {}


    void addAdjecentNode( Node<T>::Ptr n )
    {
        assert( n );
        m_adjecentNodeVector.push_back( n );
    };


    void cachePath( const Node<T>::PathInfo& p )
    {
        assert( m_traveledPathSet.find( p ) == m_traveledPathSet.end() );
        m_traveledPathSet.insert( p );
    }

    // NOTE: the endPoint may exist, but not be rachable from current position
    // in such case its pointless to brute-force search for it
    bool isReachable( const Node<T>::Ptr& endPoint )
    {
        assert( endPoint );
        std::vector<Node<T>*> traversableScope;
        getTraversableScope( &traversableScope );
        std::sort( traversableScope.begin(), traversableScope.end() );
        const bool found = std::binary_search( traversableScope.begin(), traversableScope.end(), endPoint.get() );
        const PathInfo unreachablePoint( endPoint->m_dataPtr );
        for ( auto& node : traversableScope ) {
            assert( node );
            // already mark it as not found
            if ( !found ) {
                node->cachePath( unreachablePoint );
            }
            node->unlock();
        }
        return found;
    }

    Node<T>::DataPtr data() const
    {
        return m_dataPtr;
    }

    // NOTE: meant to be called in ~PathFinder(), otherwise we get memory leak.
    // this is due to Nodes beign held as shared pointers by PathFinder, and they hold
    // same shared pointers to each other as adjecent nodes.
    void reset()
    {
        m_minDistance = -1;
        m_adjecentNodeVector.clear();
    }

    PathInfo findData( const Node<T>::Ptr& p )
    {
        assert( p );
        assert( m_dataPtr );

        if ( isLocked() ) {
            return PathInfo();
        }

        TravelLock travelLock( m_isLocked );

        PathInfo pi = cacheLookup( p->m_dataPtr );
        if ( pi != PathInfo::Invalid ) {
            return pi;
        }

        // is equal
        if ( *m_dataPtr == *(p->m_dataPtr) ) {
            pi = PathInfo( m_dataPtr );
            pi.setPath( { m_dataPtr } );
            pi.setShortest( true );
            cachePath( pi );
            return pi;
        }

        // start searching
        PathInfo adjecentPath;
        prepareSearch( *(p->m_dataPtr) );
        for ( auto& direction : m_adjecentNodeVector ) {
            PathInfo tmp = direction->findData( p );
            if ( tmp == PathInfo::Invalid ) {
                continue;
            }
            if ( tmp.isShortest() ) {
                adjecentPath = tmp;
                break;
            }
            if ( tmp.score() > adjecentPath.score() ) {
                std::swap( tmp, adjecentPath );
            }
        }

        // intrepret search result
        switch ( adjecentPath ) {
            case PathInfo::Found:
                adjecentPath.m_path.insert( adjecentPath.m_path.begin(), m_dataPtr );
                adjecentPath.setShortest( adjecentPath.size() == static_cast<uint64_t>( m_minDistance + 1 ) );
            case PathInfo::NotFound:
                cachePath( adjecentPath );
            case PathInfo::Invalid:
                break;
            default:
                assert( !"Unhandled enum" );
        }
        return adjecentPath;
    }

    void setDistanceFunction( const DistanceFunction& distanceFunction )
    {
        m_distanceFunction = distanceFunction;
    }

    void completeSearch()
    {
        m_minDistance = -1;
    }

    Node<T>::PathInfo cacheLookup( const Node<T>::DataPtr& t ) const
    {
        const auto it = m_traveledPathSet.find( t );
        return it != m_traveledPathSet.end() ? *it : PathInfo();
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
