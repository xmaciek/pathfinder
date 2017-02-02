



#pragma once

#include <algorithm>
#include <functional>
#include <memory>
#include <set>
#include <vector>

// helper class meant to automatically release lock when leaving scope
class TravelLock {
public:
    inline TravelLock( bool& b ) : m_lock( b ) { m_lock = true; }
    inline ~TravelLock() { m_lock = false; }

protected:

private:
    bool& m_lock;
    TravelLock( const TravelLock& );
    TravelLock& operator = ( const TravelLock& );
};

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
