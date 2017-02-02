#pragma once
#if __cplusplus < 201103L
#error You need a compiller which supports atleast c++11, fully.
#endif

#include <cassert>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <set>
#include <vector>

#include "node.hpp"
#include "timestamp.hpp"

template<typename T>
class PathFinder {
public:
    typedef std::shared_ptr<T> DataPtr;
    typedef std::function< std::vector<DataPtr>( const DataPtr& )> AdjecentNodeFunction;

protected:
    std::mutex m_mutex;
    typedef std::set< typename Node<T>::Ptr, typename Node<T>::CompareFunction > WaypointSet;
    WaypointSet m_waypointSet;

public:
    PathFinder( const std::vector<DataPtr>& waypoints, const AdjecentNodeFunction& adjNodeFunc,
                const typename Node<T>::DistanceFunction& distanceFunction ) :
        m_waypointSet( &Node<T>::cmpLT )
    {
        // create nodes
        {
            DEBUG_DURATION( "Creating nodes and attaching data..." );
            for ( const DataPtr& w : waypoints ) {
                typename Node<T>::Ptr node = std::make_shared<Node<T>>( w );
                node->setDistanceFunction( distanceFunction );
                m_waypointSet.insert( node );
            }
        }

        // assembly outgoing nodes
        {
            DEBUG_DURATION( "Assembling node connections..." );
            for ( auto& node : m_waypointSet ) {
                std::vector<DataPtr> adjecentNodeData = adjNodeFunc( node->data() );
                for ( DataPtr& d : adjecentNodeData ) {
                    typename WaypointSet::iterator it = m_waypointSet.find( std::make_shared<Node<T>>( d ) );
                    if ( it != m_waypointSet.end() ) {
                        node->addAdjecentNode( *it );
                    } else {
                        assert( !"the adjecent node is not known as waypoint" );
                    }
                }
            }
        }
    }

    ~PathFinder()
    {
        DEBUG_DURATION( "Releasig node resources..." );
        for ( auto& n : m_waypointSet ) {
            n->reset();
        }
    }

    // returns path information, including start point
    PathInfo<T> findPath( const T& start, const T& end )
    {
        return findPath( std::make_shared<T>( start ), std::make_shared<T>( end ) );
    }

    // returns path information, including start point
    PathInfo<T> findPath( const DataPtr& start, const DataPtr& end )
    {
        std::lock_guard<std::mutex> lockGuard( m_mutex );

        typename Node<T>::Ptr startNode = std::make_shared<Node<T>>( start );
        const typename WaypointSet::iterator startNodeIt = m_waypointSet.find( startNode );
        if ( startNodeIt == m_waypointSet.end() ) {
            std::cerr << "Start position is not known as waypoint";
            return PathInfo<T>();
        }

        typename Node<T>::Ptr endNode = std::make_shared<Node<T>>( end );
        const typename WaypointSet::iterator endNodeIt = m_waypointSet.find( endNode );
        if ( endNodeIt == m_waypointSet.end() ) {
            std::cerr << "End position is not known as waypoint" << std::endl;
            return PathInfo<T>( end );
        }

        // peeking at the cache
        {
            DEBUG_DURATION( "Browsing cache..." );
            const PathInfo<T> pi = startNode->cacheLookup( (*endNodeIt)->data() );
            switch ( pi ) {
                case PathInfo<T>::Invalid:
                    DEBUG_MESSAGE( "not found " );
                    break;
                case PathInfo<T>::Found:
                    DEBUG_MESSAGE( "found, skipping search " );
                    return pi;
                case PathInfo<T>::NotFound:
                    DEBUG_MESSAGE( "path is known to not exists, skipping search " );
                    return pi;
                default:
                    assert( !"Unhandled enum" );
            }
        }

        // checking if the node can be reached from starting position
        {
            DEBUG_DURATION( "Checking if destination is reachable..." );
            if ( (*startNodeIt)->isReachable( *endNodeIt ) ) {
                DEBUG_MESSAGE( "yes " );
            } else {
                DEBUG_MESSAGE( "no, skipping search " );
                return PathInfo<T>( (*endNodeIt)->data() );
            }
        }

        // the searching
        PathInfo<T> path;
        {
            DEBUG_DURATION( "Searching..." );
            path = (*startNodeIt)->findData( *endNodeIt );
        }

        // finalize searching, clearing minimum distance helper variable
        {
            DEBUG_DURATION( "Postsearch activity..." );
            for ( auto& n : m_waypointSet ) {
                n->completeSearch();
            }
        }

        return path;
    }
};
