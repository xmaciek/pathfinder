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

#include <cassert>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <set>
#include <vector>

#include "pathfinder_version.h"
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
            DEBUG_ERROR( "Start position is not known as waypoint" );
            return PathInfo<T>();
        }

        typename Node<T>::Ptr endNode = std::make_shared<Node<T>>( end );
        const typename WaypointSet::iterator endNodeIt = m_waypointSet.find( endNode );
        if ( endNodeIt == m_waypointSet.end() ) {
            DEBUG_ERROR( "End position is not known as waypoint" );
            return PathInfo<T>();
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
