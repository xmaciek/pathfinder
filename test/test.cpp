#include <gtest/gtest.h>

#include "../include/pathfinder.hpp"

static std::vector<std::shared_ptr<int64_t>> getData()
{
    std::vector<std::shared_ptr<int64_t>> data;
    for ( auto i=0; i<100; i++ ) {
        data.push_back( std::make_shared<int64_t>( i ) );
    }

    for ( auto i=200; i<300; i++ ) {
        data.push_back( std::make_shared<int64_t>( i ) );
    }
    return data;
}

static bool cmpLT( const std::shared_ptr<int64_t>& a, const std::shared_ptr<int64_t>& b )
{
    assert( a );
    assert( b );
    return *a < *b;
}

static std::vector<std::shared_ptr<int64_t>> adjecentFor( const std::shared_ptr<int64_t>& p, const std::vector<std::shared_ptr<int64_t>>& data )
{
    std::vector<std::shared_ptr<int64_t>> adjecent;
    if ( !std::binary_search( data.cbegin(), data.cend(), p, &cmpLT ) ) {
        return adjecent;
    }
    std::vector<std::shared_ptr<int64_t>>::const_iterator it = std::lower_bound( data.cbegin(), data.cend(), p, &cmpLT );
    if ( it != data.cbegin() ) {
        if ( **(it-1) == **(it)-1 ) {
            adjecent.push_back( *( it - 1 ) );
        }
    }
    it++;
    if ( it != data.cend() ) {
        if ( **(it-1) == **(it)-1 ) {
            adjecent.push_back( *it );
        }
    }
    return adjecent;
}


static int64_t manhattan( int64_t a, int64_t b )
{
    return abs( a - b );
}


#define GET_PATH( start, end ) \
    PathFinder<int64_t> pathfinder( data, adjecentFunction, &manhattan ); \
    const PathInfo<int64_t> path = pathfinder.findPath( start, end );


// testing 1d paths as its easier to implement and test
TEST( PathFinder, findPath )
{
    // the range is <0, 100) + <200, 300)
    const std::vector<std::shared_ptr<int64_t>> data = getData();
    auto adjecentFunction = std::bind( &adjecentFor, std::placeholders::_1, data );

    // path 1 exists
    {
        GET_PATH( 16, 10 );
        EXPECT_EQ( path, PathInfo<int64_t>::Found );
        EXPECT_EQ( path.size(), 7 );
        EXPECT_GT( path.score(), 0 );
    }

    // path 2 exists
    {
        GET_PATH( 10, 16 );
        EXPECT_EQ( path, PathInfo<int64_t>::Found );
        EXPECT_EQ( path.size(), 7 );
        EXPECT_GT( path.score(), 0 );
    }

    // path not found because there is no connection between points
    {
        GET_PATH( 20, 220 );
        EXPECT_EQ( path, PathInfo<int64_t>::NotFound );
        EXPECT_EQ( path.size(), 0 );
        EXPECT_EQ( path.score(), 0 );
    }

    // path not found because there is no connection between points
    {
        GET_PATH( 220, 20 );
        EXPECT_EQ( path, PathInfo<int64_t>::NotFound );
        EXPECT_EQ( path.size(), 0 );
        EXPECT_EQ( path.score(), 0 );
    }

    // path invalid because -10 does not exists as start point
    {
        GET_PATH( -10, 16 );
        EXPECT_EQ( path, PathInfo<int64_t>::Invalid );
        EXPECT_EQ( path.size(), 0 );
        EXPECT_EQ( path.score(), 0 );
    }

    // path invalid because 160 does not exists as start point
    {
        GET_PATH( 10, 160 );
        EXPECT_EQ( path, PathInfo<int64_t>::Invalid );
        EXPECT_EQ( path.size(), 0 );
        EXPECT_EQ( path.score(), 0 );
    }

    // path invalid because -10 does not exists as end point
    {
        GET_PATH( 60, -10 );
        EXPECT_EQ( path, PathInfo<int64_t>::Invalid );
        EXPECT_EQ( path.size(), 0 );
        EXPECT_EQ( path.score(), 0 );
    }

    // path invalid because both points do not exists
    {
        GET_PATH( -20, 160 );
        EXPECT_EQ( path, PathInfo<int64_t>::Invalid );
        EXPECT_EQ( path.size(), 0 );
        EXPECT_EQ( path.score(), 0 );
    }

    // path invalid because both points do not exists
    {
        GET_PATH( 160, -20 );
        EXPECT_EQ( path, PathInfo<int64_t>::Invalid );
        EXPECT_EQ( path.size(), 0 );
        EXPECT_EQ( path.score(), 0 );
    }

}




int main( int argc, char** argv )
{
    testing::InitGoogleTest( &argc, argv );
    return RUN_ALL_TESTS();
}
