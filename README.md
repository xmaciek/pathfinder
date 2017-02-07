# PathFinder
A generic template pathfinder for C++

#### Basic Example:
```C++
std::vector<std::shared_ptr<YourClass>> vectorOfYourClassPointers = ...;
PathFinder<YourClass> pathFinder( vectorOfYourClassPointers, &YourClass::adjecentObjectsFunction, &YourClass::distanceFunction );
PathInfo<YourClass> path = pathFinder.findPath( yourClassStartPointPtr, yourClassEndPointPtr );
switch ( path ) {
    case PathInfo<YourClass>::Found:
        processPath( path.begin(), path.end() ); // the .begin()/.end() returns std::vector<std::shared_ptr<YourClass>>::[const_]iterator
        break;
    case PathInfo<YourClass>::NotFound: // self explainatory
        break;
    case PathInfo<YourClass>::Invalid: // an error has occured :p
        break;
}
```
###### See `example/example.cpp` for more details.

#### Requirements:
C++11 compiler


###### This project is under MIT license.
