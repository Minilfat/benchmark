 
## Introduction:

Initialization time and searching points within some distance time comparison for different octree implementations:
 <https://github.com/BioDynaMo/biodynamo/tree/master/src/spatial_organization>
and <https://github.com/jbehley/octree>


## Build Instructions

Check out code from this repository:
```
git clone https://github.com/Minilfat/benchmark.git
cd benchmark
```

Create build directory
```
mkdir build 
cd build
```

Configure and build all targets
```
cmake ..
make
```

Run programm with test data from benchmark/points folder
```
./benchmark ../points/10k.csv
```
