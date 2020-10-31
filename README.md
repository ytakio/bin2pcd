# BIN to PCD Converter

Based on ply2pcd in PCL. It simply convert simple format in the following to PCD format.

## Input format

**point_data.h**
```cpp
typedef struct {
	struct {
		float x,y,z;
	};
	struct {
		uint8_t r,g,b,a;
	};
} point_data_t;
```

## Build

```sh
$ mkdir build; cd build
$ cmake -DCMAKE_BUILD_TYPE=Release
```

## Usage

```sh
$ ./build/bin2pcd [input.bin] [output.pcd]
$ pcl_viewer [output.pcd]
```

