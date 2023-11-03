package main

/*
#include <stdlib.h>
typedef struct Vec3 {
  double x;
  double y;
  double z;
} Vec3;
*/
import "C"

import (
	"fmt"
	"image"
	"unsafe"

	"github.com/fzipp/pathfind"
)

//export FindPath
func FindPath(floorPlan *C.char, startX C.double, startY C.double, endX C.double, endY C.double,
	height C.double) (*C.Vec3, C.int) {
	plan := C.GoString(floorPlan)
	polygons, _, err := polygonsFromJSON([]byte(plan))
	if err != nil {
		return nil, 0
	}
	start := image.Pt(int(startX), int(startY))
	destination := image.Pt(int(endX), int(endY))

	pathfinder := pathfind.NewPathfinder(polygons)
	path := pathfinder.Path(start, destination)
	fmt.Println(path)

	// 在 C 中为结果数组分配内存
	result := (*C.Vec3)(C.malloc(C.size_t(len(path)) * C.size_t(unsafe.Sizeof(C.Vec3{}))))
	resultSlice := (*[1 << 30]C.Vec3)(unsafe.Pointer(result))[:len(path):len(path)]

	for i, pt := range path {
		resultSlice[i] = C.Vec3{
			x: C.double(pt.X),
			y: C.double(pt.Y),
			z: height,
		}
	}

	return result, C.int(len(path))
}
