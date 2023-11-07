package main

/*
#include <stdlib.h>
typedef struct Vec2 {
  double x;
  double y;
} Vec2;
*/
import "C"

import (
	"fmt"
	"image"
	"unsafe"

	"github.com/superboySB/pathfind"
)

//export FindPath
func FindPath(floorPlan *C.char, startX C.double, startY C.double, endX C.double, endY C.double) (*C.Vec2, C.int) {
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
	result := (*C.Vec2)(C.malloc(C.size_t(len(path)) * C.size_t(unsafe.Sizeof(C.Vec2{}))))
	resultSlice := (*[1 << 30]C.Vec2)(unsafe.Pointer(result))[:len(path):len(path)]

	for i, pt := range path {
		resultSlice[i] = C.Vec2{
			x: C.double(pt.X),
			y: C.double(pt.Y),
		}
	}

	return result, C.int(len(path))
}
