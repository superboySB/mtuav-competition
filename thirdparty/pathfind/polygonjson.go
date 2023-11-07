// Copyright 2023 Frederik Zipp. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"encoding/json"
	"fmt"
	"image"
)

// 使用 json.Decoder 从 io.Reader 流式读取和解析 JSON
// func polygonsFromJSONStream(r io.Reader) ([][]image.Point, image.Point, error) {
// 	var jsonStruct struct {
// 		Canvas struct {
// 			W int `json:"w"`
// 			H int `json:"h"`
// 		} `json:"canvas"`
// 		Polygons [][]struct {
// 			X int `json:"x"`
// 			Y int `json:"y"`
// 		} `json:"polygons"`
// 	}

// 	dec := json.NewDecoder(r)
// 	if err := dec.Decode(&jsonStruct); err != nil {
// 		return nil, image.Point{}, fmt.Errorf("could not decode polygon JSON: %w", err)
// 	}

// 	size := image.Pt(jsonStruct.Canvas.W, jsonStruct.Canvas.H)
// 	polygons := make([][]image.Point, 0, len(jsonStruct.Polygons))

// 	for _, jsonPolygon := range jsonStruct.Polygons {
// 		polygon := make([]image.Point, len(jsonPolygon))
// 		for i, v := range jsonPolygon {
// 			polygon[i] = image.Pt(v.X, v.Y)
// 		}
// 		polygons = append(polygons, polygon)
// 	}

// 	return polygons, size, nil
// }

//
// [Polygon Constructor]: https://alaricus.github.io/PolygonConstructor/
func polygonsFromJSON(jsonData []byte) (ps [][]image.Point, size image.Point, err error) {
	var jsonStruct polygonToolJSON
	err = json.Unmarshal(jsonData, &jsonStruct)
	if err != nil {
		return nil, image.Point{}, fmt.Errorf("could not unmarshal polygon JSON: %w", err)
	}
	return jsonStruct.polygons(), jsonStruct.size(), nil
}

type polygonToolJSON struct {
	Canvas struct {
		W int `json:"w"`
		H int `json:"h"`
	} `json:"canvas"`
	Polygons [][]struct {
		X float64 `json:"x"`
		Y float64 `json:"y"`
	} `json:"polygons"`
}

func (pj polygonToolJSON) size() image.Point {
	return image.Pt(pj.Canvas.W, pj.Canvas.H)
}

func (pj polygonToolJSON) polygons() [][]image.Point {
	ps := make([][]image.Point, len(pj.Polygons))
	for i, jsonPolygon := range pj.Polygons {
		p := make([]image.Point, len(jsonPolygon))
		for j, v := range jsonPolygon {
			p[j] = image.Pt(int(v.X), int(v.Y))
		}
		ps[i] = p
	}
	return ps
}
