package geom

import (
	"math"

	"github.com/flywave/go-geom"
	vec2d "github.com/flywave/go3d/float64/vec2"
	"github.com/tidwall/rtree"
)

type RtreeNode struct {
	Feature *geom.Feature
	Id      string
	genMap  map[string]bool
	normal  *vec2d.T
	length  float64
}

func GenMidLine(col *geom.FeatureCollection, extend float64) *geom.FeatureCollection {
	res := geom.NewFeatureCollection()
	allNode := []*RtreeNode{}
	for _, c := range col.Features {
		if c.GeometryData.Type != "LineString" {
			continue
		}
		c.ID = c.Properties["id"]
		if c.ID == nil {
			c.ID = NewUUid32()
		}
		d := &RtreeNode{
			Feature: c,
			Id:      c.ID.(string),
			genMap:  make(map[string]bool),
			normal:  normal(c.GeometryData.LineString),
			length:  length(c.GeometryData.LineString),
		}
		if d.length < 5 {
			continue
		}
		allNode = append(allNode, d)
	}

	tree := buildRtree(allNode)
	for _, n := range allNode {
		bx := n.Feature.BoundingBox
		nds := []*RtreeNode{}
		tree.Search([2]float64{bx[0] - extend, bx[1] - extend}, [2]float64{bx[2] + extend, bx[3] + extend}, func(min, max [2]float64, v interface{}) bool {
			node := v.(*RtreeNode)
			nds = append(nds, node)
			return true
		})

		f := genMind(n, nds)
		if f != nil {
			f.ID = NewUUid32()
			res.Features = append(res.Features, f)
		}
	}
	return res
}

func buildRtree(col []*RtreeNode) *rtree.RTree {
	tree := &rtree.RTree{}
	for _, c := range col {
		if c.Feature.BoundingBox == nil {
			c.Feature.BoundingBox = geom.BoundingBoxFromPoints(c.Feature.GeometryData.LineString)
		}
		bx := c.Feature.BoundingBox
		tree.Insert([2]float64{bx[0], bx[1]}, [2]float64{bx[2], bx[3]}, c)
	}
	return tree
}

func genMind(src *RtreeNode, dests []*RtreeNode) *geom.Feature {
	minDist := math.MaxFloat64
	var destNode *RtreeNode
	var distance1, distance2 float64
	for _, d := range dests {
		if isParallel(src.normal, d.normal) {
			distance1, distance2 = coumputedistance(src.Feature.GeometryData.LineString, d.Feature.GeometryData.LineString)
			dis := math.Min(distance1, distance2)
			if minDist > dis && dis > 0.5 {
				minDist = dis
				destNode = d
			}
		}
	}

	if destNode == nil {
		return nil
	}

	if destNode.genMap[src.Id] || src.genMap[destNode.Id] {
		return nil
	}

	// if destNode.length < src.length {
	// 	return destNode.Feature
	// } else {
	// 	return src.Feature
	// }
	line := coumputeMidline(src, destNode, distance1, distance2)
	return geom.NewLineStringFeature(line)
}

func normal(l1 [][]float64) *vec2d.T {
	if len(l1) < 2 {
		return nil
	}

	v11 := &vec2d.T{l1[0][0], l1[0][1]}
	v12 := &vec2d.T{l1[1][0], l1[1][1]}
	return v12.Sub(v11).Normalize()
}

func length(l1 [][]float64) float64 {
	if len(l1) < 2 {
		return 0
	}

	v11 := &vec2d.T{l1[0][0], l1[0][1]}
	v12 := &vec2d.T{l1[1][0], l1[1][1]}
	return v12.Sub(v11).Length()
}

func dot(n1, n2 *vec2d.T) float64 {
	return vec2d.Dot(n1, n2)
}

func isParallel(l1, l2 *vec2d.T) bool {
	f := dot(l1, l2)
	ang := math.Acos(f) / math.Pi * 180
	return ang < 15
}

func coumputedistance(src, dest [][]float64) (float64, float64) {
	v11 := &vec2d.T{src[0][0], src[0][1]}
	v12 := &vec2d.T{src[1][0], src[1][1]}

	v21 := &vec2d.T{dest[0][0], dest[0][1]}
	v22 := &vec2d.T{dest[1][0], dest[1][1]}

	d1 := vec2d.PointSegmentDistance(v11, v21, v22)
	d2 := vec2d.PointSegmentDistance(v12, v21, v22)
	return d1, d2
}

func coumputeMidline(src, dest *RtreeNode, dis1, dis2 float64) [][]float64 {
	src.genMap[dest.Id] = true
	dest.genMap[src.Id] = true
	v11 := &vec2d.T{src.Feature.GeometryData.LineString[0][0], src.Feature.GeometryData.LineString[0][1]}
	v12 := &vec2d.T{src.Feature.GeometryData.LineString[1][0], src.Feature.GeometryData.LineString[1][1]}

	v21 := &vec2d.T{dest.Feature.GeometryData.LineString[0][0], dest.Feature.GeometryData.LineString[0][1]}
	v22 := &vec2d.T{dest.Feature.GeometryData.LineString[1][0], dest.Feature.GeometryData.LineString[1][1]}

	d1 := v11.Add(v21).Scale(0.5)
	d2 := v12.Add(v22).Scale(0.5)

	// destNl := *dest.normal
	// f := dot(src.normal, &destNl)
	// if f < 0 {
	// 	destNl = destNl.Inverted()
	// }
	// destNl = *destNl.Rotate90DegRight().Normalize()

	// d1 := destNl.Scaled(dis1 / 2)
	// d2 := destNl.Scaled(dis2 / 2)

	// d1 = vec2d.Add(v11, &d1)
	// d2 = vec2d.Add(v12, &d2)

	// return [][]float64{{d1[0], d1[1], 0}, {d2[0], d2[1], 0}}

	return [][]float64{{d1[0], d1[1], 0}, {d2[0], d2[1], 0}}
}
