package geom

import (
	"math"

	"github.com/flywave/go-geom"
	vec2d "github.com/flywave/go3d/float64/vec2"
	"github.com/tidwall/rtree"
)

type PointNode struct {
	Feature *geom.Feature
	Id      string
	Point   []float64
	Index   int
	normal  *vec2d.T
	visited bool
}

type MegerOpts struct {
	SearchRadius float64
	Distance     float64
}

func MegerCenterLine(fc *geom.FeatureCollection, opt *MegerOpts) {
	allNode := []*PointNode{}
	for _, f := range fc.Features {
		if f.GeometryData.Type != "LineString" {
			continue
		}

		f.ID = f.Properties["id"]
		if f.ID == nil {
			f.ID = NewUUid32()
		}

		for i, p := range f.GeometryData.LineString {
			n := &PointNode{
				Feature: f,
				Id:      f.ID.(string),
				Point:   p,
				Index:   i,
				normal:  normal(f.GeometryData.LineString),
			}
			allNode = append(allNode, n)
		}
	}

	tree := buildPointRtree(allNode)
	for _, n := range allNode {
		if n.visited {
			continue
		}
		nds := []*PointNode{}
		tree.Search(
			[2]float64{n.Point[0] - opt.SearchRadius, n.Point[1] - opt.SearchRadius},
			[2]float64{n.Point[0] + opt.SearchRadius, n.Point[1] + opt.SearchRadius},
			func(min, max [2]float64, v interface{}) bool {
				node := v.(*PointNode)
				if !node.visited {
					nds = append(nds, node)
				}
				return true
			})
		connectLines(n, nds, opt)
	}
}

func connectLines(src *PointNode, dests []*PointNode, opt *MegerOpts) {
	minDist := math.MaxFloat64
	var destNodes []*PointNode
	for _, d := range dests {
		if src.Id == d.Id {
			continue
		}
		if hasIntersection(src.Feature.GeometryData.LineString, d.Feature.GeometryData.LineString) {
			continue
		}
		if isParallel(src.normal, d.normal) {
			// dis := pointDistance(src.Point, d.Point)
			l1 := src.Feature.GeometryData.LineString
			l2 := d.Feature.GeometryData.LineString

			d1, d2 := coumputedistance(l1, l2)
			dis := math.Min(d1, d2)
			if minDist > dis && opt.Distance > dis {
				minDist = dis
				destNodes = append(destNodes, d)
			}
		}
	}

	src.visited = true
	if len(destNodes) == 0 {
		return
	}

	vp := &vec2d.T{src.Point[0], src.Point[1]}
	for _, d := range destNodes {
		vp2 := &vec2d.T{d.Point[0], d.Point[1]}
		vp.Add(vp2)
	}
	vp.Scale(1. / float64(len(destNodes)+1))

	p := src.Feature.GeometryData.LineString[src.Index]
	p[0] = vp[0]
	p[1] = vp[1]
	geom.BoundingBoxFromGeometryData(&src.Feature.GeometryData)

	for _, d := range destNodes {
		p := d.Feature.GeometryData.LineString[d.Index]
		p[0] = vp[0]
		p[1] = vp[1]
		geom.BoundingBoxFromGeometryData(&d.Feature.GeometryData)
		d.visited = true
	}
}

func buildPointRtree(col []*PointNode) *rtree.RTree {
	tree := &rtree.RTree{}
	for _, c := range col {
		tree.Insert([2]float64{c.Point[0], c.Point[1]}, [2]float64{c.Point[0], c.Point[1]}, c)
	}
	return tree
}

func megerLine(v11, v12, v21, v22 *vec2d.T) [][]float64 {
	p1 := vec2d.PointSegmentVerticalPoint(v11, v21, v22)
	p11 := vec2d.Add(v11, p1)
	p11.Scale(0.5)

	p2 := vec2d.PointSegmentVerticalPoint(v12, v21, v22)
	p21 := vec2d.Add(v12, p2)
	p21.Scale(0.5)

	p3 := vec2d.PointSegmentVerticalPoint(v21, v11, v12)
	p31 := vec2d.Add(v21, p3)
	p31.Scale(0.5)

	p4 := vec2d.PointSegmentVerticalPoint(v22, v11, v12)
	p41 := vec2d.Add(v22, p4)
	p41.Scale(0.5)

	ls := [6][2]vec2d.T{{p11, p21}, {p11, p31}, {p11, p41}, {p21, p31}, {p21, p41}, {p31, p41}}
	len := 0.0
	var res int
	for idx, ll := range ls {
		sub := vec2d.Sub(&ll[0], &ll[1])
		l := sub.Length()
		if l > len {
			len = l
			res = idx
		}
	}
	vl := ls[res]

	return [][]float64{{vl[0][0], vl[0][1], 0}, {vl[1][0], vl[1][1], 0}}

}
