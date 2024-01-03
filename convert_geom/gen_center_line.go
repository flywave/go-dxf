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

type CenterLineOpts struct {
	SearchExtend, MaxWidth, MinWidth, LineMinlength float64
}

func GenCenterLine(col *geom.FeatureCollection, opt *CenterLineOpts) *geom.FeatureCollection {
	res := geom.NewFeatureCollection()
	allNode := []*RtreeNode{}
	for _, c := range col.Features {
		if c.GeometryData.Type != "LineString" {
			continue
		}
		if c.ID == nil {
			c.ID = c.Properties["id"]
			if c.ID == nil {
				c.ID = NewUUid32()
			}
		}
		d := &RtreeNode{
			Feature: c,
			Id:      c.ID.(string),
			genMap:  make(map[string]bool),
			normal:  normal(c.GeometryData.LineString),
			length:  length(c.GeometryData.LineString),
		}
		if d.length < opt.LineMinlength {
			continue
		}
		allNode = append(allNode, d)
	}

	tree := buildRtree(allNode)
	for _, n := range allNode {
		nds := []*RtreeNode{}
		for _, p := range n.Feature.GeometryData.LineString {
			min, max := genSearchBox(p, n, opt.SearchExtend)
			tree.Search(min, max, func(min, max [2]float64, v interface{}) bool {
				node := v.(*RtreeNode)
				nds = append(nds, node)
				return true
			})
		}

		f := genMind(n, nds, opt)
		if f != nil {
			f.ID = NewUUid32()
			res.Features = append(res.Features, f)
		}
	}
	return res
}

func genSearchBox(p []float64, nd *RtreeNode, extend float64) ([2]float64, [2]float64) {
	nl := *nd.normal
	nl.Scale(extend)
	right := nl.Rotated(-math.Pi / 2)
	left := nl.Rotated(math.Pi / 2.0)
	org := &vec2d.T{p[0], p[1]}
	r := vec2d.Add(org, &right)
	l := vec2d.Add(org, &left)
	return [2]float64{math.Min(r[0], l[0]), math.Min(r[1], l[1])}, [2]float64{math.Max(r[0], l[0]), math.Max(r[1], l[1])}
}

func buildRtree(col []*RtreeNode) *rtree.RTree {
	tree := &rtree.RTree{}
	for _, c := range col {
		if c.Feature.BoundingBox == nil {
			c.Feature.BoundingBox = geom.BoundingBoxFromPoints(c.Feature.GeometryData.LineString)
		}
		bx := c.Feature.BoundingBox
		tree.Insert([2]float64{bx[0][0], bx[0][1]}, [2]float64{bx[1][0], bx[1][1]}, c)
	}
	return tree
}

func genMind(src *RtreeNode, dests []*RtreeNode, opt *CenterLineOpts) *geom.Feature {
	minDist := math.MaxFloat64
	var destNode *RtreeNode
	for _, d := range dests {
		if src.Id == d.Id {
			continue
		}
		if !hasIntersection(src.Feature.GeometryData.LineString, d.Feature.GeometryData.LineString) {
			continue
		}
		if isParallel(src.normal, d.normal) {
			d1, d2 := coumputedistance(src.Feature.GeometryData.LineString, d.Feature.GeometryData.LineString)
			dis := math.Min(d1, d2)
			if minDist > dis && opt.MinWidth < dis && dis < opt.MaxWidth {
				minDist = dis
				destNode = d
			}
		}
	}

	if destNode == nil {
		return nil
	}

	if destNode.genMap[src.Id] && src.genMap[destNode.Id] {
		return nil
	}

	line := coumputeMidline(src, destNode)
	return geom.NewLineStringFeature(line)
}
