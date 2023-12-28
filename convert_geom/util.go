package geom

import (
	"math"

	vec2d "github.com/flywave/go3d/float64/vec2"
)

func hasIntersection(l1, l2 [][]float64) bool {
	v11 := &vec2d.T{l1[0][0], l1[0][1]}
	v12 := &vec2d.T{l1[1][0], l1[1][1]}

	v21 := &vec2d.T{l2[0][0], l2[0][1]}
	v22 := &vec2d.T{l2[1][0], l2[1][1]}
	b := HasIntersection(v11, v21, v22) || HasIntersection(v12, v21, v22)
	if !b {
		return IsVertical(v11, v21, v22) && IsVertical(v12, v21, v22)
	} else {
		return b
	}
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
	ang := math.Acos(math.Abs(f)) / math.Pi * 180
	return ang < 15 || math.Abs(math.Abs(f)-1) < 1e-5
}

func pointDistance(p1, p2 []float64) float64 {

	v11 := &vec2d.T{p1[0], p1[1]}
	v12 := &vec2d.T{p2[0], p2[1]}
	return v12.Sub(v11).Length()
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

func coumputeMidline(src, dest *RtreeNode) [][]float64 {
	src.genMap[dest.Id] = true
	dest.genMap[src.Id] = true
	v11 := &vec2d.T{src.Feature.GeometryData.LineString[0][0], src.Feature.GeometryData.LineString[0][1]}
	v12 := &vec2d.T{src.Feature.GeometryData.LineString[1][0], src.Feature.GeometryData.LineString[1][1]}

	v21 := &vec2d.T{dest.Feature.GeometryData.LineString[0][0], dest.Feature.GeometryData.LineString[0][1]}
	v22 := &vec2d.T{dest.Feature.GeometryData.LineString[1][0], dest.Feature.GeometryData.LineString[1][1]}

	var d1, d2 *vec2d.T
	destNl := *dest.normal
	f := dot(src.normal, &destNl)
	if f < 0 {
		d1 = v11.Add(v22).Scale(0.5)
		d2 = v12.Add(v21).Scale(0.5)
	} else {
		d1 = v11.Add(v21).Scale(0.5)
		d2 = v12.Add(v22).Scale(0.5)
	}

	return [][]float64{{d1[0], d1[1], 0}, {d2[0], d2[1], 0}}
}

func HasIntersection(p *vec2d.T, x1 *vec2d.T, x2 *vec2d.T) bool {
	v1 := vec2d.Sub(p, x1)
	v2 := vec2d.Sub(p, x2)

	v3 := vec2d.Sub(x2, x1)
	v4 := vec2d.Sub(x1, x2)

	if v1.IsZero() || v2.IsZero() {
		return false
	}

	vn1 := v1.Normalized()
	vn3 := v3.Normalized()

	vn2 := v2.Normalized()
	vn4 := v4.Normalized()

	return vec2d.Dot(&vn1, &vn3) > 1e-2 && vec2d.Dot(&vn2, &vn4) > 1e-2
}

func IsVertical(p *vec2d.T, x1 *vec2d.T, x2 *vec2d.T) bool {
	v1 := vec2d.Sub(p, x1)
	v2 := vec2d.Sub(p, x2)

	v3 := vec2d.Sub(x2, x1)
	v4 := vec2d.Sub(x1, x2)

	if v1.IsZero() || v2.IsZero() {
		return false
	}

	vn1 := v1.Normalized()
	vn3 := v3.Normalized()

	vn2 := v2.Normalized()
	vn4 := v4.Normalized()

	f1 := vec2d.Dot(&vn1, &vn3)
	f2 := vec2d.Dot(&vn2, &vn4)
	return (math.Abs(f1) < 1e-2) || (math.Abs(f2) < 1e-2)
}
