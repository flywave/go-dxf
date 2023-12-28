package geom

import (
	"strconv"
	"strings"

	dxf "github.com/flywave/go-dxf"
	"github.com/flywave/go-dxf/entity"
	geom "github.com/flywave/go-geom"
	"github.com/pborman/uuid"
)

func ConvertToGeomFeatures(inputFile string, ty string) (map[string]*geom.FeatureCollection, error) {
	draw, err := dxf.FromFile(inputFile)
	if err != nil {
		return nil, err
	}

	geomMap := make(map[string]*geom.FeatureCollection)
	ents := draw.Entities()

	fn := func(layerName string, f *geom.Feature) {
		if f == nil || (ty != "" && string(f.GeometryData.Type) != ty) {
			return
		}

		f.ID = NewUUid32()
		col, ok := geomMap[layerName]
		if !ok {
			col = geom.NewFeatureCollection()
			geomMap[layerName] = col
		}

		f.Properties["layer"] = layerName
		col.Features = append(col.Features, f)
	}
	for _, e := range ents {
		layerName := e.Layer().Name()
		var f *geom.Feature
		switch ety := e.(type) {
		case *entity.Line:
			l := [][]float64{ety.Start, ety.End}
			f = geom.NewLineStringFeature(l)
		case *entity.LwPolyline:
			for i := 0; i < len(ety.Vertices)-1; i++ {
				l := [][]float64{ety.Vertices[i], ety.Vertices[i+1]}
				f = geom.NewLineStringFeature(l)
				fn(layerName, f)
			}
			continue
		case *entity.Text:
			v := strings.ReplaceAll(ety.Value, " ", "")
			if val, err := strconv.ParseFloat(v, 64); err == nil {
				c := []float64{ety.Coord1[0], ety.Coord1[1]}
				f = geom.NewPointFeature(c)
				f.Properties["value"] = val
			}
		case *entity.ThreeDFace:
			f = geom.NewPolygonFeature([][][]float64{ety.Points})
		}
		fn(layerName, f)
	}

	return geomMap, nil
}

func NewUUid32() string {
	id := uuid.NewRandom().String()
	id = strings.ReplaceAll(id, "-", "")
	return id
}
