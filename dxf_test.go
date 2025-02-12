package dxf_test

import (
	"bytes"
	"crypto/sha1"
	"fmt"
	"io"
	"math"
	"os"
	"path/filepath"
	"testing"

	"github.com/flywave/go-dxf"
	"github.com/flywave/go-dxf/color"
	geom "github.com/flywave/go-dxf/convert_geom"
	"github.com/flywave/go-dxf/insunit"
	"github.com/flywave/go-dxf/table"
	"github.com/flywave/go-geom/general"

	"github.com/flywave/go-dxf/drawing"
	"github.com/flywave/go-dxf/entity"
	vec2d "github.com/flywave/go3d/float64/vec2"
)

// TOLERANCE is the epsilon value used in comparing floats.
const TOLERANCE = 0.000001

// cmpF64 compares two floats to see if they are within the given tolerance.
func cmpF64(f1, f2 float64) bool {
	if math.IsInf(f1, 1) {
		return math.IsInf(f2, 1)
	}
	if math.IsInf(f2, 1) {
		return math.IsInf(f1, 1)
	}
	if math.IsInf(f1, -1) {
		return math.IsInf(f2, -1)
	}
	if math.IsInf(f2, -1) {
		return math.IsInf(f1, -1)
	}
	return math.Abs(f1-f2) < TOLERANCE
}

func checkEntities(t *testing.T, expected, got entity.Entities) bool {
	if len(expected) != len(got) {
		t.Errorf("number of entities, expected %v got %v", len(expected), len(got))
		return false
	}
	for i, ee := range expected {
		switch et := ee.(type) {
		case *entity.Point:
			p, ok := got[i].(*entity.Point)
			if !ok {
				t.Errorf("type for %v, expected %T got %T", i, et, got[i])
				return false
			}
			if len(et.Coord) != len(p.Coord) {
				t.Errorf("point %v : coord len, expected %v got %v", i, len(et.Coord), len(p.Coord))
				return false
			}
			for j, crd := range et.Coord {
				if !cmpF64(crd, p.Coord[j]) {
					t.Errorf("point %v : coord %v, expected %v got %v", i, j, crd, p.Coord[j])
					t.Logf("ePoint %v, Point %v", et.Coord, p.Coord)
					return false
				}
			}
		}
	}
	return true
}

func hashBytes(b []byte) string {
	return fmt.Sprintf("%x", sha1.Sum(b))
}
func hashFile(filename string) (string, error) {
	tfile := filepath.Join("testdata", filename)
	f, err := os.Open(tfile)
	if err != nil {
		return "", err
	}
	text, err := io.ReadAll(f)
	f.Close()
	if err != nil {
		return "", err
	}
	return hashBytes(text), nil
}

func TestFromStringData(t *testing.T) {
	type tcase struct {
		filename         string
		ExpectedEntities entity.Entities
		fn               func(*testing.T, *drawing.Drawing, tcase)
	}
	fn := func(tc tcase) (string, func(*testing.T)) {
		return tc.filename, func(t *testing.T) {
			tfile := filepath.Join("testdata", tc.filename)

			data, err := os.ReadFile(tfile)
			if err != nil {
				t.Errorf("file, could not open file %v : %v", tfile, err)
			}
			d, err := dxf.FromStringData(string(data))
			if err != nil {
				t.Errorf("error, expected nil, got %v", err)
				return
			}

			if tc.ExpectedEntities == nil {
				// Let's go ahead and log out the entities in the file.
				// This is useful for build out test cases.
				t.Logf("Number of entities: %d", d.Entities())
				for i, e := range d.Entities() {
					t.Logf("\t%3d:[%[2]T]%[2]v", i, e)
				}
			} else if !checkEntities(t, tc.ExpectedEntities, d.Entities()) {
				return
			}

			if tc.fn != nil {
				tc.fn(t, d, tc)
			}
		}
	}

	tests := []tcase{
		{
			filename: "point.dxf",
			ExpectedEntities: entity.Entities{
				entity.NewPoint(),
				entity.NewPoint(100.0, 100.0, 0.0),
				entity.NewPoint(200.0, 100.0, 0.0),
			},
		},
		{
			filename: "arc.dxf",
			ExpectedEntities: entity.Entities{
				func() entity.Entity {
					cr := entity.NewCircle()
					cr.Direction = []float64{0.0, 0.0, 1.0}
					cr.Radius = 100
					arc := entity.NewArc(cr)
					arc.Angle = []float64{0.0, 60.0}
					return arc
				}(),
			},
		},
	}
	for _, tc := range tests {
		t.Run(fn(tc))
	}
}

func TestFromFile(t *testing.T) {
	type tcase struct {
		filename         string
		ExpectedEntities entity.Entities
		fn               func(*testing.T, *drawing.Drawing, tcase)
	}
	fn := func(tc tcase) (string, func(*testing.T)) {
		return tc.filename, func(t *testing.T) {
			tfile := filepath.Join("testdata", tc.filename)
			d, err := dxf.FromFile(tfile)
			if err != nil {
				t.Errorf("error, expected nil, got %v", err)
				return
			}

			if len(tc.ExpectedEntities) != 0 && !checkEntities(t, tc.ExpectedEntities, d.Entities()) {
				return
			}

			if tc.fn != nil {
				tc.fn(t, d, tc)
			}
		}
	}

	tests := []tcase{
		{
			filename: "point.dxf",
			ExpectedEntities: entity.Entities{
				entity.NewPoint(),
				entity.NewPoint(100.0, 100.0, 0.0),
				entity.NewPoint(200.0, 100.0, 0.0),
			},
		},
		{
			filename: "mypoint.dxf",
			ExpectedEntities: entity.Entities{
				entity.NewPoint(),
				entity.NewPoint(100.0, 100.0, 0.0),
				entity.NewPoint(100.0, 200.0, 0.0),
			},
		},
		{
			filename: "mypoint_with_extent.dxf",
			ExpectedEntities: entity.Entities{
				entity.NewPoint(),
				entity.NewPoint(100.0, 100.0, 0.0),
				entity.NewPoint(100.0, 200.0, 0.0),
			},
		},
		{
			filename: "arc.dxf",
			ExpectedEntities: entity.Entities{
				func() entity.Entity {
					cr := entity.NewCircle()
					cr.Direction = []float64{0.0, 0.0, 1.0}
					cr.Radius = 100
					arc := entity.NewArc(cr)
					arc.Angle = []float64{0.0, 60.0}
					return arc
				}(),
			},
		},
	}
	for _, tc := range tests {
		t.Run(fn(tc))
	}
}

func TestNewDrawing(t *testing.T) {
	type tcase struct {
		filename string
		draw     func(d *drawing.Drawing)
	}
	fn := func(tc tcase) (string, func(*testing.T)) {
		return tc.filename, func(t *testing.T) {
			// calculate the sha256
			fileHash, err := hashFile(tc.filename)
			if err != nil {
				t.Errorf("hash of file(%v) error, expected nil got %v", tc.filename, err)
				return
			}
			d := drawing.New()
			tc.draw(d)
			var buff bytes.Buffer
			_, err = io.Copy(&buff, d)
			if err != nil {
				t.Errorf("copy of buffer, expected nil got %v", err)
			}
			buffHash := hashBytes(buff.Bytes())
			if fileHash != buffHash {
				t.Errorf("hash, expected %v got %v", fileHash, buffHash)
				outputfn := filepath.Join("testdata", buffHash+"_"+tc.filename)
				of, err := os.Create(outputfn)
				if err != nil {
					t.Logf("could not create debug file: %v", outputfn)
					return
				}
				io.Copy(of, &buff)
				of.Close()
				t.Logf("wrote out file to: %v", outputfn)
			}
		}
	}
	tests := []tcase{
		{
			filename: "mypoint.dxf",
			draw: func(d *drawing.Drawing) {
				d.Point(0.0, 0.0, 0.0)
				d.Point(100.0, 100.0, 0.0)
				d.Point(100.0, 200.0, 0.0)
			},
		},
		{
			filename: "mypoint_with_extent.dxf",
			draw: func(d *drawing.Drawing) {
				d.Point(0.0, 0.0, 0.0)
				d.Point(100.0, 100.0, 0.0)
				d.Point(100.0, 200.0, 0.0)
				d.SetExt()
			},
		},
		{
			filename: "mypoint_with_units.dxf",
			draw: func(d *drawing.Drawing) {
				d.Point(0.0, 0.0, 0.0)
				d.Point(100.0, 100.0, 0.0)
				d.Point(100.0, 200.0, 0.0)
				d.Header().LtScale = 1
				d.Header().InsUnit = insunit.Inches
				d.Header().InsLUnit = insunit.Architectural
				d.SetExt()
			},
		},
		{
			filename: "torus.dxf",
			draw: func(d *drawing.Drawing) {
				d.Header().LtScale = 100.0
				d.AddLayer("Toroidal", dxf.DefaultColor, dxf.DefaultLineType, true)
				d.AddLayer("Poloidal", color.Red, table.LT_HIDDEN, true)
				z := 0.0
				r1 := 200.0
				r2 := 500.0
				ndiv := 16
				dtheta := 2.0 * math.Pi / float64(ndiv)
				theta := 0.0
				for i := 0; i < ndiv; i++ {
					d.ChangeLayer("Toroidal")
					d.Circle(0.0, 0.0, z+r1*math.Cos(theta), r2-r1*math.Sin(theta))
					d.ChangeLayer("Poloidal")
					c, _ := d.Circle(r2*math.Cos(theta), r2*math.Sin(theta), 0.0, r1)
					dxf.SetExtrusion(c, []float64{-1.0 * math.Sin(theta), math.Cos(theta), 0.0})
					theta += dtheta
				}
			},
		},
		{
			filename: "my_arc.dxf",
			draw: func(d *drawing.Drawing) {
				// x , y, z, radius, start, end
				d.Arc(0.0, 0.0, 0.0, 100.0, 0.0, 60.0)
			},
		},
	}
	for _, tc := range tests {
		t.Run(fn(tc))
	}

}

func TestDistance(t *testing.T) {
	p1 := &vec2d.T{0, 5}

	p2 := &vec2d.T{0, 0}
	p3 := &vec2d.T{5, 5}

	dis := vec2d.PointSegmentDistance(p1, p2, p3)
	fmt.Println(dis)
	d2 := math.Pow(dis, 2)
	fmt.Println(math.Sqrt(d2 * 2))
}
func TestDwg(t *testing.T) {
	d, err := geom.ConvertToGeomFeatures("testdata/修改余吾煤业采掘工程平面图2023.9.26.dxf", "")
	if err != nil {
		t.Errorf("error, expected nil, got %v", err)
		return
	}

	for k, v := range d {
		bt, _ := v.MarshalJSON()
		os.WriteFile("testdata/余吾/"+k+".json", bt, os.ModePerm)
	}
}

func TestDwg2(t *testing.T) {
	d, err := geom.ConvertToGeomFeatures("testdata/11-3布尔台煤矿42煤采掘工程平面图.dxf", "")
	if err != nil {
		t.Errorf("error, expected nil, got %v", err)
		return
	}
	col := d["22煤巷道"]
	bt, _ := col.MarshalJSON()
	os.WriteFile("testdata/22煤巷道.json", bt, os.ModePerm)
}

func TestMeger(t *testing.T) {
	d, err := os.ReadFile("testdata/已完成巷道.json")
	if err != nil {
		t.Errorf("error, expected nil, got %v", err)
		return
	}

	col, _ := general.UnmarshalFeatureCollection(d)
	col2 := geom.GenCenterLine(col, &geom.CenterLineOpts{SearchExtend: 10, MaxWidth: 10, MinWidth: 1.5, LineMinlength: 5})
	geom.MegerCenterLine(col2, &geom.MegerOpts{SearchRadius: 10, Distance: 1})
	bt, _ := col2.MarshalJSON()
	os.WriteFile("testdata/meger.json", bt, os.ModePerm)
}

func TestMeger2(t *testing.T) {
	d, err := os.ReadFile("testdata/part.geojson")
	if err != nil {
		t.Errorf("error, expected nil, got %v", err)
		return
	}

	col2, _ := general.UnmarshalFeatureCollection(d)
	// col2 := geom.GenCenterLine(col, &geom.CenterLineOpts{SearchExtend: 10, MaxWidth: 10, MinWidth: 1.5, LineMinlength: 10})
	geom.MegerCenterLine(col2, &geom.MegerOpts{SearchRadius: 10, Distance: 1})
	bt, _ := col2.MarshalJSON()
	os.WriteFile("testdata/meger2.json", bt, os.ModePerm)
}

func TestMeger3(t *testing.T) {
	d, err := os.ReadFile("testdata/meger.json")
	if err != nil {
		t.Errorf("error, expected nil, got %v", err)
		return
	}

	col2, _ := general.UnmarshalFeatureCollection(d)
	geom.MegerCenterLine(col2, &geom.MegerOpts{SearchRadius: 10, Distance: 2})
	bt, _ := col2.MarshalJSON()
	os.WriteFile("testdata/meger3.json", bt, os.ModePerm)
}

func TestMeger4(t *testing.T) {
	d, err := os.ReadFile("testdata/part.geojson")
	if err != nil {
		t.Errorf("error, expected nil, got %v", err)
		return
	}

	col2, _ := general.UnmarshalFeatureCollection(d)
	geom.MegerCenterLine(col2, &geom.MegerOpts{SearchRadius: 10, Distance: 2})
	bt, _ := col2.MarshalJSON()
	os.WriteFile("testdata/meger4.json", bt, os.ModePerm)
}

func TestMegerClosed(t *testing.T) {
	opts := &geom.MegerOpts{}
	opts.SearchRadius = 10
	opts.Distance = 1
	d, err := os.ReadFile("testdata/part.geojson")
	if err != nil {
		t.Errorf("error, expected nil, got %v", err)
		return
	}

	col2, _ := general.UnmarshalFeatureCollection(d)
	geom.MegerCenterLine(col2, opts)
	bt, _ := col2.MarshalJSON()
	os.WriteFile("testdata/meger5.json", bt, os.ModePerm)
}
