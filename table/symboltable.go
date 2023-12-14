package table

import (
	"github.com/flywave/go-dxf/format"
	"github.com/flywave/go-dxf/handle"
)

// SymbolTable is interface for AcDbSymbolTableRecord.
type SymbolTable interface {
	IsSymbolTable() bool
	Format(format.Formatter)
	Handle() int
	SetHandle(*int)
	SetOwner(handle.Handler)
	Name() string
}
