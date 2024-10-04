package btrie

import (
	"bytes"
	"fmt"
	"iter"
)

// OrderedBytesMap is essentially an ordered map[[]byte]V.
// Keys must be non-nil.
// A nil key returned by any method generally means "does not exist".
// Implementations must clearly document any additional constraints on keys and values.
// Implementations must clearly document if any methods accept or return references to its internal storage.
// All OrderedBytesMap implementations in this package are tries.
type OrderedBytesMap[V any] interface {
	// Put sets the value for key, returning the previous value.
	// Put will panic if this OrderedBytesMap does not support mutation.
	Put(key []byte, value V) (previous V, ok bool)

	// Get returns the value for key.
	Get(key []byte) (value V, ok bool)

	// Delete removes the value for key, returning the previous value.
	// Delete will panic if this OrderedBytesMap does not support mutation.
	Delete(key []byte) (previous V, ok bool)

	// Range does stuff.
	// Implementations should consider making a defensive copy of bounds using [Bounds.Clone].
	Range(bounds *Bounds) iter.Seq2[[]byte, V]

	// Cursor does stuff.
	// Implementations should consider making a defensive copy of bounds using [Bounds.Clone].
	// Cursor will panic if this OrderedBytesMap does not support mutation.
	Cursor(bounds *Bounds) iter.Seq[Pos[V]]
}

// Bounds is the argument type for [OrderedBytesMap.Range] and [OrderedBytesMap.Cursor].
// A nil value for [Bounds.Begin] or [Bounds.End] represents +/-Inf;
// which one depends on the value of [Bounds.Reverse].
// Note that an empty bounds value is not nil; -Inf < []byte{} < []byte{0}.
// For non-nil values, Begin is inclusive and End is exclusive regardless of the direction.
// While a Bounds instance can be constructed directly, using [From] with [Bounds.To] or [Bounds.DownTo] is clearer.
type Bounds struct {
	Begin, End []byte
	Reverse    bool
}

// From creates a new Bounds with bounds.Begin=key.
func From(key []byte) *Bounds {
	return &Bounds{key, nil, false}
}

// Clone returns a deep copy of b.
func (b *Bounds) Clone() *Bounds {
	return &Bounds{
		bytes.Clone(b.Begin),
		bytes.Clone(b.End),
		b.Reverse,
	}
}

func (b *Bounds) String() string {
	if b.Reverse {
		return fmt.Sprintf("[%X down to %X]", b.Begin, b.End)
	}
	return fmt.Sprintf("[%X to %X]", b.Begin, b.End)
}

// To sets b.End to key and b.Reverse to false, returning b.
// To will panic if b.Begin >= b.End.
func (b *Bounds) To(key []byte) *Bounds {
	if b.Begin != nil && key != nil && bytes.Compare(b.Begin, key) >= 0 {
		panic("bounds From >= To")
	}
	b.End = key
	b.Reverse = false
	return b
}

// DownTo sets b.End to key and b.Reverse to true, returning b.
// DownTo will panic if b.Begin <= b.End.
func (b *Bounds) DownTo(key []byte) *Bounds {
	if b.Begin != nil && key != nil && bytes.Compare(b.Begin, key) <= 0 {
		panic("bounds From <= DownTo")
	}
	b.End = key
	b.Reverse = true
	return b
}

// Contains returns whether key is within b.
func (b *Bounds) Contains(key []byte) bool {
	if b.Reverse {
		// end < key <= begin
		if b.End != nil && bytes.Compare(key, b.End) <= 0 {
			return false
		}
		if b.Begin != nil && bytes.Compare(key, b.Begin) > 0 {
			return false
		}
	} else {
		// begin <= key < end
		if b.Begin != nil && bytes.Compare(key, b.Begin) < 0 {
			return false
		}
		if b.End != nil && bytes.Compare(key, b.End) >= 0 {
			return false
		}
	}
	return true
}

// A Pos represents a mutable position in the sequence returned by [OrderedBytesMap.Cursor].
type Pos[V any] interface {
	// Key returns the key at this position.
	Key() []byte

	// Value returns the value at this position.
	Value() V

	// Set sets the value at this position.
	Set(value V)

	// Delete removes the key/value pair at this position.
	Delete()
}

// DefaultPos is a default Pos implementation which delegates to the containing OrderedBytesMap.
// Do not use this implementation if Set or Delete might corrupt the Cursor which created it.
func DefaultPos[V any](bMap OrderedBytesMap[V], key []byte) Pos[V] {
	return &position[V]{bMap, key}
}

type position[V any] struct {
	bMap OrderedBytesMap[V]
	key  []byte
}

func (p *position[V]) Key() []byte {
	return p.key
}

func (p *position[V]) Value() V {
	value, _ := p.bMap.Get(p.key)
	return value
}

func (p *position[V]) Set(value V) {
	p.bMap.Put(p.key, value)
}

func (p *position[V]) Delete() {
	p.bMap.Delete(p.key)
}

// EmptySeq is an empty [iter.Seq].
func EmptySeq[T any](_ func(T) bool) {}

// EmptySeq2 is an empty [iter.Seq2].
func EmptySeq2[K, V any](_ func(K, V) bool) {}

// TODO: After the initial implementations, define a common interface (maybe the same?).
// Use this to allow different representations at different locations in the trie.
// The goal would be to locally self-optimize the trie for speed or space.
// I'm specifically considering sparse vs. dense data, but there may be other possibilities.
// This is probably not feasible for the fully or partially persistent variants
// since those need to maintain all history.

// Note: iteraters need to clean up before returning
// BIG Note: Document iter.Seq/Seq2 if single use. Multiple use would look like:
//   it := bMap.Range(From(begin).To(end))
//   for k, v := range it {
//     do stuff
//   }
//   for k, v := range it {
//     do more stuff
//   }

// The reference collecting versions need to use runtime.SetFinalizer(). See
// https://stackoverflow.com/questions/63025066/is-it-safe-to-use-a-uintptr-as-a-weak-reference
