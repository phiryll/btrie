package btrie

import (
	"bytes"
	"fmt"
	"math"
)

// The original Bounds was a struct with exported members and a Reverse field.
// This ends up being simpler internally because the forward/reverse differences are significant.

// Bounds is the argument type for [OrderedBytesMap.Range].
// A nil value for [Bounds.Begin] or [Bounds.End] represents +/-Inf;
// which one depends on the value of [Bounds.IsReverse].
// Note that an empty begin/end value is not nil; -Inf < []byte{} < []byte{0}.
// For non-nil values, Begin is inclusive and End is exclusive regardless of the direction.
// [Bounds.Begin] and [Bounds.End] return references to internal slices.
// Ways to construct a Bounds instance:
//
//	From(begin).To(end)      // IsReverse() is false
//	From(begin).DownTo(end)  // IsReverse() is true
type Bounds interface {
	Begin() []byte
	End() []byte

	// IsReverse returns false if this Bounds was created by [beginKey.To],
	// and true if it was created by [beginKey.DownTo].
	IsReverse() bool

	// Compare returns 0 if key is within this Bounds, -1 if beyond Begin, and +1 if beyond End.
	// Compare will panic if key is nil.
	// -Inf < {} < {0}.
	Compare(key []byte) int

	// Clone returns a deep clone of this Bounds.
	Clone() Bounds

	String() string

	// childBounds returns the start and stop key bytes, inclusive,
	// for the children of partialKey that a traversal should recurse into.
	// If IsReverse is false or true, returns start <= stop or start >= stop respectively.
	// If ok is false, no children should be recursed into.
	//
	// For example, with partialKey {5, 8} and bounds [{5, 8, 4, 13} to {5, 8, 7}], return (4, 6, true).
	childBounds(partialKey []byte) (start, stop byte, ok bool)
}

// A BoundsBuilder is returned by [From].
type BoundsBuilder interface {
	To(end []byte) Bounds
	DownTo(end []byte) Bounds
}

type (
	baseBounds struct {
		begin, end []byte
	}
	forward baseBounds
	reverse baseBounds
)

func (b forward) Begin() []byte  { return b.begin }
func (b forward) End() []byte    { return b.end }
func (forward) IsReverse() bool  { return false }
func (b forward) Clone() Bounds  { return forward{bytes.Clone(b.begin), bytes.Clone(b.end)} }
func (b forward) String() string { return fmt.Sprintf("[%X to %X]", b.begin, b.end) }

func (b reverse) Begin() []byte  { return b.begin }
func (b reverse) End() []byte    { return b.end }
func (reverse) IsReverse() bool  { return true }
func (b reverse) Clone() Bounds  { return reverse{bytes.Clone(b.begin), bytes.Clone(b.end)} }
func (b reverse) String() string { return fmt.Sprintf("[%X down to %X]", b.begin, b.end) }

type beginKey []byte

// From returns a builder with To and DownTo methods for constructing a Bounds.
func From(begin []byte) BoundsBuilder {
	return beginKey(begin)
}

// To returns a new Bounds from begin (inclusve) to key (exclusive).
// To will panic if begin >= key.
func (begin beginKey) To(end []byte) Bounds {
	if begin != nil && end != nil && bytes.Compare(begin, end) >= 0 {
		panic("bounds From >= To")
	}
	return forward{begin, end}
}

// DownTo returns a new Bounds from begin (inclusve) down to key (exclusive).
// DownTo will panic if begin <= key.
func (begin beginKey) DownTo(end []byte) Bounds {
	if begin != nil && end != nil && bytes.Compare(begin, end) <= 0 {
		panic("bounds From <= DownTo")
	}
	return reverse{begin, end}
}

func (b forward) Compare(key []byte) int {
	if key == nil {
		panic("key cannot be nil")
	}
	if b.begin != nil && bytes.Compare(key, b.begin) < 0 {
		return -1
	}
	if b.end != nil && bytes.Compare(b.end, key) <= 0 {
		return +1
	}
	// begin <= key < end
	return 0
}

func (b reverse) Compare(key []byte) int {
	if key == nil {
		panic("key cannot be nil")
	}
	if b.begin != nil && bytes.Compare(key, b.begin) > 0 {
		return -1
	}
	if b.end != nil && bytes.Compare(b.end, key) >= 0 {
		return +1
	}
	// end < key <= begin
	return 0
}

//nolint:nonamedreturns
func (b forward) childBounds(partialKey []byte) (start, stop byte, ok bool) {
	start, ok = lower(b.begin, partialKey)
	if !ok {
		return 0, 0, false
	}
	stop, ok = upper(b.end, partialKey)
	if !ok {
		return 0, 0, false
	}
	// This optimization would be invalid in reverse.childBounds.
	if len(partialKey)+1 == len(b.end) && bytes.HasPrefix(b.end, partialKey) {
		if stop == 0 {
			return 0, 0, false
		}
		stop--
	}
	return start, stop, true
}

//nolint:nonamedreturns
func (b reverse) childBounds(partialKey []byte) (start, stop byte, ok bool) {
	start, ok = upper(b.begin, partialKey)
	if !ok {
		return 0, 0, false
	}
	stop, ok = lower(b.end, partialKey)
	if !ok {
		return 0, 0, false
	}
	return start, stop, true
}

// Compares partialKey to bound[:len(partialKey)], or all of bound if partialKey is longer.
func comparePrefix(partialKey, bound []byte) int {
	if len(bound) > len(partialKey) {
		return bytes.Compare(partialKey, bound[:len(partialKey)])
	}
	return bytes.Compare(partialKey, bound)
}

// Return the lower key byte (start for forward, and stop for reverse).
func lower(bound, partialKey []byte) (byte, bool) {
	if bound == nil {
		return 0, true
	}
	switch comparePrefix(partialKey, bound) {
	case -1:
		return 0, false
	case +1:
		return 0, true
	default:
		if len(bound) == len(partialKey) {
			return 0, true
		}
		return bound[len(partialKey)], true
	}
}

// Return the upper key byte (stop for forward, and start for reverse).
func upper(bound, partialKey []byte) (byte, bool) {
	if bound == nil {
		return math.MaxUint8, true
	}
	switch comparePrefix(partialKey, bound) {
	case -1:
		return math.MaxUint8, true
	case +1:
		return 0, false
	default:
		if len(bound) == len(partialKey) {
			return 0, false
		}
		return bound[len(partialKey)], true
	}
}
