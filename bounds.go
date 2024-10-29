package btrie

import (
	"bytes"
	"fmt"
	"math"
)

// The original Bounds was a struct with exported members and a Reverse field.
// This ends up being simpler internally because the forward/reverse differences are significant.

// Bounds is the argument type for [BTrie.Range].
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
	// Begin returns the [From] argument used to construct this Bounds.
	Begin() []byte

	// End returns the [BoundsBuilder.To] or [BoundsBuilder.DownTo] argument used to construct this Bounds.
	End() []byte

	// IsReverse returns false if this Bounds was created by [BoundsBuilder.To],
	// and true if it was created by [BoundsBuilder.DownTo].
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

func (b forward) Begin() []byte { return b.begin }
func (b forward) End() []byte   { return b.end }
func (forward) IsReverse() bool { return false }
func (b forward) Clone() Bounds { return forward{bytes.Clone(b.begin), bytes.Clone(b.end)} }

func (b reverse) Begin() []byte { return b.begin }
func (b reverse) End() []byte   { return b.end }
func (reverse) IsReverse() bool { return true }
func (b reverse) Clone() Bounds { return reverse{bytes.Clone(b.begin), bytes.Clone(b.end)} }

func (b forward) String() string {
	return fmt.Sprintf("[%s to %s]", keyName(b.begin), keyName(b.end))
}

func (b reverse) String() string {
	return fmt.Sprintf("[%s down to %s]", keyName(b.begin), keyName(b.end))
}

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
	return childBounds(b.begin, b.end, partialKey)
}

//nolint:nonamedreturns
func (b reverse) childBounds(partialKey []byte) (start, stop byte, ok bool) {
	low, high, ok := childBounds(b.end, b.begin, partialKey)
	return high, low, ok
}

//nolint:nestif,nonamedreturns
func childBounds(low, high, partialKey []byte) (start, stop byte, ok bool) {
	// For each of start and stop, one of 3 things could happen:
	// - return early with (0, 0, false) because all children of partialKey are out of bounds
	// - else, set the value to low/high[len(partialKey)] if partialKey is a prefix of low/high
	// - else, the value remains the same, 0 or 0xFF
	start, stop = 0, math.MaxUint8

	keySize := len(partialKey)
	if low != nil {
		diffSize := len(low) - keySize
		lowPrefix := low
		if diffSize > 0 {
			lowPrefix = low[:keySize]
		}
		cmp := bytes.Compare(partialKey, lowPrefix)
		if cmp == -1 {
			return 0, 0, false
		}
		if cmp == 0 && diffSize > 0 {
			start = low[keySize]
		}
	}
	if high != nil {
		diffSize := len(high) - keySize
		highPrefix := high
		if diffSize > 0 {
			highPrefix = high[:keySize]
		}
		cmp := bytes.Compare(partialKey, highPrefix)
		if cmp == +1 {
			return 0, 0, false
		}
		if cmp == 0 {
			if diffSize == 0 {
				return 0, 0, false
			}
			stop = high[keySize]
		}
	}
	return start, stop, true
}
