package kv

import (
	"bytes"
	"fmt"
	"math"
)

// Bounds is the argument type for [Store.Range].
// A nil value for [Bounds.Begin] or [Bounds.End] represents +/-Inf;
// which one depends on the value of [Bounds.IsReverse].
// Note that an empty value is not nil; -Inf < []byte{} < []byte{0}.
// For non-nil values, Begin is inclusive and End is exclusive regardless of the direction.
// [Bounds.Begin] and [Bounds.End] return references to internal slices.
// Ways to construct a Bounds instance:
//
//	From(begin).To(end)      // IsReverse is false
//	From(begin).DownTo(end)  // IsReverse is true
type Bounds struct {
	// Begin is the [From] argument used to construct this Bounds.
	Begin []byte

	// End is the [Bounds.To] or [Bounds.DownTo] argument used to construct this Bounds.
	End []byte

	// IsReverse is false if this Bounds was created by [Bounds.To],
	// and true if it was created by [Bounds.DownTo].
	IsReverse bool
}

// Clone returns a deep clone of this Bounds.
func (b *Bounds) Clone() *Bounds {
	return &Bounds{bytes.Clone(b.Begin), bytes.Clone(b.End), b.IsReverse}
}

func (b *Bounds) String() string {
	if b.IsReverse {
		return fmt.Sprintf("[%s down to %s]", KeyName(b.Begin), KeyName(b.End))
	}
	return fmt.Sprintf("[%s to %s]", KeyName(b.Begin), KeyName(b.End))
}

// From returns a Bounds with the given Begin, nil End, and IsReverse false.
// This is normally used with [To] or [DownTo].
func From(begin []byte) *Bounds {
	return &Bounds{begin, nil, false}
}

// To returns a new Bounds from b.Begin (inclusve) to end (exclusive), with IsReverse false.
// To will panic if begin >= end.
func (b *Bounds) To(end []byte) *Bounds {
	if b.Begin != nil && end != nil && bytes.Compare(b.Begin, end) >= 0 {
		panic("bounds From >= To")
	}
	return &Bounds{b.Begin, end, false}
}

// DownTo returns a new Bounds from b.Begin (inclusve) down to end (exclusive), with IsReverse true.
// DownTo will panic if begin <= end.
func (b *Bounds) DownTo(end []byte) *Bounds {
	if b.Begin != nil && end != nil && bytes.Compare(b.Begin, end) <= 0 {
		panic("bounds From <= DownTo")
	}
	return &Bounds{b.Begin, end, true}
}

// CompareKey returns 0 if key is within this Bounds, -1 if beyond Begin, and +1 if beyond End.
// CompareKey will panic if key is nil.
// -Inf < {} < {0}.
func (b *Bounds) CompareKey(key []byte) int {
	if key == nil {
		panic("key must be non-nil")
	}
	if b.IsReverse {
		if b.Begin != nil && bytes.Compare(key, b.Begin) > 0 {
			return -1
		}
		if b.End != nil && bytes.Compare(b.End, key) >= 0 {
			return +1
		}
		// end < key <= begin
		return 0
	}
	if b.Begin != nil && bytes.Compare(key, b.Begin) < 0 {
		return -1
	}
	if b.End != nil && bytes.Compare(b.End, key) <= 0 {
		return +1
	}
	// begin <= key < end
	return 0
}

// childBounds returns the start and stop key bytes, inclusive,
// for the children of partialKey that a traversal should recurse into.
// If IsReverse is false or true, returns start <= stop or start >= stop respectively.
// If ok is false, no children should be recursed into.
//
// For example, with partialKey {5, 8} and bounds [{5, 8, 4, 13} to {5, 8, 7}], return (4, 6, true).
//
//nolint:nonamedreturns
func (b *Bounds) childBounds(partialKey []byte) (start, stop byte, ok bool) {
	if b.IsReverse {
		low, high, ok := childBounds(b.End, b.Begin, partialKey)
		return high, low, ok
	}
	return childBounds(b.Begin, b.End, partialKey)
}

//nolint:nestif,nonamedreturns
func childBounds(low, high, partialKey []byte) (start, stop byte, ok bool) {
	// For each of start and stop, one of 3 things could happen:
	// - return early with (0, 0, false) because all children of partialKey are out of bounds
	// - else, set the value to low/high[len(partialKey)] if partialKey is a prefix of low/high
	// - else, the value remains the same, 0 or 0xFF
	start, stop = 0, math.MaxUint8

	keyLen := len(partialKey)
	if low != nil {
		diffLen := len(low) - keyLen
		lowPrefix := low
		if diffLen > 0 {
			lowPrefix = low[:keyLen]
		}
		cmp := bytes.Compare(partialKey, lowPrefix)
		if cmp == -1 {
			return 0, 0, false
		}
		if cmp == 0 && diffLen > 0 {
			start = low[keyLen]
		}
	}
	if high != nil {
		diffLen := len(high) - keyLen
		highPrefix := high
		if diffLen > 0 {
			highPrefix = high[:keyLen]
		}
		cmp := bytes.Compare(partialKey, highPrefix)
		if cmp == +1 {
			return 0, 0, false
		}
		if cmp == 0 {
			if diffLen == 0 {
				return 0, 0, false
			}
			stop = high[keyLen]
		}
	}
	return start, stop, true
}
