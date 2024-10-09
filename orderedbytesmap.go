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
	// Put sets the value for key, returning the previous value and whether or not the previous value did exist.
	// Put will panic if this OrderedBytesMap does not support mutation.
	Put(key []byte, value V) (previous V, ok bool)

	// Get returns the value for key and whether or not it exists.
	Get(key []byte) (value V, ok bool)

	// Delete removes the value for key, returning the previous value and whether or not the previous value did exist.
	// Delete will panic if this OrderedBytesMap does not support mutation.
	Delete(key []byte) (previous V, ok bool)

	// Range returns a sequence of key/value pairs over the given bounds.
	// Implementations should consider making a defensive copy of bounds using [Bounds.Clone].
	Range(bounds *Bounds) iter.Seq2[[]byte, V]
}

// Bounds is the argument type for [OrderedBytesMap.Range].
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

// Compare returns where key is in relation to b, taking b.Reverse into account.
// If b.Reverse is false, the result will be -1 if key < b.Begin, 0 if begin <= key < end, and +1 if end <= key.
// If b.Reverse is true, the result will be -1 if key > b.Begin, 0 if begin >= key > end, and +1 if end >= key.
// In other words, 0 if within the bounds, -1 if beyond b.Begin, and +1 if beyond b.End.
func (b *Bounds) Compare(key []byte) int {
	if b.Reverse {
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

// TODO: After the initial implementations, define a common interface (maybe the same?).
// Use this to allow different representations at different locations in the trie.
// The goal would be to locally self-optimize the trie for speed or space.
// I'm specifically considering sparse vs. dense data, but there may be other possibilities.
// This is probably not feasible for the fully or partially persistent variants
// since those need to maintain all history.

// Note: iter.Pull iteraters need to clean up before returning
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
