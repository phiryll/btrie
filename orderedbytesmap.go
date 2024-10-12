package btrie

import (
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
	Range(bounds Bounds) iter.Seq2[[]byte, V]
}

func emptySeq[T any](_ func(T) bool) {}

// TODO: After the initial implementations, define a common interface (maybe the same?).
// Use this to allow different representations at different locations in the trie.
// The goal would be to locally self-optimize the trie for speed or space.
// I'm specifically considering sparse vs. dense data, but there may be other possibilities.
// This is probably not feasible for the fully or partially persistent variants
// since those need to maintain all history.

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
