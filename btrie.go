// Package btrie defines interfaces for and provides multiple implementations of a binary trie.
package btrie

import "iter"

// OrderedBytesMap is essentially an ordered map[[]byte]V.
// Keys must be non-nil.
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
