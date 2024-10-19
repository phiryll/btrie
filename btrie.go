// Package btrie defines interfaces for and provides multiple implementations of a binary trie.
package btrie

import (
	"fmt"
	"iter"
)

// OrderedBytesMap is essentially an ordered map[[]byte]V.
// Keys must be non-nil.
// Implementations must clearly document any additional constraints on keys and values.
// Implementations must clearly document if any methods accept or return references to its internal storage.
// Implementations must clearly document if the iterator returned by Range is single-use.
// All OrderedBytesMap implementations in this package are tries.
type OrderedBytesMap[V any] interface {
	// Put sets the value for key, returning the previous value and whether or not the previous value existed.
	// Put will panic if this OrderedBytesMap does not support mutation.
	Put(key []byte, value V) (previous V, ok bool)

	// Get returns the value for key and whether or not it exists.
	Get(key []byte) (value V, ok bool)

	// Delete removes the value for key, returning the previous value and whether or not the previous value existed.
	// Delete will panic if this OrderedBytesMap does not support mutation.
	Delete(key []byte) (previous V, ok bool)

	// Range returns a sequence of key/value pairs over the given bounds.
	// Implementations should make a defensive copy of bounds using [Bounds.Clone] if necessary.
	// Most OrderedBytesMap implementations should not be mutated while a Range iteration is in progress.
	// Implementations should document if they can be safely mutated during iteration.
	Range(bounds Bounds) iter.Seq2[[]byte, V]
}

func emptySeq[T any](_ func(T) bool) {}

func keyName(key []byte) string {
	if key == nil {
		return "nil"
	}
	if len(key) == 0 {
		return "empty"
	}
	return fmt.Sprintf("%X", key)
}
