// Package btrie defines interfaces for and provides multiple implementations of a binary trie.
package btrie

import (
	"fmt"
	"io"
	"iter"
	"strings"
)

// BTrie is essentially an ordered map[[]byte]V.
// Keys must be non-nil.
// Implementations must clearly document any additional constraints on keys and values.
// Implementations must clearly document if any methods accept or return references to its internal storage.
// Implementations must clearly document if the iterator returned by Range is single-use.
// Although nothing in this interface mandates it, all BTrie implementations in this package are tries.
// If an implemention implements [fmt.Stringer], it should produce a value appropriate for debugging,
// how verbose is up to the implementer.
type BTrie[V any] interface {
	// Get returns the value for key and whether or not it exists.
	Get(key []byte) (value V, ok bool)

	// Put sets the value for key, returning the previous value and whether or not the previous value existed.
	// Put will panic if this BTrie does not support mutation.
	Put(key []byte, value V) (previous V, ok bool)

	// Delete removes the value for key, returning the previous value and whether or not the previous value existed.
	// Delete will panic if this BTrie does not support mutation.
	Delete(key []byte) (previous V, ok bool)

	// Range returns a sequence of key/value pairs over the given bounds.
	// Implementations should make a defensive copy of bounds using [Bounds.Clone] if necessary.
	// Most BTrie implementations should not be mutated while a Range iteration is in progress.
	// Implementations should document if they can be safely mutated during iteration.
	Range(bounds *Bounds) iter.Seq2[[]byte, V]
}

// KeyName returns a user-friendly string for key,
// one of "nil", "empty", or the key bytes as a hex string.
func KeyName(key []byte) string {
	if key == nil {
		return "nil"
	}
	if len(key) == 0 {
		return "empty"
	}
	return fmt.Sprintf("%X", key)
}

func emptySeq[V any](_ func(V) bool) {}

// Sprint returns a pretty-printed representation of a key/value sequence.
// Note that the value is only "pretty" if the sequence is ordered by key.
// Values are printed using the `%v` format specifier.
// Returns an empty string if the sequence is empty.
func Sprint[V any](seq iter.Seq2[[]byte, V]) string {
	var s strings.Builder
	if _, err := Fprint(&s, seq); err != nil {
		panic(err)
	}
	return s.String()
}

func indent(n int) string {
	const (
		// must be 2 characters to align with hex-formatted bytes
		indent         = ". "
		repeatedIndent = ". . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . "
	)
	if 2*n > len(repeatedIndent) {
		return strings.Repeat(indent, n)
	}
	return repeatedIndent[:2*n]
}

// Fprint writes a pretty-printed representation of a key/value sequence to w.
// Note that the value is only "pretty" if the sequence is ordered by key.
// Values are printed using the `%v` format specifier.
// Writes nothing to w if the sequence is empty.
func Fprint[V any](w io.Writer, seq iter.Seq2[[]byte, V]) (int, error) {
	n := 0
	prevKey := []byte{}
	for key, value := range seq {
		limit := min(len(key), len(prevKey))
		i := 0
		for i < limit && key[i] == prevKey[i] {
			i++
		}
		k, err := fmt.Fprintf(w, "%s%X: %v\n", indent(i), key[i:], value)
		n += k
		if err != nil {
			//nolint:wrapcheck
			return n, err
		}
		prevKey = key
	}
	return n, nil
}

// Optional is a helper type for implementations, at the expense of some efficiency if methods are not inlined.
// This is not intended to be a general Optional implementation.
// The zero value `Optional[someType]{}` will create a new empty instance.
type Optional[T any] struct {
	value T
	ok    bool
}

// OptionalOf returns a new Optional containing value.
func OptionalOf[T any](value T) Optional[T] {
	return Optional[T]{value, true}
}

// IsEmpty returns whether o contains a valid value.
func (o *Optional[T]) IsEmpty() bool {
	return !o.ok
}

// Get returns the value contained by o and whether or not it exists.
func (o *Optional[T]) Get() (T, bool) {
	return o.value, o.ok
}

// Set sets the value contained by o, returning the previous value and whether or not it existed.
func (o *Optional[T]) Set(value T) (T, bool) {
	prev, ok := o.value, o.ok
	o.value, o.ok = value, true
	return prev, ok
}

// Clear removes the value contained by o, returning the previous value and whether or not it existed.
func (o *Optional[T]) Clear() (T, bool) {
	prev, ok := o.value, o.ok
	var zero T
	o.value, o.ok = zero, false
	return prev, ok
}
