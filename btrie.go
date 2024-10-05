// Package btrie defines interfaces for and provides multiple implementations of a binary trie.
package btrie

// BTrie is ....
type BTrie[V any] interface {
	DeprPut(key []byte, value V) (previous V)
	DeprGet(key []byte) V
	DeprDelete(key []byte) (previous V)
	DeprRange(begin, end []byte) Cursor[V]
}

// Cursor is the type returned by [BTrie.Range].
type Cursor[V any] interface {
	HasNext() bool
	Next() (key []byte, value V)
}

type Entry[V any] struct {
	Key   []byte
	Value V
}
