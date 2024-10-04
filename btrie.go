// Package btrie defines interfaces for and provides multiple implementations of a binary trie.
package btrie

// BTrie is ....
type BTrie interface {
	Put(key, value []byte) (previous []byte)
	Get(key []byte) []byte
	Delete(key []byte) (previous []byte)
	Range(begin, end []byte) Cursor
}

// Cursor is the type returned by [BTrie.Range].
type Cursor interface {
	HasNext() bool
	Next() (key, value []byte)
}
