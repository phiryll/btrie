// Package btrie defines interfaces for and provides multiple implementations of a binary trie.
package btrie

// BTrie is ....
type BTrie interface {
	DeprPut(key, value []byte) (previous []byte)
	DeprGet(key []byte) []byte
	DeprDelete(key []byte) (previous []byte)
	DeprRange(begin, end []byte) Cursor
}

// Cursor is the type returned by [BTrie.Range].
type Cursor interface {
	HasNext() bool
	Next() (key, value []byte)
}
