// Package btrie defines interfaces for and provides multiple implementations of a binary trie.
package btrie

type Entry[V any] struct {
	Key   []byte
	Value V
}
