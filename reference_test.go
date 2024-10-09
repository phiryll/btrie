package btrie_test

import (
	"bytes"
	"iter"
	"slices"

	"github.com/phiryll/btrie"
)

func newReference() btrie.OrderedBytesMap[byte] {
	return &reference{map[string]byte{}}
}

// reference implements the OrderedBytesMap[byte] interface, but it is not a trie.
// This serves as an expected value to compare against an OrderedBytesMap implementation while testing.
type reference struct {
	m map[string]byte
}

func (r *reference) Put(key []byte, value byte) (byte, bool) {
	index := string(key)
	prev, ok := r.m[index]
	r.m[index] = value
	if ok {
		return prev, true
	}
	return 0, false
}

func (r *reference) Get(key []byte) (byte, bool) {
	value, ok := r.m[string(key)]
	return value, ok
}

func (r *reference) Delete(key []byte) (byte, bool) {
	index := string(key)
	value, ok := r.m[index]
	delete(r.m, index)
	return value, ok
}

func (r *reference) Range(bounds *Bounds) iter.Seq2[[]byte, byte] {
	type refEntry struct {
		Key   []byte
		Value byte
	}
	entries := []refEntry{}
	for k, v := range r.m {
		key := []byte(k)
		if bounds.Compare(key) != 0 {
			continue
		}
		entries = append(entries, refEntry{key, v})
	}
	slices.SortFunc(entries, func(a, b refEntry) int {
		return bytes.Compare(a.Key, b.Key)
	})
	return func(yield func([]byte, byte) bool) {
		for _, entry := range entries {
			if !yield(entry.Key, entry.Value) {
				return
			}
		}
	}
}

func (r *reference) Cursor(bounds *Bounds) iter.Seq[btrie.Pos[byte]] {
	return func(yield func(btrie.Pos[byte]) bool) {
		for k := range r.Range(bounds) {
			if !yield(btrie.DefaultPos(r, k)) {
				return
			}
		}
	}
}
