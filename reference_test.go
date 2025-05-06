package btrie_test

import (
	"fmt"
	"iter"
	"maps"
	"slices"
	"strings"
)

func newReference() TestBTrie {
	return reference{}
}

// reference implements the TestBTrie interface, but it is not a trie.
// This serves as an expected value to compare against a BTrie[byte] implementation while testing.
type reference map[string]byte

func (r reference) Clone() TestBTrie {
	return maps.Clone(r)
}

func (r reference) Put(key []byte, value byte) (byte, bool) {
	if key == nil {
		panic("key must be non-nil")
	}
	index := string(key)
	prev, ok := r[index]
	r[index] = value
	if ok {
		return prev, true
	}
	return 0, false
}

func (r reference) Get(key []byte) (byte, bool) {
	if key == nil {
		panic("key must be non-nil")
	}
	value, ok := r[string(key)]
	return value, ok
}

func (r reference) Delete(key []byte) (byte, bool) {
	if key == nil {
		panic("key must be non-nil")
	}
	index := string(key)
	value, ok := r[index]
	delete(r, index)
	return value, ok
}

func (r reference) String() string {
	var s strings.Builder
	s.WriteString("{")
	for k, v := range r.Range(forwardAll) {
		fmt.Fprintf(&s, "%s:%v, ", keyName(k), v)
	}
	s.WriteString("}")
	return s.String()
}

func (r reference) Range(bounds *Bounds) iter.Seq2[[]byte, byte] {
	entries := []entry{}
	for k, v := range r {
		key := []byte(k)
		if bounds.CompareKey(key) == 0 {
			entries = append(entries, entry{key, v})
		}
	}
	if bounds.IsReverse {
		slices.SortFunc(entries, cmpEntryReverse)
	} else {
		slices.SortFunc(entries, cmpEntryForward)
	}
	return func(yield func([]byte, byte) bool) {
		for _, entry := range entries {
			if !yield(entry.key, entry.value) {
				return
			}
		}
	}
}
