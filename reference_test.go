package btrie_test

import (
	"fmt"
	"iter"
	"slices"
	"strings"
)

func newReference() Obm {
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

//nolint:revive
func (r *reference) String() string {
	var s strings.Builder
	s.WriteString("{")
	for k, v := range r.Range(From(nil).To(nil)) {
		fmt.Fprintf(&s, "%X:%v, ", k, v)
	}
	s.WriteString("}")
	return s.String()
}

func (r *reference) Range(bounds Bounds) iter.Seq2[[]byte, byte] {
	entries := []entry{}
	for k, v := range r.m {
		key := []byte(k)
		if bounds.Compare(key) != 0 {
			continue
		}
		entries = append(entries, entry{key, v})
	}
	if bounds.IsReverse() {
		slices.SortFunc(entries, cmpEntryReverse)
	} else {
		slices.SortFunc(entries, cmpEntryForward)
	}
	return func(yield func([]byte, byte) bool) {
		for _, entry := range entries {
			if !yield(entry.Key, entry.Value) {
				return
			}
		}
	}
}
