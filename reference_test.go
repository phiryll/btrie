package kv_test

import (
	"cmp"
	"fmt"
	"iter"
	"maps"
	"slices"
	"strings"

	"github.com/phiryll/kv"
)

func newReference() TestStore {
	return reference{}
}

// reference implements the TestStore interface, but it is not a trie.
// This serves as an expected value to compare against a Store[byte] implementation while testing.
type reference map[string]byte

func (r reference) Clone() TestStore {
	return maps.Clone(r)
}

func (r reference) Set(key []byte, value byte) (byte, bool) {
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

// Does not work with NaN.
func negCompare[T cmp.Ordered](x, y T) int {
	if x < y {
		return +1
	}
	if x > y {
		return -1
	}
	return 0
}

func (r reference) Range(bounds *Bounds) iter.Seq2[[]byte, byte] {
	bounds = bounds.Clone()
	return func(yield func([]byte, byte) bool) {
		var keys []string
		for key := range r {
			if bounds.CompareKey([]byte(key)) == 0 {
				keys = append(keys, key)
			}
		}
		if bounds.IsReverse {
			slices.SortFunc(keys, negCompare)
		} else {
			slices.Sort(keys)
		}
		for _, key := range keys {
			if !yield([]byte(key), r[key]) {
				return
			}
		}
	}
}

func (r reference) String() string {
	var s strings.Builder
	s.WriteString("{")
	for _, key := range slices.Sorted(maps.Keys(r)) {
		fmt.Fprintf(&s, "%s:%v, ", kv.KeyName([]byte(key)), r[key])
	}
	s.WriteString("}")
	return s.String()
}
