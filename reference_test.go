package kv_test

import (
	"bytes"
	"fmt"
	"iter"
	"slices"
	"strings"

	"github.com/phiryll/kv"
)

// reference serves as an expected value to compare against while testing,
// and a source of entries from which to create a new Store.
// This implementation is meant to be as trivially correct as possible.
type reference struct {
	entries map[string]byte
	ascKeys [][]byte
	dirty   bool
}

func newReference() *reference {
	return &reference{
		entries: map[string]byte{},
	}
}

func (r *reference) makeDirty() {
	r.ascKeys = nil
	r.dirty = true
}

func (r *reference) refresh() {
	if !r.dirty {
		return
	}
	var keys [][]byte
	for key := range r.entries {
		keys = append(keys, []byte(key))
	}
	slices.SortFunc(keys, bytes.Compare)
	r.ascKeys = keys
	r.dirty = false
}

func (r *reference) Get(key []byte) (byte, bool) {
	if key == nil {
		panic("key must be non-nil")
	}
	value, ok := r.entries[string(key)]
	return value, ok
}

func (r *reference) Set(key []byte, value byte) (byte, bool) {
	if key == nil {
		panic("key must be non-nil")
	}
	prev, ok := r.entries[string(key)]
	r.entries[string(key)] = value
	r.makeDirty()
	return prev, ok
}

func (r *reference) Delete(key []byte) (byte, bool) {
	if key == nil {
		panic("key must be non-nil")
	}
	value, ok := r.entries[string(key)]
	if ok {
		delete(r.entries, string(key))
		r.makeDirty()
	}
	return value, ok
}

func (r *reference) Range(bounds *Bounds) iter.Seq2[[]byte, byte] {
	bounds = bounds.Clone()
	if bounds.IsReverse {
		// Desc behaves differently than Range, with low inclusive and high exclusive.
		low, high := bounds.End, bounds.Begin
		if low != nil {
			low = nextKey(low)
		}
		if high != nil {
			high = nextKey(high)
		}
		return r.Desc(low, high)
	}
	return r.Asc(bounds.Begin, bounds.End)
}

// Future methods on Store, replacing Range.

func (r *reference) All() iter.Seq2[[]byte, byte] {
	return func(yield func([]byte, byte) bool) {
		for k, v := range r.entries {
			if !yield([]byte(k), v) {
				return
			}
		}
	}
}

func (r *reference) between(low, high []byte) (int, int) {
	lowIndex, highIndex := 0, len(r.ascKeys)
	if low != nil {
		lowIndex, _ = slices.BinarySearchFunc(r.ascKeys, low, bytes.Compare)
	}
	if high != nil {
		highIndex, _ = slices.BinarySearchFunc(r.ascKeys[lowIndex:], high, bytes.Compare)
		highIndex += lowIndex
	}
	return lowIndex, highIndex
}

func (r *reference) Asc(low, high []byte) iter.Seq2[[]byte, byte] {
	if low != nil && high != nil && bytes.Compare(low, high) >= 0 {
		panic("low >= high")
	}
	return func(yield func([]byte, byte) bool) {
		r.refresh()
		lowIndex, highIndex := r.between(low, high)
		for i := lowIndex; i < highIndex; i++ {
			key := r.ascKeys[i]
			if !yield(key, r.entries[string(key)]) {
				return
			}
		}
	}
}

func (r *reference) Desc(low, high []byte) iter.Seq2[[]byte, byte] {
	if low != nil && high != nil && bytes.Compare(low, high) >= 0 {
		panic("low >= high")
	}
	return func(yield func([]byte, byte) bool) {
		r.refresh()
		lowIndex, highIndex := r.between(low, high)
		for i := highIndex - 1; i >= lowIndex; i-- {
			key := r.ascKeys[i]
			if !yield(key, r.entries[string(key)]) {
				return
			}
		}
	}
}

func (r *reference) String() string {
	var s strings.Builder
	s.WriteString("{\n")
	fmt.Fprintf(&s, "  dirty: %v\n", r.dirty)
	s.WriteString("  entries: {")
	for key, value := range r.entries {
		fmt.Fprintf(&s, "%s:%02X, ", kv.KeyName([]byte(key)), value)
	}
	s.WriteString(" }\n")
	s.WriteString("  ascKeys: {")
	for _, key := range r.ascKeys {
		fmt.Fprintf(&s, "%s, ", kv.KeyName(key))
	}
	s.WriteString(" }\n")
	s.WriteString("}")
	return s.String()
}
