package btrie_test

import (
	"bytes"
	"fmt"
	"iter"
	"slices"
	"sort"

	"github.com/phiryll/btrie"
)

func deprNewReference() btrie.BTrie[byte] {
	return &reference{map[int32]byte{}}
}

func newReference() btrie.OrderedBytesMap[byte] {
	return &reference{map[int32]byte{}}
}

// reference mostly implements the BTrie interface, but it is not a BTrie.
// Keys must be 1-3 bytes, and values are exactly 1 byte.
// This serves as an expected value to compare against a BTrie implementation while testing.
type reference struct {
	// the high byte is 0, 1, or 2 if the key is 1, 2, or 3 bytes
	m map[int32]byte
}

func refIndex(key []byte) int32 {
	switch len(key) {
	case 1:
		return int32(key[0])
	case 2:
		return 0x01_00_00_00 | int32(key[0])<<8 | int32(key[1])
	case 3:
		return 0x02_00_00_00 | int32(key[0])<<16 | int32(key[1])<<8 | int32(key[2])
	default:
		panic(fmt.Sprintf("unsupported key length: %d", len(key)))
	}
}

func refKey(index int32) []byte {
	switch index & 0x03_00_00_00 {
	case 0:
		return []byte{byte(index)}
	case 0x01_00_00_00:
		return []byte{byte(index >> 8), byte(index)}
	case 0x02_00_00_00:
		return []byte{byte(index >> 16), byte(index >> 8), byte(index)}
	default:
		panic(fmt.Sprintf("invalid index: %d", index))
	}
}

func (r *reference) Put(key []byte, value byte) (byte, bool) {
	index := refIndex(key)
	prev, ok := r.m[index]
	r.m[index] = value
	if ok {
		return prev, true
	}
	return 0, false
}

func (r *reference) Get(key []byte) (byte, bool) {
	value, ok := r.m[refIndex(key)]
	return value, ok
}

func (r *reference) Delete(key []byte) (byte, bool) {
	index := refIndex(key)
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
		key := refKey(k)
		if !bounds.Contains(key) {
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

func (r *reference) DeprPut(key []byte, value byte) byte {
	prev, ok := r.Put(key, value)
	if !ok {
		return 0
	}
	return prev
}

func (r *reference) DeprGet(key []byte) byte {
	value, ok := r.Get(key)
	if !ok {
		return 0
	}
	return value
}

func (r *reference) DeprDelete(key []byte) byte {
	value, ok := r.Delete(key)
	if !ok {
		return 0
	}
	return value
}

func (r *reference) DeprRange(begin, end []byte) btrie.Cursor[byte] {
	entries := []btrie.Entry[byte]{}
	for k, v := range r.m {
		key := refKey(k)
		if begin != nil && bytes.Compare(key, begin) < 0 {
			continue
		}
		if end != nil && bytes.Compare(key, end) >= 0 {
			continue
		}
		entries = append(entries, btrie.Entry[byte]{key, v})
	}
	sort.Slice(entries, func(i, j int) bool {
		return bytes.Compare(entries[i].Key, entries[j].Key) < 0
	})
	return &deprCursor[byte]{entries, 0}
}

type deprCursor[V any] struct {
	entries []btrie.Entry[V]
	index   int
}

func (c *deprCursor[V]) HasNext() bool {
	return c.index < len(c.entries)
}

func (c *deprCursor[V]) Next() ([]byte, V) {
	entry := c.entries[c.index]
	c.index++
	return entry.Key, entry.Value
}
