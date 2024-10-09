package btrie_test

import (
	"bytes"
	"fmt"
	"iter"
	"slices"

	"github.com/phiryll/btrie"
)

func newReference() btrie.OrderedBytesMap[byte] {
	return &reference{map[int32]byte{}}
}

// TODO: expand to 4 bytes (truncated)

// reference implements the OrderedBytesMap[byte] interface, but it is not a trie.
// Keys must be 1-3 bytes, and values are exactly 1 byte.
// This serves as an expected value to compare against an OrderedBytesMap implementation while testing.
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
