package btrie_test

import (
	"bytes"
	"fmt"
	"sort"
)

func newReference() BTrie {
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

func (r *reference) Put(key, value []byte) []byte {
	index := refIndex(key)
	b, ok := r.m[index]
	if value == nil {
		delete(r.m, index)
	} else {
		r.m[index] = value[0]
	}
	if ok {
		return []byte{b}
	}
	return nil
}

func (r *reference) Get(key []byte) []byte {
	if b, ok := r.m[refIndex(key)]; ok {
		return []byte{b}
	}
	return nil
}

func (r *reference) Delete(key []byte) []byte {
	return r.Put(key, nil)
}

func (r *reference) Range(begin, end []byte) Cursor {
	entries := []Entry{}
	for k, v := range r.m {
		key := refKey(k)
		if begin != nil && bytes.Compare(key, begin) < 0 {
			continue
		}
		if end != nil && bytes.Compare(key, end) >= 0 {
			continue
		}
		entries = append(entries, Entry{key, []byte{v}})
	}
	sort.Slice(entries, func(i, j int) bool {
		return bytes.Compare(entries[i].Key, entries[j].Key) < 0
	})
	return &cursor{entries, 0}
}

type cursor struct {
	entries []Entry
	index   int
}

func (c *cursor) HasNext() bool {
	return c.index < len(c.entries)
}

func (c *cursor) Next() Entry {
	entry := c.entries[c.index]
	c.index++
	return entry
}
