package kv_test

import (
	"fmt"
	"testing"

	"github.com/phiryll/kv"
	"github.com/stretchr/testify/assert"
)

const (
	maxByte = 0xFF
)

var (
	// Keys used by Bounds testing and benchmarking.
	empty  = []byte{}
	before = []byte{0x02, 0x27}
	low    = []byte{0x04, 0x99, 0x72, 0xD3}
	within = []byte{0x27, 0x83, 0x02}
	high   = []byte{0x42, 0x12, 0xBC, 0x60}
	after  = []byte{0xA5, 0x02}

	// low < low2, they share a common prefix of 2 bytes.
	low2    = []byte{0x04, 0x99, 0x9E, 0x27}
	midLows = []byte{0x04, 0x99, 0x85}
)

func next(key []byte) []byte {
	if key == nil {
		panic("key must be non-nil")
	}
	return append(append([]byte{}, key...), 0x00)
}

// This may not return the immediate predecessor, since a unique one might not exist.
// For example, {A, B, 0xFF} < {A, B, 0xFF, 0xFF} < ... < {A, B+1}.
func prev(key []byte) []byte {
	baseKeyLen := len(key) - 1
	baseKey := append([]byte{}, key[:baseKeyLen]...)
	lastByte := key[baseKeyLen]
	if lastByte == 0x00 {
		return baseKey
	}
	return append(baseKey, lastByte-1, maxByte)
}

func TestNext(t *testing.T) {
	t.Parallel()
	assert.Panics(t, func() {
		next(nil)
	})
	for _, tt := range []struct {
		key     []byte
		nextKey []byte
	}{
		{[]byte{}, []byte{0}},
		{[]byte{0x23, 0x87, 0x00}, []byte{0x23, 0x87, 0x00, 0x00}},
		{[]byte{0x23, 0x87, 0x12}, []byte{0x23, 0x87, 0x12, 0x00}},
		{[]byte{0x23, 0x87, 0xFF}, []byte{0x23, 0x87, 0xFF, 0x00}},
	} {
		copyKey := append([]byte{}, tt.key...)
		assert.Equal(t, tt.nextKey, next(tt.key))
		assert.Equal(t, copyKey, tt.key)
	}
}

func TestPrev(t *testing.T) {
	t.Parallel()
	assert.Panics(t, func() {
		prev(nil)
	})
	assert.Panics(t, func() {
		prev([]byte{})
	})
	for _, tt := range []struct {
		key     []byte
		prevKey []byte
	}{
		{[]byte{0}, []byte{}},
		{[]byte{0x23, 0x87, 0x00, 0x00}, []byte{0x23, 0x87, 0x00}},
		{[]byte{0x23, 0x87, 0x00}, []byte{0x23, 0x87}},
		{[]byte{0x23, 0x87, 0x12}, []byte{0x23, 0x87, 0x11, 0xFF}},
		{[]byte{0x23, 0x87, 0xFF}, []byte{0x23, 0x87, 0xFE, 0xFF}},
	} {
		copyKey := append([]byte{}, tt.key...)
		assert.Equal(t, tt.prevKey, prev(tt.key))
		assert.Equal(t, copyKey, tt.key)
	}
}

func TestBoundsBuilderPanics(t *testing.T) {
	t.Parallel()
	assert.Panics(t, func() {
		From(high).To(low)
	})
	assert.Panics(t, func() {
		From(low).DownTo(high)
	})
	assert.Panics(t, func() {
		From(low).To(low)
	})
	assert.Panics(t, func() {
		From(low).DownTo(low)
	})
}

func TestBoundsBuilder(t *testing.T) {
	t.Parallel()
	for _, tt := range []struct {
		first, second []byte
	}{
		{low, high},
		{nil, low},
		{low, nil},
		{nil, nil},
	} {
		t.Run(fmt.Sprintf("(%s,%s)", kv.KeyName(tt.first), kv.KeyName(tt.second)), func(t *testing.T) {
			t.Parallel()
			bounds := From(tt.first).To(tt.second)
			assert.Equal(t, tt.first, bounds.Begin)
			assert.Equal(t, tt.second, bounds.End)
			assert.False(t, bounds.IsReverse)

			bounds = From(tt.second).DownTo(tt.first)
			assert.Equal(t, tt.second, bounds.Begin)
			assert.Equal(t, tt.first, bounds.End)
			assert.True(t, bounds.IsReverse)
		})
	}
}

func TestBoundsCompareKeyPanics(t *testing.T) {
	t.Parallel()
	assert.Panics(t, func() {
		forwardAll.CompareKey(nil)
	})
	assert.Panics(t, func() {
		reverseAll.CompareKey(nil)
	})
}

//nolint:funlen,maintidx
func TestBoundsCompareKey(t *testing.T) {
	t.Parallel()
	for _, tt := range []struct {
		bounds                *Bounds
		before, within, after keySet
	}{
		// forward bounds
		{
			From(nil).To(empty),
			keySet{},
			keySet{},
			keySet{empty, next(empty), before, prev(low), low, next(low), within, prev(high), high, next(high), after},
		},
		{
			From(nil).To(low),
			keySet{},
			keySet{empty, next(empty), before, prev(low)},
			keySet{low, next(low), within, prev(high), high, next(high), after},
		},
		{
			From(nil).To(nil),
			keySet{},
			keySet{empty, next(empty), before, prev(low), low, next(low), within, prev(high), high, next(high), after},
			keySet{},
		},
		{
			From(empty).To(low),
			keySet{},
			keySet{empty, next(empty), before, prev(low)},
			keySet{low, next(low), within, prev(high), high, next(high), after},
		},
		{
			From(empty).To(nil),
			keySet{},
			keySet{empty, next(empty), before, prev(low), low, next(low), within, prev(high), high, next(high), after},
			keySet{},
		},
		{
			From(low[:2]).To(low),
			keySet{empty, next(empty), before, low[:1], prev(low[:2])},
			keySet{low[:2], next(low[:2]), low[:3], prev(low)},
			keySet{low, next(low), within, prev(high), high, next(high), after},
		},
		{
			From(low).To(low2),
			keySet{empty, next(empty), before, low[:1], low[:2], low[:3], prev(low)},
			keySet{low, next(low), midLows, prev(low2)},
			keySet{low2, next(low2), within, prev(high), high, next(high), after},
		},
		{
			From(low).To(high),
			keySet{empty, next(empty), before, prev(low)},
			keySet{low, next(low), within, prev(high)},
			keySet{high, next(high), after},
		},
		{
			From(low).To(nil),
			keySet{empty, next(empty), before, prev(low)},
			keySet{low, next(low), within, prev(high), high, next(high), after},
			keySet{},
		},

		// reverse bounds
		{
			From(nil).DownTo(low),
			keySet{},
			keySet{next(low), within, prev(high), high, next(high), after},
			keySet{empty, next(empty), before, prev(low), low},
		},
		{
			From(nil).DownTo(empty),
			keySet{},
			keySet{next(empty), before, prev(low), low, next(low), within, prev(high), high, next(high), after},
			keySet{empty},
		},
		{
			From(nil).DownTo(nil),
			keySet{},
			keySet{empty, next(empty), before, prev(low), low, next(low), within, prev(high), high, next(high), after},
			keySet{},
		},
		{
			From(high).DownTo(low),
			keySet{next(high), after},
			keySet{next(low), within, prev(high), high},
			keySet{empty, next(empty), before, prev(low), low},
		},
		{
			From(low).DownTo(low[:2]),
			keySet{next(low), within, prev(high), high, next(high), after},
			keySet{next(low[:2]), low[:3], prev(low), low},
			keySet{empty, next(empty), before, low[:1], low[:2], prev(low[:2])},
		},
		{
			From(low2).DownTo(low),
			keySet{next(low2), within, prev(high), high, next(high), after},
			keySet{next(low), midLows, prev(low2), low2},
			keySet{empty, next(empty), before, prev(low), low[:1], low[:2], low[:3], low},
		},
		{
			From(low).DownTo(empty),
			keySet{next(low), within, prev(high), high, next(high), after},
			keySet{next(empty), before, prev(low), low},
			keySet{empty},
		},
		{
			From(low).DownTo(nil),
			keySet{next(low), within, prev(high), high, next(high), after},
			keySet{empty, next(empty), before, prev(low), low},
			keySet{},
		},
		{
			From(empty).DownTo(nil),
			keySet{next(empty), before, prev(low), low, next(low), within, prev(high), high, next(high), after},
			keySet{empty},
			keySet{},
		},
	} {
		t.Run(tt.bounds.String(), func(t *testing.T) {
			t.Parallel()
			count := 0
			for _, key := range tt.before {
				assert.Equal(t, -1, tt.bounds.CompareKey(key), "%s", kv.KeyName(key))
				count++
			}
			for _, key := range tt.within {
				assert.Equal(t, 0, tt.bounds.CompareKey(key), "%s", kv.KeyName(key))
				count++
			}
			for _, key := range tt.after {
				assert.Equal(t, +1, tt.bounds.CompareKey(key), "%s", kv.KeyName(key))
				count++
			}
		})
	}
}

//nolint:funlen,maintidx
func TestChildBounds(t *testing.T) {
	t.Parallel()
	type expectedChildBounds struct {
		key         []byte
		start, stop byte
		ok          bool
	}
	for _, tt := range []struct {
		bounds   *Bounds
		expected []expectedChildBounds
	}{
		// forward, begin[0] != end[0]
		{
			From(nil).To(empty),
			[]expectedChildBounds{
				{empty, 0, 0, false},
				{next(empty), 0, 0, false},
			},
		},
		{
			From(nil).To(next(empty)),
			[]expectedChildBounds{
				{empty, 0, 0, true},
				{next(empty), 0, 0, false},
			},
		},
		{
			From(nil).To(low),
			[]expectedChildBounds{
				{empty, 0, low[0], true},
				{next(empty), 0, maxByte, true},
				{before, 0, maxByte, true},
				{low[:1], 0, low[1], true},
				{low[:2], 0, low[2], true},
				{low[:3], 0, low[3], true},
				{prev(low), 0, maxByte, true},
				{low, 0, 0, false},
				{next(low), 0, 0, false},
			},
		},
		{
			From(nil).To(nil),
			[]expectedChildBounds{
				{empty, 0, maxByte, true},
				{next(empty), 0, maxByte, true},
				{within, 0, maxByte, true},
				{after, 0, maxByte, true},
			},
		},
		{
			From(empty).To(low),
			[]expectedChildBounds{
				{empty, 0, low[0], true},
				{next(empty), 0, maxByte, true},
				{before, 0, maxByte, true},
				{low[:1], 0, low[1], true},
				{low[:2], 0, low[2], true},
				{low[:3], 0, low[3], true},
				{prev(low), 0, maxByte, true},
				{low, 0, 0, false},
				{next(low), 0, 0, false},
			},
		},
		{
			From(empty).To(nil),
			[]expectedChildBounds{
				{empty, 0, maxByte, true},
				{next(empty), 0, maxByte, true},
				{within, 0, maxByte, true},
				{after, 0, maxByte, true},
			},
		},
		{
			From(low).To(high),
			[]expectedChildBounds{
				{empty, low[0], high[0], true},
				{next(empty), 0, 0, false},
				{before, 0, 0, false},
				{low[:1], low[1], maxByte, true},
				{low[:2], low[2], maxByte, true},
				{low[:3], low[3], maxByte, true},
				{prev(low), 0, 0, false},
				{low, 0, maxByte, true},
				{next(low), 0, maxByte, true},
				{within, 0, maxByte, true},
				{high[:1], 0, high[1], true},
				{high[:2], 0, high[2], true},
				{high[:3], 0, high[3], true},
				{prev(high), 0, maxByte, true},
				{high, 0, 0, false},
				{next(high), 0, 0, false},
				{after, 0, 0, false},
			},
		},
		{
			From(low).To(nil),
			[]expectedChildBounds{
				{empty, low[0], maxByte, true},
				{next(empty), 0, 0, false},
				{before, 0, 0, false},
				{low[:1], low[1], maxByte, true},
				{low[:2], low[2], maxByte, true},
				{low[:3], low[3], maxByte, true},
				{prev(low), 0, 0, false},
				{low, 0, maxByte, true},
				{next(low), 0, maxByte, true},
				{after, 0, maxByte, true},
			},
		},

		// reverse, begin[0] != end[0]
		{
			From(nil).DownTo(low),
			[]expectedChildBounds{
				{after, maxByte, 0, true},
				{within, maxByte, 0, true},
				{next(low), maxByte, 0, true},
				{low, maxByte, 0, true},
				{prev(low), 0, 0, false},
				{low[:3], maxByte, low[3], true},
				{low[:2], maxByte, low[2], true},
				{low[:1], maxByte, low[1], true},
				{before, 0, 0, false},
				{next(empty), 0, 0, false},
				{empty, maxByte, low[0], true},
			},
		},
		{
			From(nil).DownTo(empty),
			[]expectedChildBounds{
				{after, maxByte, 0, true},
				{before, maxByte, 0, true},
				{next(empty), maxByte, 0, true},
				{empty, maxByte, 0, true},
			},
		},
		{
			From(nil).DownTo(nil),
			[]expectedChildBounds{
				{after, maxByte, 0, true},
				{before, maxByte, 0, true},
				{next(empty), maxByte, 0, true},
				{empty, maxByte, 0, true},
			},
		},
		{
			From(high).DownTo(low),
			[]expectedChildBounds{
				{after, 0, 0, false},
				{next(high), 0, 0, false},
				{high, 0, 0, false},
				{prev(high), maxByte, 0, true},
				{high[:3], high[3], 0, true},
				{high[:2], high[2], 0, true},
				{high[:1], high[1], 0, true},
				{within, maxByte, 0, true},
				{next(low), maxByte, 0, true},
				{low, maxByte, 0, true},
				{prev(low), 0, 0, false},
				{low[:3], maxByte, low[3], true},
				{low[:2], maxByte, low[2], true},
				{low[:1], maxByte, low[1], true},
				{before, 0, 0, false},
				{next(empty), 0, 0, false},
				{empty, high[0], low[0], true},
			},
		},
		{
			From(low).DownTo(empty),
			[]expectedChildBounds{
				{after, 0, 0, false},
				{within, 0, 0, false},
				{next(low), 0, 0, false},
				{low, 0, 0, false},
				{prev(low), maxByte, 0, true},
				{low[:3], low[3], 0, true},
				{low[:2], low[2], 0, true},
				{low[:1], low[1], 0, true},
				{before, maxByte, 0, true},
				{next(empty), maxByte, 0, true},
				{empty, low[0], 0, true},
			},
		},
		{
			From(low).DownTo(nil),
			[]expectedChildBounds{
				{after, 0, 0, false},
				{within, 0, 0, false},
				{next(low), 0, 0, false},
				{low, 0, 0, false},
				{prev(low), maxByte, 0, true},
				{low[:3], low[3], 0, true},
				{low[:2], low[2], 0, true},
				{low[:1], low[1], 0, true},
				{before, maxByte, 0, true},
				{next(empty), maxByte, 0, true},
				{empty, low[0], 0, true},
			},
		},
		{
			From(empty).DownTo(nil),
			[]expectedChildBounds{
				{after, 0, 0, false},
				{next(empty), 0, 0, false},
				{empty, 0, 0, false},
			},
		},

		// forward, 2 byte common prefix
		{
			From(low).To(low2),
			[]expectedChildBounds{
				{empty, low[0], low[0], true},
				{next(empty), 0, 0, false},
				{before, 0, 0, false},
				{low[:1], low[1], low[1], true},
				{low[:2], low[2], low2[2], true},
				{low[:3], low[3], maxByte, true},
				{prev(low), 0, 0, false},
				{low, 0, maxByte, true},
				{next(low), 0, maxByte, true},
				{midLows, 0, maxByte, true},
				{low2[:3], 0, low2[3], true},
				{prev(low2), 0, maxByte, true},
				{low2, 0, 0, false},
				{next(low2), 0, 0, false},
				{after, 0, 0, false},
			},
		},

		// reverse, 2 byte common prefix
		{
			From(low2).DownTo(low),
			[]expectedChildBounds{
				{after, 0, 0, false},
				{next(low2), 0, 0, false},
				{low2, 0, 0, false},
				{prev(low2), maxByte, 0, true},
				{low2[:3], low2[3], 0, true},
				{midLows, maxByte, 0, true},
				{next(low), maxByte, 0, true},
				{low, maxByte, 0, true},
				{prev(low), 0, 0, false},
				{low[:3], maxByte, low[3], true},
				{low[:2], low2[2], low[2], true},
				{low[:1], low[1], low[1], true},
				{before, 0, 0, false},
				{next(empty), 0, 0, false},
				{empty, low[0], low[0], true},
			},
		},

		// forward, From is a prefix of To
		{
			From(low[:2]).To(low),
			[]expectedChildBounds{
				{empty, low[0], low[0], true},
				{next(empty), 0, 0, false},
				{before, 0, 0, false},
				{low[:1], low[1], low[1], true},
				{prev(low[:2]), 0, 0, false},
				{low[:2], 0, low[2], true},
				{next(low[:2]), 0, maxByte, true},
				{low[:3], 0, low[3], true},
				{prev(low), 0, maxByte, true},
				{low, 0, 0, false},
				{next(low), 0, 0, false},
				{after, 0, 0, false},
			},
		},

		// reverse, DownTo is a prefix of From
		{
			From(low).DownTo(low[:2]),
			[]expectedChildBounds{
				{after, 0, 0, false},
				{next(low), 0, 0, false},
				{low, 0, 0, false},
				{prev(low), maxByte, 0, true},
				{low[:3], low[3], 0, true},
				{next(low[:2]), maxByte, 0, true},
				{low[:2], low[2], 0, true},
				{prev(low[:2]), 0, 0, false},
				{low[:1], low[1], low[1], true},
				{before, 0, 0, false},
				{next(empty), 0, 0, false},
				{empty, low[0], low[0], true},
			},
		},
	} {
		t.Run(tt.bounds.String(), func(t *testing.T) {
			t.Parallel()
			for _, exp := range tt.expected {
				start, stop, ok := kv.TestingChildBounds(tt.bounds, exp.key)
				assert.Equal(t, exp.start, start, "%s", kv.KeyName(exp.key))
				assert.Equal(t, exp.stop, stop, "%s", kv.KeyName(exp.key))
				assert.Equal(t, exp.ok, ok, "%s", kv.KeyName(exp.key))
			}
		})
	}
}
