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
	empty   = []byte{}
	before  = []byte{0x02, 0x27}
	lowKey  = []byte{0x04, 0x99, 0x72, 0xD3}
	within  = []byte{0x27, 0x83, 0x02}
	highKey = []byte{0x42, 0x12, 0xBC, 0x60}
	after   = []byte{0xA5, 0x02}

	// low < low2, they share a common prefix of 2 bytes.
	low2    = []byte{0x04, 0x99, 0x9E, 0x27}
	midLows = []byte{0x04, 0x99, 0x85}
)

func TestBoundsBuilderPanics(t *testing.T) {
	t.Parallel()
	assert.Panics(t, func() {
		From(highKey).To(lowKey)
	})
	assert.Panics(t, func() {
		From(lowKey).DownTo(highKey)
	})
	assert.Panics(t, func() {
		From(lowKey).To(lowKey)
	})
	assert.Panics(t, func() {
		From(lowKey).DownTo(lowKey)
	})
}

func TestBoundsBuilder(t *testing.T) {
	t.Parallel()
	for _, tt := range []struct {
		first, second []byte
	}{
		{lowKey, highKey},
		{nil, lowKey},
		{lowKey, nil},
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
			keySet{
				empty, nextKey(empty), before, prevKey(lowKey), lowKey, nextKey(lowKey), within,
				prevKey(highKey), highKey, nextKey(highKey), after,
			},
		},
		{
			From(nil).To(lowKey),
			keySet{},
			keySet{empty, nextKey(empty), before, prevKey(lowKey)},
			keySet{lowKey, nextKey(lowKey), within, prevKey(highKey), highKey, nextKey(highKey), after},
		},
		{
			From(nil).To(nil),
			keySet{},
			keySet{
				empty, nextKey(empty), before, prevKey(lowKey), lowKey, nextKey(lowKey), within,
				prevKey(highKey), highKey, nextKey(highKey), after,
			},
			keySet{},
		},
		{
			From(empty).To(lowKey),
			keySet{},
			keySet{empty, nextKey(empty), before, prevKey(lowKey)},
			keySet{lowKey, nextKey(lowKey), within, prevKey(highKey), highKey, nextKey(highKey), after},
		},
		{
			From(empty).To(nil),
			keySet{},
			keySet{
				empty, nextKey(empty), before, prevKey(lowKey), lowKey, nextKey(lowKey), within,
				prevKey(highKey), highKey, nextKey(highKey), after,
			},
			keySet{},
		},
		{
			From(lowKey[:2]).To(lowKey),
			keySet{empty, nextKey(empty), before, lowKey[:1], prevKey(lowKey[:2])},
			keySet{lowKey[:2], nextKey(lowKey[:2]), lowKey[:3], prevKey(lowKey)},
			keySet{lowKey, nextKey(lowKey), within, prevKey(highKey), highKey, nextKey(highKey), after},
		},
		{
			From(lowKey).To(low2),
			keySet{empty, nextKey(empty), before, lowKey[:1], lowKey[:2], lowKey[:3], prevKey(lowKey)},
			keySet{lowKey, nextKey(lowKey), midLows, prevKey(low2)},
			keySet{low2, nextKey(low2), within, prevKey(highKey), highKey, nextKey(highKey), after},
		},
		{
			From(lowKey).To(highKey),
			keySet{empty, nextKey(empty), before, prevKey(lowKey)},
			keySet{lowKey, nextKey(lowKey), within, prevKey(highKey)},
			keySet{highKey, nextKey(highKey), after},
		},
		{
			From(lowKey).To(nil),
			keySet{empty, nextKey(empty), before, prevKey(lowKey)},
			keySet{lowKey, nextKey(lowKey), within, prevKey(highKey), highKey, nextKey(highKey), after},
			keySet{},
		},

		// reverse bounds
		{
			From(nil).DownTo(lowKey),
			keySet{},
			keySet{nextKey(lowKey), within, prevKey(highKey), highKey, nextKey(highKey), after},
			keySet{empty, nextKey(empty), before, prevKey(lowKey), lowKey},
		},
		{
			From(nil).DownTo(empty),
			keySet{},
			keySet{
				nextKey(empty), before, prevKey(lowKey), lowKey, nextKey(lowKey), within,
				prevKey(highKey), highKey, nextKey(highKey), after,
			},
			keySet{empty},
		},
		{
			From(nil).DownTo(nil),
			keySet{},
			keySet{
				empty, nextKey(empty), before, prevKey(lowKey), lowKey, nextKey(lowKey), within,
				prevKey(highKey), highKey, nextKey(highKey), after,
			},
			keySet{},
		},
		{
			From(highKey).DownTo(lowKey),
			keySet{nextKey(highKey), after},
			keySet{nextKey(lowKey), within, prevKey(highKey), highKey},
			keySet{empty, nextKey(empty), before, prevKey(lowKey), lowKey},
		},
		{
			From(lowKey).DownTo(lowKey[:2]),
			keySet{nextKey(lowKey), within, prevKey(highKey), highKey, nextKey(highKey), after},
			keySet{nextKey(lowKey[:2]), lowKey[:3], prevKey(lowKey), lowKey},
			keySet{empty, nextKey(empty), before, lowKey[:1], lowKey[:2], prevKey(lowKey[:2])},
		},
		{
			From(low2).DownTo(lowKey),
			keySet{nextKey(low2), within, prevKey(highKey), highKey, nextKey(highKey), after},
			keySet{nextKey(lowKey), midLows, prevKey(low2), low2},
			keySet{empty, nextKey(empty), before, prevKey(lowKey), lowKey[:1], lowKey[:2], lowKey[:3], lowKey},
		},
		{
			From(lowKey).DownTo(empty),
			keySet{nextKey(lowKey), within, prevKey(highKey), highKey, nextKey(highKey), after},
			keySet{nextKey(empty), before, prevKey(lowKey), lowKey},
			keySet{empty},
		},
		{
			From(lowKey).DownTo(nil),
			keySet{nextKey(lowKey), within, prevKey(highKey), highKey, nextKey(highKey), after},
			keySet{empty, nextKey(empty), before, prevKey(lowKey), lowKey},
			keySet{},
		},
		{
			From(empty).DownTo(nil),
			keySet{
				nextKey(empty), before, prevKey(lowKey), lowKey, nextKey(lowKey), within,
				prevKey(highKey), highKey, nextKey(highKey), after,
			},
			keySet{empty},
			keySet{},
		},
	} {
		t.Run(tt.bounds.String(), func(t *testing.T) {
			t.Parallel()
			count := 0
			for _, k := range tt.before {
				assert.Equal(t, -1, tt.bounds.CompareKey(k), "%s", kv.KeyName(k))
				count++
			}
			for _, k := range tt.within {
				assert.Equal(t, 0, tt.bounds.CompareKey(k), "%s", kv.KeyName(k))
				count++
			}
			for _, k := range tt.after {
				assert.Equal(t, +1, tt.bounds.CompareKey(k), "%s", kv.KeyName(k))
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
				{nextKey(empty), 0, 0, false},
			},
		},
		{
			From(nil).To(nextKey(empty)),
			[]expectedChildBounds{
				{empty, 0, 0, true},
				{nextKey(empty), 0, 0, false},
			},
		},
		{
			From(nil).To(lowKey),
			[]expectedChildBounds{
				{empty, 0, lowKey[0], true},
				{nextKey(empty), 0, maxByte, true},
				{before, 0, maxByte, true},
				{lowKey[:1], 0, lowKey[1], true},
				{lowKey[:2], 0, lowKey[2], true},
				{lowKey[:3], 0, lowKey[3], true},
				{prevKey(lowKey), 0, maxByte, true},
				{lowKey, 0, 0, false},
				{nextKey(lowKey), 0, 0, false},
			},
		},
		{
			From(nil).To(nil),
			[]expectedChildBounds{
				{empty, 0, maxByte, true},
				{nextKey(empty), 0, maxByte, true},
				{within, 0, maxByte, true},
				{after, 0, maxByte, true},
			},
		},
		{
			From(empty).To(lowKey),
			[]expectedChildBounds{
				{empty, 0, lowKey[0], true},
				{nextKey(empty), 0, maxByte, true},
				{before, 0, maxByte, true},
				{lowKey[:1], 0, lowKey[1], true},
				{lowKey[:2], 0, lowKey[2], true},
				{lowKey[:3], 0, lowKey[3], true},
				{prevKey(lowKey), 0, maxByte, true},
				{lowKey, 0, 0, false},
				{nextKey(lowKey), 0, 0, false},
			},
		},
		{
			From(empty).To(nil),
			[]expectedChildBounds{
				{empty, 0, maxByte, true},
				{nextKey(empty), 0, maxByte, true},
				{within, 0, maxByte, true},
				{after, 0, maxByte, true},
			},
		},
		{
			From(lowKey).To(highKey),
			[]expectedChildBounds{
				{empty, lowKey[0], highKey[0], true},
				{nextKey(empty), 0, 0, false},
				{before, 0, 0, false},
				{lowKey[:1], lowKey[1], maxByte, true},
				{lowKey[:2], lowKey[2], maxByte, true},
				{lowKey[:3], lowKey[3], maxByte, true},
				{prevKey(lowKey), 0, 0, false},
				{lowKey, 0, maxByte, true},
				{nextKey(lowKey), 0, maxByte, true},
				{within, 0, maxByte, true},
				{highKey[:1], 0, highKey[1], true},
				{highKey[:2], 0, highKey[2], true},
				{highKey[:3], 0, highKey[3], true},
				{prevKey(highKey), 0, maxByte, true},
				{highKey, 0, 0, false},
				{nextKey(highKey), 0, 0, false},
				{after, 0, 0, false},
			},
		},
		{
			From(lowKey).To(nil),
			[]expectedChildBounds{
				{empty, lowKey[0], maxByte, true},
				{nextKey(empty), 0, 0, false},
				{before, 0, 0, false},
				{lowKey[:1], lowKey[1], maxByte, true},
				{lowKey[:2], lowKey[2], maxByte, true},
				{lowKey[:3], lowKey[3], maxByte, true},
				{prevKey(lowKey), 0, 0, false},
				{lowKey, 0, maxByte, true},
				{nextKey(lowKey), 0, maxByte, true},
				{after, 0, maxByte, true},
			},
		},

		// reverse, begin[0] != end[0]
		{
			From(nil).DownTo(lowKey),
			[]expectedChildBounds{
				{after, maxByte, 0, true},
				{within, maxByte, 0, true},
				{nextKey(lowKey), maxByte, 0, true},
				{lowKey, maxByte, 0, true},
				{prevKey(lowKey), 0, 0, false},
				{lowKey[:3], maxByte, lowKey[3], true},
				{lowKey[:2], maxByte, lowKey[2], true},
				{lowKey[:1], maxByte, lowKey[1], true},
				{before, 0, 0, false},
				{nextKey(empty), 0, 0, false},
				{empty, maxByte, lowKey[0], true},
			},
		},
		{
			From(nil).DownTo(empty),
			[]expectedChildBounds{
				{after, maxByte, 0, true},
				{before, maxByte, 0, true},
				{nextKey(empty), maxByte, 0, true},
				{empty, maxByte, 0, true},
			},
		},
		{
			From(nil).DownTo(nil),
			[]expectedChildBounds{
				{after, maxByte, 0, true},
				{before, maxByte, 0, true},
				{nextKey(empty), maxByte, 0, true},
				{empty, maxByte, 0, true},
			},
		},
		{
			From(highKey).DownTo(lowKey),
			[]expectedChildBounds{
				{after, 0, 0, false},
				{nextKey(highKey), 0, 0, false},
				{highKey, 0, 0, false},
				{prevKey(highKey), maxByte, 0, true},
				{highKey[:3], highKey[3], 0, true},
				{highKey[:2], highKey[2], 0, true},
				{highKey[:1], highKey[1], 0, true},
				{within, maxByte, 0, true},
				{nextKey(lowKey), maxByte, 0, true},
				{lowKey, maxByte, 0, true},
				{prevKey(lowKey), 0, 0, false},
				{lowKey[:3], maxByte, lowKey[3], true},
				{lowKey[:2], maxByte, lowKey[2], true},
				{lowKey[:1], maxByte, lowKey[1], true},
				{before, 0, 0, false},
				{nextKey(empty), 0, 0, false},
				{empty, highKey[0], lowKey[0], true},
			},
		},
		{
			From(lowKey).DownTo(empty),
			[]expectedChildBounds{
				{after, 0, 0, false},
				{within, 0, 0, false},
				{nextKey(lowKey), 0, 0, false},
				{lowKey, 0, 0, false},
				{prevKey(lowKey), maxByte, 0, true},
				{lowKey[:3], lowKey[3], 0, true},
				{lowKey[:2], lowKey[2], 0, true},
				{lowKey[:1], lowKey[1], 0, true},
				{before, maxByte, 0, true},
				{nextKey(empty), maxByte, 0, true},
				{empty, lowKey[0], 0, true},
			},
		},
		{
			From(lowKey).DownTo(nil),
			[]expectedChildBounds{
				{after, 0, 0, false},
				{within, 0, 0, false},
				{nextKey(lowKey), 0, 0, false},
				{lowKey, 0, 0, false},
				{prevKey(lowKey), maxByte, 0, true},
				{lowKey[:3], lowKey[3], 0, true},
				{lowKey[:2], lowKey[2], 0, true},
				{lowKey[:1], lowKey[1], 0, true},
				{before, maxByte, 0, true},
				{nextKey(empty), maxByte, 0, true},
				{empty, lowKey[0], 0, true},
			},
		},
		{
			From(empty).DownTo(nil),
			[]expectedChildBounds{
				{after, 0, 0, false},
				{nextKey(empty), 0, 0, false},
				{empty, 0, 0, false},
			},
		},

		// forward, 2 byte common prefix
		{
			From(lowKey).To(low2),
			[]expectedChildBounds{
				{empty, lowKey[0], lowKey[0], true},
				{nextKey(empty), 0, 0, false},
				{before, 0, 0, false},
				{lowKey[:1], lowKey[1], lowKey[1], true},
				{lowKey[:2], lowKey[2], low2[2], true},
				{lowKey[:3], lowKey[3], maxByte, true},
				{prevKey(lowKey), 0, 0, false},
				{lowKey, 0, maxByte, true},
				{nextKey(lowKey), 0, maxByte, true},
				{midLows, 0, maxByte, true},
				{low2[:3], 0, low2[3], true},
				{prevKey(low2), 0, maxByte, true},
				{low2, 0, 0, false},
				{nextKey(low2), 0, 0, false},
				{after, 0, 0, false},
			},
		},

		// reverse, 2 byte common prefix
		{
			From(low2).DownTo(lowKey),
			[]expectedChildBounds{
				{after, 0, 0, false},
				{nextKey(low2), 0, 0, false},
				{low2, 0, 0, false},
				{prevKey(low2), maxByte, 0, true},
				{low2[:3], low2[3], 0, true},
				{midLows, maxByte, 0, true},
				{nextKey(lowKey), maxByte, 0, true},
				{lowKey, maxByte, 0, true},
				{prevKey(lowKey), 0, 0, false},
				{lowKey[:3], maxByte, lowKey[3], true},
				{lowKey[:2], low2[2], lowKey[2], true},
				{lowKey[:1], lowKey[1], lowKey[1], true},
				{before, 0, 0, false},
				{nextKey(empty), 0, 0, false},
				{empty, lowKey[0], lowKey[0], true},
			},
		},

		// forward, From is a prefix of To
		{
			From(lowKey[:2]).To(lowKey),
			[]expectedChildBounds{
				{empty, lowKey[0], lowKey[0], true},
				{nextKey(empty), 0, 0, false},
				{before, 0, 0, false},
				{lowKey[:1], lowKey[1], lowKey[1], true},
				{prevKey(lowKey[:2]), 0, 0, false},
				{lowKey[:2], 0, lowKey[2], true},
				{nextKey(lowKey[:2]), 0, maxByte, true},
				{lowKey[:3], 0, lowKey[3], true},
				{prevKey(lowKey), 0, maxByte, true},
				{lowKey, 0, 0, false},
				{nextKey(lowKey), 0, 0, false},
				{after, 0, 0, false},
			},
		},

		// reverse, DownTo is a prefix of From
		{
			From(lowKey).DownTo(lowKey[:2]),
			[]expectedChildBounds{
				{after, 0, 0, false},
				{nextKey(lowKey), 0, 0, false},
				{lowKey, 0, 0, false},
				{prevKey(lowKey), maxByte, 0, true},
				{lowKey[:3], lowKey[3], 0, true},
				{nextKey(lowKey[:2]), maxByte, 0, true},
				{lowKey[:2], lowKey[2], 0, true},
				{prevKey(lowKey[:2]), 0, 0, false},
				{lowKey[:1], lowKey[1], lowKey[1], true},
				{before, 0, 0, false},
				{nextKey(empty), 0, 0, false},
				{empty, lowKey[0], lowKey[0], true},
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
