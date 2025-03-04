package btrie_test

import (
	"fmt"
	"testing"

	"github.com/phiryll/btrie"
	"github.com/stretchr/testify/assert"
)

var (
	// Testing Bounds with combinations of nil, empty, low, and high.
	// The remaining values are test arguments to Bounds.Compare().
	// Names are relative to low <= X < high.
	empty      = []byte{}
	afterEmpty = []byte{0}
	before     = []byte{0x02, 0x27}
	beforeLow  = []byte{0x04, 0x99, 0x72, 0xD2, 0xFF}
	low        = []byte{0x04, 0x99, 0x72, 0xD3}
	afterLow   = []byte{0x04, 0x99, 0x72, 0xD3, 0x00}
	within     = []byte{0x27, 0x83, 0x02}
	beforeHigh = []byte{0x42, 0x12, 0xBC, 0x5F, 0xFF}
	high       = []byte{0x42, 0x12, 0xBC, 0x60}
	afterHigh  = []byte{0x42, 0x12, 0xBC, 0x60, 0x00}
	after      = []byte{0xA5, 0x00}

	// low < low2, they share a common prefix of 2 bytes.
	beforeLow2 = []byte{0x04, 0x99, 0x9E, 0x26, 0xFF}
	low2       = []byte{0x04, 0x99, 0x9E, 0x27}
	afterLow2  = []byte{0x04, 0x99, 0x9E, 0x27, 0x00}
)

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
		t.Run(fmt.Sprintf("(%s,%s)", keyName(tt.first), keyName(tt.second)), func(t *testing.T) {
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

func TestBoundsComparePanics(t *testing.T) {
	t.Parallel()
	assert.Panics(t, func() {
		forwardAll.Compare(nil)
	})
	assert.Panics(t, func() {
		reverseAll.Compare(nil)
	})
}

//nolint:funlen
func TestBoundsCompare(t *testing.T) {
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
			keySet{empty, afterEmpty, before, beforeLow, low, afterLow, within, beforeHigh, high, afterHigh, after},
		},
		{
			From(nil).To(low),
			keySet{},
			keySet{empty, afterEmpty, before, beforeLow},
			keySet{low, afterLow, within, beforeHigh, high, afterHigh, after},
		},
		{
			From(nil).To(nil),
			keySet{},
			keySet{empty, afterEmpty, before, beforeLow, low, afterLow, within, beforeHigh, high, afterHigh, after},
			keySet{},
		},
		{
			From(empty).To(low),
			keySet{},
			keySet{empty, afterEmpty, before, beforeLow},
			keySet{low, afterLow, within, beforeHigh, high, afterHigh, after},
		},
		{
			From(empty).To(nil),
			keySet{},
			keySet{empty, afterEmpty, before, beforeLow, low, afterLow, within, beforeHigh, high, afterHigh, after},
			keySet{},
		},
		{
			From(low).To(high),
			keySet{empty, afterEmpty, before, beforeLow},
			keySet{low, afterLow, within, beforeHigh},
			keySet{high, afterHigh, after},
		},
		{
			From(low).To(nil),
			keySet{empty, afterEmpty, before, beforeLow},
			keySet{low, afterLow, within, beforeHigh, high, afterHigh, after},
			keySet{},
		},

		// reverse bounds
		{
			From(nil).DownTo(low),
			keySet{},
			keySet{afterLow, within, beforeHigh, high, afterHigh, after},
			keySet{empty, afterEmpty, before, beforeLow, low},
		},
		{
			From(nil).DownTo(empty),
			keySet{},
			keySet{afterEmpty, before, beforeLow, low, afterLow, within, beforeHigh, high, afterHigh, after},
			keySet{empty},
		},
		{
			From(nil).DownTo(nil),
			keySet{},
			keySet{empty, afterEmpty, before, beforeLow, low, afterLow, within, beforeHigh, high, afterHigh, after},
			keySet{},
		},
		{
			From(high).DownTo(low),
			keySet{afterHigh, after},
			keySet{afterLow, within, beforeHigh, high},
			keySet{empty, afterEmpty, before, beforeLow, low},
		},
		{
			From(low).DownTo(empty),
			keySet{afterLow, within, beforeHigh, high, afterHigh, after},
			keySet{afterEmpty, before, beforeLow, low},
			keySet{empty},
		},
		{
			From(low).DownTo(nil),
			keySet{afterLow, within, beforeHigh, high, afterHigh, after},
			keySet{empty, afterEmpty, before, beforeLow, low},
			keySet{},
		},
		{
			From(empty).DownTo(nil),
			keySet{afterEmpty, before, beforeLow, low, afterLow, within, beforeHigh, high, afterHigh, after},
			keySet{empty},
			keySet{},
		},
	} {
		t.Run(tt.bounds.String(), func(t *testing.T) {
			t.Parallel()
			count := 0
			for _, key := range tt.before {
				assert.Equal(t, -1, tt.bounds.Compare(key), "%s", keyName(key))
				count++
			}
			for _, key := range tt.within {
				assert.Equal(t, 0, tt.bounds.Compare(key), "%s", keyName(key))
				count++
			}
			for _, key := range tt.after {
				assert.Equal(t, +1, tt.bounds.Compare(key), "%s", keyName(key))
				count++
			}
			assert.Equal(t, 11, count, "test case is missing keys")
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
		// forward bounds, begin[0] != end[0]
		{
			From(nil).To(empty),
			[]expectedChildBounds{
				{empty, 0, 0, false},
				{afterEmpty, 0, 0, false},
			},
		},
		{
			From(nil).To(afterEmpty),
			[]expectedChildBounds{
				{empty, 0, 0, true},
				{afterEmpty, 0, 0, false},
			},
		},
		{
			From(nil).To(low),
			[]expectedChildBounds{
				{empty, 0, 0x04, true},
				{afterEmpty, 0, 0xFF, true},
				{before, 0, 0xFF, true},
				{low[:1], 0, 0x99, true},
				{low[:2], 0, 0x72, true},
				{low[:3], 0, 0xD3, true},
				{beforeLow, 0, 0xFF, true},
				{low, 0, 0, false},
				{afterLow, 0, 0, false},
			},
		},
		{
			From(nil).To(nil),
			[]expectedChildBounds{
				{empty, 0, 0xFF, true},
				{afterEmpty, 0, 0xFF, true},
				{within, 0, 0xFF, true},
				{after, 0, 0xFF, true},
			},
		},
		{
			From(empty).To(low),
			[]expectedChildBounds{
				{empty, 0, 0x04, true},
				{afterEmpty, 0, 0xFF, true},
				{before, 0, 0xFF, true},
				{low[:1], 0, 0x99, true},
				{low[:2], 0, 0x72, true},
				{low[:3], 0, 0xD3, true},
				{beforeLow, 0, 0xFF, true},
				{low, 0, 0, false},
				{afterLow, 0, 0, false},
			},
		},
		{
			From(empty).To(nil),
			[]expectedChildBounds{
				{empty, 0, 0xFF, true},
				{afterEmpty, 0, 0xFF, true},
				{within, 0, 0xFF, true},
				{after, 0, 0xFF, true},
			},
		},
		{
			From(low).To(high),
			[]expectedChildBounds{
				{empty, 0x04, 0x42, true},
				{afterEmpty, 0, 0, false},
				{before, 0, 0, false},
				{low[:1], 0x99, 0xFF, true},
				{low[:2], 0x72, 0xFF, true},
				{low[:3], 0xD3, 0xFF, true},
				{beforeLow, 0, 0, false},
				{low, 0, 0xFF, true},
				{afterLow, 0, 0xFF, true},
				{within, 0, 0xFF, true},
				{high[:1], 0, 0x12, true},
				{high[:2], 0, 0xBC, true},
				{high[:3], 0, 0x60, true},
				{beforeHigh, 0, 0xFF, true},
				{high, 0, 0, false},
				{afterHigh, 0, 0, false},
				{after, 0, 0, false},
			},
		},
		{
			From(low).To(nil),
			[]expectedChildBounds{
				{empty, 0x04, 0xFF, true},
				{afterEmpty, 0, 0, false},
				{before, 0, 0, false},
				{low[:1], 0x99, 0xFF, true},
				{low[:2], 0x72, 0xFF, true},
				{low[:3], 0xD3, 0xFF, true},
				{beforeLow, 0, 0, false},
				{low, 0, 0xFF, true},
				{afterLow, 0, 0xFF, true},
				{after, 0, 0xFF, true},
			},
		},

		// reverse bounds, begin[0] != end[0]
		{
			From(nil).DownTo(low),
			[]expectedChildBounds{
				{after, 0xFF, 0, true},
				{within, 0xFF, 0, true},
				{afterLow, 0xFF, 0, true},
				{low, 0xFF, 0, true},
				{beforeLow, 0, 0, false},
				{low[:3], 0xFF, 0xD3, true},
				{low[:2], 0xFF, 0x72, true},
				{low[:1], 0xFF, 0x99, true},
				{before, 0, 0, false},
				{afterEmpty, 0, 0, false},
				{empty, 0xFF, 0x04, true},
			},
		},
		{
			From(nil).DownTo(empty),
			[]expectedChildBounds{
				{after, 0xFF, 0, true},
				{before, 0xFF, 0, true},
				{afterEmpty, 0xFF, 0, true},
				{empty, 0xFF, 0, true},
			},
		},
		{
			From(nil).DownTo(nil),
			[]expectedChildBounds{
				{after, 0xFF, 0, true},
				{before, 0xFF, 0, true},
				{afterEmpty, 0xFF, 0, true},
				{empty, 0xFF, 0, true},
			},
		},
		{
			From(high).DownTo(low),
			[]expectedChildBounds{
				{after, 0, 0, false},
				{afterHigh, 0, 0, false},
				{high, 0, 0, false},
				{beforeHigh, 0xFF, 0, true},
				{high[:3], 0x60, 0, true},
				{high[:2], 0xBC, 0, true},
				{high[:1], 0x12, 0, true},
				{within, 0xFF, 0, true},
				{afterLow, 0xFF, 0, true},
				{low, 0xFF, 0, true},
				{beforeLow, 0, 0, false},
				{low[:3], 0xFF, 0xD3, true},
				{low[:2], 0xFF, 0x72, true},
				{low[:1], 0xFF, 0x99, true},
				{before, 0, 0, false},
				{afterEmpty, 0, 0, false},
				{empty, 0x42, 0x04, true},
			},
		},
		{
			From(low).DownTo(empty),
			[]expectedChildBounds{
				{after, 0, 0, false},
				{within, 0, 0, false},
				{afterLow, 0, 0, false},
				{low, 0, 0, false},
				{beforeLow, 0xFF, 0, true},
				{low[:3], 0xD3, 0, true},
				{low[:2], 0x72, 0, true},
				{low[:1], 0x99, 0, true},
				{before, 0xFF, 0, true},
				{afterEmpty, 0xFF, 0, true},
				{empty, 0x04, 0, true},
			},
		},
		{
			From(low).DownTo(nil),
			[]expectedChildBounds{
				{after, 0, 0, false},
				{within, 0, 0, false},
				{afterLow, 0, 0, false},
				{low, 0, 0, false},
				{beforeLow, 0xFF, 0, true},
				{low[:3], 0xD3, 0, true},
				{low[:2], 0x72, 0, true},
				{low[:1], 0x99, 0, true},
				{before, 0xFF, 0, true},
				{afterEmpty, 0xFF, 0, true},
				{empty, 0x04, 0, true},
			},
		},
		{
			From(empty).DownTo(nil),
			[]expectedChildBounds{
				{after, 0, 0, false},
				{afterEmpty, 0, 0, false},
				{empty, 0, 0, false},
			},
		},

		// forward, common prefix
		{
			From(low).To(low2),
			[]expectedChildBounds{
				{empty, 0x04, 0x04, true},
				{afterEmpty, 0, 0, false},
				{before, 0, 0, false},
				{low[:1], 0x99, 0x99, true},
				{low[:2], 0x72, 0x9E, true},
				{low[:3], 0xD3, 0xFF, true},
				{beforeLow, 0, 0, false},
				{low, 0, 0xFF, true},
				{afterLow, 0, 0xFF, true},
				{low2[:3], 0, 0x27, true},
				{beforeLow2, 0, 0xFF, true},
				{low2, 0, 0, false},
				{afterLow2, 0, 0, false},
				{after, 0, 0, false},
			},
		},

		// reverse, common prefix
		{
			From(low2).DownTo(low),
			[]expectedChildBounds{
				{after, 0, 0, false},
				{afterLow2, 0, 0, false},
				{low2, 0, 0, false},
				{beforeLow2, 0xFF, 0, true},
				{low2[:3], 0x27, 0, true},
				{afterLow, 0xFF, 0, true},
				{low, 0xFF, 0, true},
				{beforeLow, 0, 0, false},
				{low[:3], 0xFF, 0xD3, true},
				{low[:2], 0x9E, 0x72, true},
				{low[:1], 0x99, 0x99, true},
				{before, 0, 0, false},
				{afterEmpty, 0, 0, false},
				{empty, 0x04, 0x04, true},
			},
		},

		// forward, From is a prefix of To
		{
			From(low[:2]).To(low),
			[]expectedChildBounds{
				{empty, 0x04, 0x04, true},
				{afterEmpty, 0, 0, false},
				{before, 0, 0, false},
				{low[:1], 0x99, 0x99, true},
				{low[:2], 0, 0x72, true},
				{low[:3], 0, 0xD3, true},
				{beforeLow, 0, 0xFF, true},
				{low, 0, 0, false},
				{afterLow, 0, 0, false},
				{after, 0, 0, false},
			},
		},

		// reverse, DownTo is a prefix of From
		{
			// {0x04, 0x99, 0x72, 0xD3} DOWN TO {0x04, 0x99}
			From(low).DownTo(low[:2]),
			[]expectedChildBounds{
				{after, 0, 0, false},
				{afterLow, 0, 0, false},
				{low, 0, 0, false},
				{beforeLow, 0xFF, 0, true},
				{low[:3], 0xD3, 0, true},
				{low[:2], 0x72, 0, true},
				{low[:1], 0x99, 0x99, true},
				{before, 0, 0, false},
				{afterEmpty, 0, 0, false},
				{empty, 0x04, 0x04, true},
			},
		},
	} {
		t.Run(tt.bounds.String(), func(t *testing.T) {
			t.Parallel()
			for _, exp := range tt.expected {
				start, stop, ok := btrie.TestingChildBounds(tt.bounds, exp.key)
				assert.Equal(t, exp.start, start, "%s", keyName(exp.key))
				assert.Equal(t, exp.stop, stop, "%s", keyName(exp.key))
				assert.Equal(t, exp.ok, ok, "%s", keyName(exp.key))
			}
		})
	}
}
