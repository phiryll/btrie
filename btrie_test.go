package btrie_test

import (
	"bytes"
	"iter"
	"testing"

	"github.com/phiryll/btrie"
	"github.com/stretchr/testify/assert"
)

type (
	Obm    = btrie.OrderedBytesMap[byte]
	Bounds = btrie.Bounds

	entry struct {
		key   []byte
		value byte
	}
)

var (
	From = btrie.From
)

func emptySeqInt(_ func(int) bool) {}

func emptyAdjInt(_ []int) iter.Seq[int] {
	return emptySeqInt
}

func collect(itr iter.Seq2[[]byte, byte]) []entry {
	entries := []entry{}
	for k, v := range itr {
		entries = append(entries, entry{k, v})
	}
	return entries
}

func cmpEntryForward(a, b entry) int {
	return bytes.Compare(a.key, b.key)
}

func cmpEntryReverse(a, b entry) int {
	return bytes.Compare(b.key, a.key)
}

// Tests both forward and reverse bounds.
func assertEqualRanges(t *testing.T, expected, actual Obm, bounds Bounds) {
	assert.Equal(t, collect(expected.Range(bounds)), collect(actual.Range(bounds)),
		"%s", bounds)
	var reverse Bounds
	if bounds.IsReverse() {
		reverse = From(bounds.End()).To(bounds.Begin())
	} else {
		reverse = From(bounds.End()).DownTo(bounds.Begin())
	}
	assert.Equal(t, collect(expected.Range(reverse)), collect(actual.Range(reverse)),
		"%s", bounds)
}

func TestReference(t *testing.T) {
	t.Parallel()
	// TODO: invoke shared non-random testier.
}

func TestSimple(t *testing.T) {
	t.Parallel()
	// TODO: invoke shared non-random testier.
}

// Things that failed at one point or another during testing.

func TestFail1(t *testing.T) {
	t.Parallel()
	bt := btrie.NewSimple[byte]()
	bt.Put([]byte{5}, 0)
	assert.Equal(t,
		[]entry{},
		collect(bt.Range(From([]byte{5, 0}).To([]byte{6}))))
	assert.Equal(t,
		[]entry{{[]byte{5}, 0}},
		collect(bt.Range(From([]byte{4}).To([]byte{5, 0}))))
}

func TestFail2(t *testing.T) {
	t.Parallel()
	bt := btrie.NewSimple[byte]()
	bt.Put([]byte{0xB3, 0x9C}, 184)

	// forgot to check isTerminal
	actual, actualOk := bt.Get([]byte{0xB3})
	assert.False(t, actualOk)
	assert.Equal(t, byte(0), actual)

	actual, actualOk = bt.Get([]byte{0xB3, 0x9C})
	assert.True(t, actualOk)
	assert.Equal(t, byte(184), actual)
}

func TestFail3(t *testing.T) {
	t.Parallel()
	bt := btrie.NewSimple[byte]()
	bt.Put([]byte{0xB3, 0x9C}, 184)

	actual, actualOk := bt.Delete([]byte{0xB3})
	assert.False(t, actualOk)
	assert.Equal(t, byte(0), actual)

	// Make sure the subtree wasn't deleted.
	actual, actualOk = bt.Get([]byte{0xB3, 0x9C})
	assert.True(t, actualOk)
	assert.Equal(t, byte(184), actual)
}

func TestFail4(t *testing.T) {
	t.Parallel()
	bt := btrie.NewSimple[byte]()
	bt.Put([]byte{0x50, 0xEF}, 45)
	assert.Equal(t,
		[]entry{},
		collect(bt.Range(From([]byte{0x50}).DownTo([]byte{0x15}))))
}

func TestFail5(t *testing.T) {
	t.Parallel()
	bt := btrie.NewSimple[byte]()
	bt.Put([]byte{0x50, 0xEF}, 45)
	assert.Equal(t,
		[]entry{{key: []byte{0x50, 0xEF}, value: 45}},
		collect(bt.Range(From([]byte{0xFD}).DownTo([]byte{0x3D}))))
}

func TestFail6(t *testing.T) {
	t.Parallel()
	bt := btrie.NewSimple[byte]()
	bt.Put([]byte{0x50, 0xEF}, 45)
	assert.Equal(t,
		[]entry{{key: []byte{0x50, 0xEF}, value: 45}},
		collect(bt.Range(From([]byte{0x51}).DownTo([]byte{0x50}))))
}

func TestFail7(t *testing.T) {
	// Failure is due to continuing iteration past false yield().
	// Failure requires the second Put.
	t.Parallel()
	bt := btrie.NewSimple[byte]()
	bt.Put([]byte{3}, 0)
	bt.Put([]byte{4}, 0)
	assert.Equal(t,
		[]entry{},
		collect(bt.Range(From([]byte{1}).To([]byte{2}))))
}
