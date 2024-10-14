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
		Key   []byte
		Value byte
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
	return bytes.Compare(a.Key, b.Key)
}

func cmpEntryReverse(a, b entry) int {
	return bytes.Compare(b.Key, a.Key)
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

// TODO: every possible subtree shape (distinct first key bytes)
// depth <= 4 (including root), width <= 3, is/isNot terminal per node
// max trie size = 121 nodes, but not all subsets are possible.
// For example, the root (empty key) must be present,
// even if it has no children and is not terminal.
// Include the empty trie.

// 1*3  + 1 = 4
// 4*3  + 1 = 13
// 13*3 + 1 = 40
// 40*3 + 1 = 121

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
		[]entry{{Key: []byte{0x50, 0xEF}, Value: 45}},
		collect(bt.Range(From([]byte{0xFD}).DownTo([]byte{0x3D}))))
}

func TestFail6(t *testing.T) {
	t.Parallel()
	bt := btrie.NewSimple[byte]()
	bt.Put([]byte{0x50, 0xEF}, 45)
	assert.Equal(t,
		[]entry{{Key: []byte{0x50, 0xEF}, Value: 45}},
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
