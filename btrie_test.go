package btrie_test

import (
	"bytes"
	"fmt"
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

	trieTestCase struct {
		name       string
		present    [][]byte
		complement [][]byte
	}
)

const zero = byte(0)

var (
	From    = btrie.From
	keyName = btrie.TestingKeyName

	// Things that failed for some implementation during development.
	testFailures = []func(*testing.T, func() Obm){
		testFail1,
		testFail2,
		testFail3,
		testFail4,
		testFail5,
		testFail6,
		testFail7,
	}

	// Keys used to build test tries.
	// These are in lexicographical order.
	presentKeys = [][]byte{
		{},
		{0},
		{0x23},
		{0x23, 0},
		{0x23, 0xA5},
		{0x23, 0xA6},
		{0xC5},
		{0xC5, 0},
		{0xC5, 0x42},
		{0xC5, 0x43},
	}

	// Non-empty keys very near presentKeys, but not in presentKeys.
	absentKeys = [][]byte{
		{0, 0},
		{0x22, 0xFF},
		{0x23, 0, 0},
		{0x23, 0xA4, 0xFF},
		{0x23, 0xA5, 0},
		{0x23, 0xA5, 0xFF},
		{0x23, 0xA6, 0},
		{0xC4, 0xFF},
		{0xC5, 0, 0},
		{0xC5, 0x41, 0xFF},
		{0xC5, 0x42, 0},
		{0xC5, 0x42, 0xFF},
		{0xC5, 0x43, 0},
	}

	// Keys used to test tries built with presentKeys: presentKeys + absentKeys _ +/-Inf.
	// Except for the nils, these are in lexicographical order.
	nearKeys = [][]byte{
		nil, // -Inf
		{},
		{0},
		{0, 0},
		{0x22, 0xFF},
		{0x23},
		{0x23, 0},
		{0x23, 0, 0},
		{0x23, 0xA4, 0xFF},
		{0x23, 0xA5},
		{0x23, 0xA5, 0},
		{0x23, 0xA5, 0xFF},
		{0x23, 0xA6},
		{0x23, 0xA6, 0},
		{0xC4, 0xFF},
		{0xC5},
		{0xC5, 0},
		{0xC5, 0, 0},
		{0xC5, 0x41, 0xFF},
		{0xC5, 0x42},
		{0xC5, 0x42, 0},
		{0xC5, 0x42, 0xFF},
		{0xC5, 0x43},
		{0xC5, 0x43, 0},
		nil, // +Inf
	}
)

// Returns a sequence of all possible subsequences of presentKeys and their complements.
func trieTestCases() iter.Seq[trieTestCase] {
	size := len(presentKeys)
	limit := 1 << len(presentKeys)
	return func(yield func(trieTestCase) bool) {
		for i := range limit {
			present := [][]byte{}
			complement := [][]byte{}
			mask := 0x01
			for j := range size {
				if i&mask != 0 {
					present = append(present, presentKeys[j])
				} else {
					complement = append(complement, presentKeys[j])
				}
				mask <<= 1
			}
			if !yield(trieTestCase{fmt.Sprintf("sub-trie: %0*b", size, i), present, complement}) {
				return
			}
		}
	}
}

func buildTestBounds() []Bounds {
	testCases := []Bounds{}
	for i, low := range nearKeys {
		for _, high := range nearKeys[i+1:] {
			testCases = append(testCases,
				From(low).To(high),
				From(high).DownTo(low))
		}
	}
	return testCases
}

func emptySeqInt(_ func(int) bool) {}

func emptyAdjInt(_ []int) iter.Seq[int] {
	return emptySeqInt
}

func cmpEntryForward(a, b entry) int {
	return bytes.Compare(a.key, b.key)
}

func cmpEntryReverse(a, b entry) int {
	return bytes.Compare(b.key, a.key)
}

func collect(itr iter.Seq2[[]byte, byte]) []entry {
	entries := []entry{}
	for k, v := range itr {
		entries = append(entries, entry{k, v})
	}
	return entries
}

func testOneKey(t *testing.T, factory func() Obm, key []byte) {
	t.Run("key: "+keyName(key), func(t *testing.T) {
		trie := factory()

		// Get/Delete/Range on empty trie
		actual, ok := trie.Get(key)
		assert.False(t, ok)
		assert.Equal(t, zero, actual)
		actual, ok = trie.Delete(key)
		assert.False(t, ok)
		assert.Equal(t, zero, actual)
		assert.Empty(t, collect(trie.Range(From(nil).To(nil))))
		assert.Empty(t, collect(trie.Range(From(nil).DownTo(nil))))

		// Put
		actual, ok = trie.Put(key, 19)
		assert.False(t, ok)
		assert.Equal(t, zero, actual)
		actual, ok = trie.Get(key)
		assert.True(t, ok)
		assert.Equal(t, byte(19), actual)
		assert.Equal(t,
			[]entry{{key, 19}},
			collect(trie.Range(From(nil).To(nil))))
		assert.Equal(t,
			[]entry{{key, 19}},
			collect(trie.Range(From(nil).DownTo(nil))))

		// Put a replacement
		actual, ok = trie.Put(key, 43)
		assert.True(t, ok)
		assert.Equal(t, byte(19), actual)
		actual, ok = trie.Get(key)
		assert.True(t, ok)
		assert.Equal(t, byte(43), actual)
		assert.Equal(t,
			[]entry{{key, 43}},
			collect(trie.Range(From(nil).To(nil))))
		assert.Equal(t,
			[]entry{{key, 43}},
			collect(trie.Range(From(nil).DownTo(nil))))

		// Delete
		actual, ok = trie.Delete(key)
		assert.True(t, ok)
		assert.Equal(t, byte(43), actual)
		actual, ok = trie.Get(key)
		assert.False(t, ok)
		assert.Equal(t, zero, actual)
		actual, ok = trie.Delete(key)
		assert.False(t, ok)
		assert.Equal(t, zero, actual)
		assert.Empty(t, collect(trie.Range(From(nil).To(nil))))
		assert.Empty(t, collect(trie.Range(From(nil).DownTo(nil))))
	})
}

func testTrieTestCase(t *testing.T, factory func() Obm, tt *trieTestCase) {
	t.Run(tt.name, func(t *testing.T) {
		trie := factory()

		// Build the trie, testing along the way.
		for i, key := range tt.present {
			value := byte(i)

			actual, ok := trie.Get(key)
			assert.False(t, ok, "%s", keyName(key))
			assert.Equal(t, zero, actual, "%s", keyName(key))

			actual, ok = trie.Delete(key)
			assert.False(t, ok, "%s", keyName(key))
			assert.Equal(t, zero, actual, "%s", keyName(key))

			actual, ok = trie.Put(key, value)
			assert.False(t, ok, "%s", keyName(key))
			assert.Equal(t, zero, actual, "%s", keyName(key))

			actual, ok = trie.Get(key)
			assert.True(t, ok, "%s", keyName(key))
			assert.Equal(t, value, actual, "%s", keyName(key))
		}

		// Make sure keys that should be absent are.
		for _, key := range tt.complement {
			actual, ok := trie.Get(key)
			assert.False(t, ok, "%s", keyName(key))
			assert.Equal(t, zero, actual, "%s", keyName(key))

			actual, ok = trie.Delete(key)
			assert.False(t, ok, "%s", keyName(key))
			assert.Equal(t, zero, actual, "%s", keyName(key))
		}
		for _, key := range absentKeys {
			actual, ok := trie.Get(key)
			assert.False(t, ok, "%s", keyName(key))
			assert.Equal(t, zero, actual, "%s", keyName(key))

			actual, ok = trie.Delete(key)
			assert.False(t, ok, "%s", keyName(key))
			assert.Equal(t, zero, actual, "%s", keyName(key))
		}

		// Test Range.
		ref := newReference()
		for i, key := range tt.present {
			ref.Put(key, byte(i))
		}
		for _, bounds := range buildTestBounds() {
			assert.Equal(t, collect(ref.Range(bounds)), collect(trie.Range(bounds)),
				"%s", bounds)
		}
	})
}

// Things that failed at one point or another during testing.

func testFail1(t *testing.T, factory func() Obm) {
	t.Run("fail 1", func(t *testing.T) {
		t.Parallel()
		trie := factory()
		trie.Put([]byte{5}, 0)
		assert.Equal(t,
			[]entry{},
			collect(trie.Range(From([]byte{5, 0}).To([]byte{6}))))
		assert.Equal(t,
			[]entry{{[]byte{5}, 0}},
			collect(trie.Range(From([]byte{4}).To([]byte{5, 0}))))
	})
}

func testFail2(t *testing.T, factory func() Obm) {
	t.Run("fail 2", func(t *testing.T) {
		t.Parallel()
		trie := factory()
		trie.Put([]byte{0xB3, 0x9C}, 184)

		// forgot to check isTerminal
		actual, actualOk := trie.Get([]byte{0xB3})
		assert.False(t, actualOk)
		assert.Equal(t, byte(0), actual)

		actual, actualOk = trie.Get([]byte{0xB3, 0x9C})
		assert.True(t, actualOk)
		assert.Equal(t, byte(184), actual)
	})
}

func testFail3(t *testing.T, factory func() Obm) {
	t.Run("fail 3", func(t *testing.T) {
		t.Parallel()
		trie := factory()
		trie.Put([]byte{0xB3, 0x9C}, 184)

		actual, actualOk := trie.Delete([]byte{0xB3})
		assert.False(t, actualOk)
		assert.Equal(t, byte(0), actual)

		// Make sure the subtree wasn't deleted.
		actual, actualOk = trie.Get([]byte{0xB3, 0x9C})
		assert.True(t, actualOk)
		assert.Equal(t, byte(184), actual)
	})
}

func testFail4(t *testing.T, factory func() Obm) {
	t.Run("fail 4", func(t *testing.T) {
		t.Parallel()
		trie := factory()
		trie.Put([]byte{0x50, 0xEF}, 45)
		assert.Equal(t,
			[]entry{},
			collect(trie.Range(From([]byte{0x50}).DownTo([]byte{0x15}))))
	})
}

func testFail5(t *testing.T, factory func() Obm) {
	t.Run("fail 5", func(t *testing.T) {
		t.Parallel()
		trie := factory()
		trie.Put([]byte{0x50, 0xEF}, 45)
		assert.Equal(t,
			[]entry{{[]byte{0x50, 0xEF}, 45}},
			collect(trie.Range(From([]byte{0xFD}).DownTo([]byte{0x3D}))))
	})
}

func testFail6(t *testing.T, factory func() Obm) {
	t.Run("fail 6", func(t *testing.T) {
		t.Parallel()
		trie := factory()
		trie.Put([]byte{0x50, 0xEF}, 45)
		assert.Equal(t,
			[]entry{{[]byte{0x50, 0xEF}, 45}},
			collect(trie.Range(From([]byte{0x51}).DownTo([]byte{0x50}))))
	})
}

func testFail7(t *testing.T, factory func() Obm) {
	// Failure is due to continuing iteration past false yield().
	// Failure requires the second Put.
	t.Run("fail 7", func(t *testing.T) {
		t.Parallel()
		trie := factory()
		trie.Put([]byte{3}, 0)
		trie.Put([]byte{4}, 0)
		assert.Equal(t,
			[]entry{},
			collect(trie.Range(From([]byte{1}).To([]byte{2}))))
	})
}

func testOrderedBytesMap(t *testing.T, factory func() Obm) {
	for _, failure := range testFailures {
		failure(t, factory)
	}
	testOneKey(t, factory, []byte{})
	testOneKey(t, factory, []byte{18, 43, 247, 14})
	for tt := range trieTestCases() {
		testTrieTestCase(t, factory, &tt)
	}
}

func TestReference(t *testing.T) {
	t.Parallel()
	testOrderedBytesMap(t, newReference)
}

func TestSimple(t *testing.T) {
	t.Parallel()
	testOrderedBytesMap(t, btrie.NewSimple[byte])
}
