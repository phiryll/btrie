package btrie_test

import (
	"bytes"
	"fmt"
	"iter"
	"math/bits"
	"reflect"
	"slices"
	"testing"

	"github.com/phiryll/btrie"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

type (
	TestBTrie = btrie.Cloneable[byte]
	Bounds    = btrie.Bounds
	keySet    = [][]byte // instances will generally have unique keys

	implDef struct {
		name    string
		factory func() TestBTrie
	}

	// A description of a trie to be tested or benchmarked.
	// Some fields may not be populated depending on the use case.
	//
	// For benchmarking, there is one trieConfig with a given trieSize used by all benchmarks,
	// shared by all trie implementations.
	trieConfig struct {
		name string

		// The number of key/value pairs in tries generated by this config.
		// Also the size of entries.
		trieSize int

		// The key/value entries in tries generated by this config.
		entries map[string]byte

		// present/absent[i] = a set of keys of length i that are present/absent.
		// For denser tries, there may be no absent keys of length 0 or 1.
		present, absent []keySet

		// forward/reverse Bounds instances to test Range.
		forward, reverse []Bounds
	}

	// A trie created by a def.factory(), possibly with entries from config.
	// For some tests, an empty trie might be created and config might be nil.
	testTrie struct {
		name   string
		trie   TestBTrie
		def    *implDef
		config *trieConfig
	}

	// Used to test Range result sets.
	entry struct {
		key   []byte
		value byte
	}
)

const (
	zero = byte(0)

	// The maximum size of created absent and bounds slices, 64K.
	maxGenSize = 1 << 16

	// The max key length from presentKeys, absentKeys, and nearKeys.
	maxTestKeySize = 3
)

var (
	implDefs = []*implDef{
		// fuzz tests assume reference is first
		{"reference", newReference},
		{"pointer-trie", asCloneable(btrie.NewPointerTrie[byte])},
	}

	From       = btrie.From
	forwardAll = From(nil).To(nil)
	reverseAll = From(nil).DownTo(nil)

	keyName = btrie.TestingKeyName

	// Keys used to build test tries.
	// These are in lexicographical order.
	presentTestKeys = keySet{
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

	// Non-empty keys very near presentTestKeys, but not in presentTestKeys.
	absentTestKeys = keySet{
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

	// presentTestKeys + absentTestKeys + +/-Inf.
	// Except for the nils, these are in lexicographical order.
	nearTestKeys = keySet{
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

	testTrieConfigs = createTestTrieConfigs()
)

func asCloneable(factory func() btrie.BTrie[byte]) func() TestBTrie {
	return func() TestBTrie {
		cloneable, ok := factory().(btrie.Cloneable[byte])
		if !ok {
			panic("not Cloneable")
		}
		return cloneable
	}
}

func emptySeqInt(_ func(int) bool) {}

func emptyAdjInt(_ int) iter.Seq[int] {
	return emptySeqInt
}

func emptyPathAdjInt(_ []int) iter.Seq[int] {
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

// trieConfigs for all possible subsequences of presentKeys.
func createTestTrieConfigs() []*trieConfig {
	result := []*trieConfig{}

	// Every trieConfig here gets the same set of test Bounds.
	var forward, reverse []Bounds
	for i, low := range nearTestKeys {
		for _, high := range nearTestKeys[i+1:] {
			forward = append(forward, From(low).To(high))
			reverse = append(reverse, From(high).DownTo(low))
		}
	}

	// Every bit pattern of i defines which keys are present in that config.
	for i := range 1 << len(presentTestKeys) {
		config := trieConfig{
			fmt.Sprintf("sub-trie=%0*b", len(presentTestKeys), i),
			bits.OnesCount(uint(i)),
			map[string]byte{},
			make([]keySet, maxTestKeySize+1),
			make([]keySet, maxTestKeySize+1),
			forward,
			reverse,
		}
		mask := 0x01
		for k, key := range presentTestKeys {
			keySize := len(key)
			if i&mask != 0 {
				config.entries[string(key)] = byte(k)
				config.present[keySize] = append(config.present[keySize], key)
			} else {
				config.absent[keySize] = append(config.absent[keySize], key)
			}
			mask <<= 1
		}
		for _, key := range absentTestKeys {
			keySize := len(key)
			config.absent[keySize] = append(config.absent[keySize], key)
		}
		result = append(result, &config)
	}
	return result
}

func TestTestTrieConfigRepeatability(t *testing.T) {
	t.Parallel()
	for i, config := range createTestTrieConfigs() {
		assert.True(t, reflect.DeepEqual(testTrieConfigs[i], config))
	}
}

func createReferenceTrie(config *trieConfig) TestBTrie {
	trie := newReference()
	for k, v := range config.entries {
		trie.Put([]byte(k), v)
	}
	return trie
}

func createTestTries(trieConfigs []*trieConfig) []*testTrie {
	result := []*testTrie{}
	for _, def := range implDefs {
		for _, config := range trieConfigs {
			trie := def.factory()
			for k, v := range config.entries {
				trie.Put([]byte(k), v)
			}
			name := fmt.Sprintf("impl=%s/%s", def.name, config.name)
			result = append(result, &testTrie{name, trie, def, config})
		}
	}
	return result
}

/*
func assertPresent(t *testing.T, key []byte, value byte, trie TestBTrie) {
	actual, ok := trie.Get(key)
	assert.True(t, ok)
	assert.Equal(t, value, actual)
	for k, v := range trie.Range(forwardAll) {
		if bytes.Equal(key, k) {
			assert.Equal(t, value, v)
			break
		}
	}
	for k, v := range trie.Range(reverseAll) {
		if bytes.Equal(key, k) {
			assert.Equal(t, value, v)
			break
		}
	}
}
*/

func assertAbsent(t *testing.T, key []byte, trie TestBTrie) {
	actual, ok := trie.Get(key)
	assert.False(t, ok)
	assert.Equal(t, zero, actual)
	actual, ok = trie.Delete(key)
	assert.False(t, ok)
	assert.Equal(t, zero, actual)
	for k := range trie.Range(forwardAll) {
		assert.NotEqual(t, key, k)
	}
	for k := range trie.Range(reverseAll) {
		assert.NotEqual(t, key, k)
	}
}

// Test that trie contains only the key/value pairs in entries,
// and that Range(forward/reverse) returns them in the correct order.
func assertSame(t *testing.T, entries map[string]byte, trie TestBTrie) {
	sliceEntries := []entry{}
	for key, expected := range entries {
		actual, ok := trie.Get([]byte(key))
		assert.True(t, ok)
		assert.Equal(t, expected, actual)
		sliceEntries = append(sliceEntries, entry{[]byte(key), expected})
	}
	slices.SortFunc(sliceEntries, cmpEntryForward)
	assert.Equal(t, sliceEntries, collect(trie.Range(forwardAll)))
	slices.SortFunc(sliceEntries, cmpEntryReverse)
	assert.Equal(t, sliceEntries, collect(trie.Range(reverseAll)))
}

func TestNilArgPanics(t *testing.T) {
	t.Parallel()
	for _, def := range implDefs {
		t.Run(def.name, func(t *testing.T) {
			t.Parallel()
			trie := def.factory()
			assert.Panics(t, func() {
				trie.Put(nil, 0)
			})
			assert.Panics(t, func() {
				trie.Get(nil)
			})
			assert.Panics(t, func() {
				trie.Delete(nil)
			})
			assert.Panics(t, func() {
				trie.Range(nil)
			})
		})
	}
}

// Tests Get/Put/Delete/Range with a specific key and trie, which should not contain key.
// The trie should be the same after invoking this function.
// Assumes trie.Range(forwardAll) works.
func testKey(t *testing.T, key []byte, trie TestBTrie) {
	const value = byte(43)
	const replacement = byte(57)
	existing := map[string]byte{}
	for k, v := range trie.Range(forwardAll) {
		existing[string(k)] = v
	}
	require.NotContains(t, existing, string(key))

	assertAbsent(t, key, trie)
	assertSame(t, existing, trie)

	actual, ok := trie.Put(key, value)
	assert.False(t, ok)
	assert.Equal(t, zero, actual)
	existing[string(key)] = value
	assertSame(t, existing, trie)

	actual, ok = trie.Put(key, replacement)
	assert.True(t, ok)
	assert.Equal(t, value, actual)
	existing[string(key)] = replacement
	assertSame(t, existing, trie)

	actual, ok = trie.Delete(key)
	assert.True(t, ok)
	assert.Equal(t, replacement, actual)
	assertAbsent(t, key, trie)
	delete(existing, string(key))
	assertSame(t, existing, trie)
}

// The empty key is often a special case in an implementation.
func TestEmptyKey(t *testing.T) {
	t.Parallel()
	key := []byte{}
	for _, def := range implDefs {
		t.Run(def.name, func(t *testing.T) {
			t.Parallel()
			trie := def.factory()
			testKey(t, key, trie)
			trie.Put([]byte{43, 15}, 94)
			trie.Put([]byte{126, 73, 12}, 45)
			testKey(t, key, trie)
		})
	}
}

func TestTrie(t *testing.T) {
	t.Parallel()
	for _, test := range createTestTries(testTrieConfigs) {
		t.Run(test.name, func(t *testing.T) {
			t.Parallel()

			// Build the trie, testing along the way.
			trie := test.def.factory()
			existing := map[string]byte{}
			for key, value := range test.config.entries {
				t.Run("op=put/key="+keyName([]byte(key)), func(t *testing.T) {
					assertAbsent(t, []byte(key), trie)
					assertSame(t, existing, trie)

					actual, ok := trie.Put([]byte(key), value)
					assert.False(t, ok)
					assert.Equal(t, zero, actual)
					existing[key] = value
					assertSame(t, existing, trie)
				})
			}

			for _, keys := range test.config.absent {
				for _, key := range keys {
					t.Run("op=absent/key="+keyName(key), func(t *testing.T) {
						assertAbsent(t, key, trie)
					})
				}
			}

			t.Run("op=range", func(t *testing.T) {
				ref := createReferenceTrie(test.config)
				for _, bounds := range test.config.forward {
					assert.Equal(t, collect(ref.Range(bounds)), collect(trie.Range(bounds)),
						"%s", bounds)
				}
				for _, bounds := range test.config.reverse {
					assert.Equal(t, collect(ref.Range(bounds)), collect(trie.Range(bounds)),
						"%s", bounds)
				}
			})
		})
	}
}

//nolint:gocognit
func TestClone(t *testing.T) {
	t.Parallel()
	for _, test := range createTestTries(testTrieConfigs) {
		t.Run(test.name, func(t *testing.T) {
			t.Parallel()
			original := test.trie
			assertSame(t, test.config.entries, original)

			// test that the clone was correct
			trie := original.Clone()
			assertSame(t, test.config.entries, trie)

			// mutate the clone and test that original hasn't changed
			for key := range test.config.entries {
				trie.Delete([]byte(key))
			}
			assertSame(t, map[string]byte{}, trie)
			for _, keys := range test.config.absent {
				for i, key := range keys {
					trie.Put(key, byte(i))
				}
			}
			assertSame(t, test.config.entries, original)

			// mutate the original and test that the clone hasn't changed
			trie = original.Clone()
			for key := range test.config.entries {
				original.Delete([]byte(key))
			}
			assertSame(t, map[string]byte{}, original)
			for _, keys := range test.config.absent {
				for i, key := range keys {
					original.Put(key, byte(i))
				}
			}
			assertSame(t, test.config.entries, trie)
		})
	}
}

// Things that failed at one point or another during testing.

func testFail1(t *testing.T, factory func() TestBTrie) {
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

func testFail2(t *testing.T, factory func() TestBTrie) {
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

func testFail3(t *testing.T, factory func() TestBTrie) {
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

func testFail4(t *testing.T, factory func() TestBTrie) {
	t.Run("fail 4", func(t *testing.T) {
		t.Parallel()
		trie := factory()
		trie.Put([]byte{0x50, 0xEF}, 45)
		assert.Equal(t,
			[]entry{},
			collect(trie.Range(From([]byte{0x50}).DownTo([]byte{0x15}))))
	})
}

func testFail5(t *testing.T, factory func() TestBTrie) {
	t.Run("fail 5", func(t *testing.T) {
		t.Parallel()
		trie := factory()
		trie.Put([]byte{0x50, 0xEF}, 45)
		assert.Equal(t,
			[]entry{{[]byte{0x50, 0xEF}, 45}},
			collect(trie.Range(From([]byte{0xFD}).DownTo([]byte{0x3D}))))
	})
}

func testFail6(t *testing.T, factory func() TestBTrie) {
	t.Run("fail 6", func(t *testing.T) {
		t.Parallel()
		trie := factory()
		trie.Put([]byte{0x50, 0xEF}, 45)
		assert.Equal(t,
			[]entry{{[]byte{0x50, 0xEF}, 45}},
			collect(trie.Range(From([]byte{0x51}).DownTo([]byte{0x50}))))
	})
}

func testFail7(t *testing.T, factory func() TestBTrie) {
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

func TestPastFailures(t *testing.T) {
	t.Parallel()
	for _, def := range implDefs {
		factory := def.factory
		t.Run(def.name, func(t *testing.T) {
			t.Parallel()
			testFail1(t, factory)
			testFail2(t, factory)
			testFail3(t, factory)
			testFail4(t, factory)
			testFail5(t, factory)
			testFail6(t, factory)
			testFail7(t, factory)
		})
	}
}
