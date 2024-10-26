package btrie_test

import (
	"bytes"
	"math/bits"
	"math/rand"
	"testing"
	"time"

	"github.com/phiryll/btrie"
	"github.com/stretchr/testify/assert"
)

const fuzzKeyLength = 4

var newPointerTrie = asCloneable(btrie.NewPointerTrie[byte])

// Fuzz testing is very parallel, and tries aren't generally thread-safe.
// rand.Rand instances are also not thread-safe.

func randomBytes(n int, random *rand.Rand) []byte {
	b := make([]byte, n)
	_, _ = random.Read(b)
	return b
}

func randomByte(random *rand.Rand) byte {
	b := []byte{0}
	_, _ = random.Read(b)
	return b[0]
}

// Returns a random key length with distribution:
//
//	50% of maxLength
//	25% of maxLength-1
//	...
//	2 of length 2
//	1 of length 1
//	1 of length 0
func randomKeyLength(maxLength int, random *rand.Rand) int {
	return bits.Len(uint(random.Intn(1 << maxLength)))
}

func randomKey(maxLength int, random *rand.Rand) []byte {
	return randomBytes(randomKeyLength(maxLength, random), random)
}

func trimKey(key []byte) []byte {
	// Independent random source for this, don't actually want repeatable.
	random := rand.New(rand.NewSource(time.Now().UnixNano()))
	keyLen := randomKeyLength(fuzzKeyLength, random)
	if len(key) < keyLen {
		return key
	}
	return key[:keyLen]
}

// Puts n random entries in trie.
func putEntries(trie TestBTrie, n int, seed int64) {
	random := rand.New(rand.NewSource(seed))
	for range n {
		key := randomKey(fuzzKeyLength, random)
		trie.Put(key, randomByte(random))
	}
}

// Returns new instances of the same TestBTries every time.
// This is so the fuzzing engine gets predictable repeat behavior.
//
//nolint:nonamedreturns
func getFuzzBaseline(factory func() TestBTrie) (ref, trie TestBTrie) {
	const putCount = 1 << 20
	const seed = 483738
	ref = newReference()
	trie = factory()
	putEntries(ref, putCount, seed)
	putEntries(trie, putCount, seed)
	return ref, trie
}

func TestBaseline(t *testing.T) {
	t.Parallel()
	ref, trie := getFuzzBaseline(newPointerTrie)
	bounds := From(nil).To(nil)
	assert.Equal(t, collect(ref.Range(bounds)), collect(trie.Range(bounds)),
		"%s", bounds)
	bounds = From(nil).DownTo(nil)
	assert.Equal(t, collect(ref.Range(bounds)), collect(trie.Range(bounds)),
		"%s", bounds)
}

func fuzzGet(f *testing.F, factory func() TestBTrie) {
	ref, trie := getFuzzBaseline(factory)
	f.Fuzz(func(t *testing.T, key []byte) {
		key = trimKey(key)
		actual, actualOk := trie.Get(key)
		expected, expectedOk := ref.Get(key)
		assert.Equal(t, expectedOk, actualOk, "Get %s", keyName(key))
		assert.Equal(t, expected, actual, "Get %s", keyName(key))
	})
}

func fuzzPut(f *testing.F, factory func() TestBTrie) {
	f.Fuzz(func(t *testing.T, key []byte, value byte) {
		key = trimKey(key)
		ref, trie := getFuzzBaseline(factory)
		actual, actualOk := trie.Put(key, value)
		expected, expectedOk := ref.Put(key, value)
		assert.Equal(t, expectedOk, actualOk, "Put %s:%d", keyName(key), value)
		assert.Equal(t, expected, actual, "Put %s:%d", keyName(key), value)
		actual, ok := trie.Get(key)
		assert.True(t, ok, "Put %s:%d", keyName(key), value)
		assert.Equal(t, value, actual, "Put %s:%d", keyName(key), value)
	})
}

func fuzzDelete(f *testing.F, factory func() TestBTrie) {
	f.Fuzz(func(t *testing.T, key []byte) {
		key = trimKey(key)
		ref, trie := getFuzzBaseline(factory)
		actual, actualOk := trie.Delete(key)
		expected, expectedOk := ref.Delete(key)
		assert.Equal(t, expectedOk, actualOk, "Delete %s", keyName(key))
		assert.Equal(t, expected, actual, "Delete %s", keyName(key))
		actual, ok := trie.Get(key)
		assert.False(t, ok, "Delete %s", keyName(key))
		assert.Equal(t, byte(0), actual, "Delete %s", keyName(key))
	})
}

func fuzzRange(f *testing.F, factory func() TestBTrie) {
	ref, trie := getFuzzBaseline(factory)
	f.Fuzz(func(t *testing.T, begin, end []byte) {
		begin = trimKey(begin)
		end = trimKey(end)
		cmp := bytes.Compare(begin, end)
		if cmp == 0 {
			end = append(end, 0)
		} else if cmp > 0 {
			begin, end = end, begin
		}
		for _, bounds := range []Bounds{
			From(begin).To(end),
			From(end).DownTo(begin),
		} {
			actual := collect(trie.Range(bounds))
			expected := collect(ref.Range(bounds))
			assert.Equal(t, expected, actual, "%s", bounds)
		}
	})
}

func FuzzGetPointerTrie(f *testing.F) {
	fuzzGet(f, newPointerTrie)
}

func FuzzPutPointerTrie(f *testing.F) {
	fuzzPut(f, newPointerTrie)
}

func FuzzDeletePointerTrie(f *testing.F) {
	fuzzDelete(f, newPointerTrie)
}

func FuzzRangePointerTrie(f *testing.F) {
	fuzzRange(f, newPointerTrie)
}
