package btrie_test

import (
	"bytes"
	"fmt"
	"math/bits"
	"math/rand"
	"testing"
	"time"

	"github.com/phiryll/btrie"
	"github.com/stretchr/testify/assert"
)

// Fuzz testing is very parallel, and tries aren't thread-safe yet.
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

func randomKeyLength(random *rand.Rand) int {
	return bits.Len(uint(random.Intn(1 << 8)))
}

func randomKey(random *rand.Rand) []byte {
	return randomBytes(randomKeyLength(random), random)
}

func trimKey(key []byte) []byte {
	// Independent random source for this, don't actually want repeatable.
	random := rand.New(rand.NewSource(time.Now().UnixNano()))
	keyLen := randomKeyLength(random)
	if len(key) < keyLen {
		return key
	}
	return key[:keyLen]
}

// Puts n random entries in obm.
func putEntries(obm Obm, n int, seed int64) {
	random := rand.New(rand.NewSource(seed))
	for range n {
		obm.Put(randomKey(random), randomByte(random))
	}
}

// TODO: Different initial states, like empty, a single entry, 0, FF, ...

// Returns new instances of the same OBMs every time.
// This is so the fuzzing engine gets predictable repeat behavior.
//
//nolint:nonamedreturns
func getBaseline(factory func() Obm) (ref, bt Obm) {
	const putCount = 10000
	const seed = 483738
	ref = newReference()
	bt = factory()
	putEntries(ref, putCount, seed)
	putEntries(bt, putCount, seed)
	return ref, bt
}

func TestBaseline(t *testing.T) {
	t.Parallel()
	ref, bt := getBaseline(btrie.NewSimple[byte])
	assert.Equal(t, collect(ref.Range(all)), collect(bt.Range(all)))
	assert.Equal(t, collect(ref.Range(reverseAll)), collect(bt.Range(reverseAll)))
}

func fuzzGet(f *testing.F, factory func() Obm) {
	f.Fuzz(func(t *testing.T, key []byte) {
		key = trimKey(key)
		opString := fmt.Sprintf("Get %X\n", key)
		ref, bt := getBaseline(factory)
		expected, expectedOk := ref.Get(key)
		actual, actualOk := bt.Get(key)
		assert.Equal(t, expectedOk, actualOk, opString)
		assert.Equal(t, expected, actual, opString)
	})
}

func fuzzPut(f *testing.F, factory func() Obm) {
	f.Fuzz(func(t *testing.T, key []byte, value byte) {
		key = trimKey(key)
		opString := fmt.Sprintf("Put %X:%d\n", key, value)
		ref, bt := getBaseline(factory)
		expected, expectedOk := ref.Put(key, value)
		actual, actualOk := bt.Put(key, value)
		assert.Equal(t, expectedOk, actualOk, opString)
		assert.Equal(t, expected, actual, opString)
		actual, ok := bt.Get(key)
		assert.True(t, ok, opString)
		assert.Equal(t, value, actual, opString)
	})
}

func fuzzDelete(f *testing.F, factory func() Obm) {
	f.Fuzz(func(t *testing.T, key []byte) {
		key = trimKey(key)
		opString := fmt.Sprintf("Delete %X\n", key)
		ref, bt := getBaseline(factory)
		expected, expectedOk := ref.Delete(key)
		actual, actualOk := bt.Delete(key)
		assert.Equal(t, expectedOk, actualOk, opString)
		assert.Equal(t, expected, actual, opString)
		actual, ok := bt.Get(key)
		assert.False(t, ok, opString)
		assert.Equal(t, byte(0), actual, opString)
	})
}

func fuzzRange(f *testing.F, factory func() Obm) {
	f.Fuzz(func(t *testing.T, begin, end []byte) {
		begin = trimKey(begin)
		end = trimKey(end)
		ref, bt := getBaseline(factory)
		cmp := bytes.Compare(begin, end)
		if cmp == 0 {
			return
		}
		if cmp > 0 {
			begin, end = end, begin
		}
		for _, bounds := range []Bounds{
			From(begin).To(end),
			From(end).DownTo(begin),
			From(nil).To(begin),
			From(begin).To(nil),
			From(nil).DownTo(begin),
			From(begin).DownTo(nil),
		} {
			assert.Equal(t, collect(ref.Range(bounds)), collect(bt.Range(bounds)),
				"%s", bounds)
		}
	})
}

func FuzzSimpleGet(f *testing.F) {
	fuzzGet(f, btrie.NewSimple[byte])
}

func FuzzSimplePut(f *testing.F) {
	fuzzPut(f, btrie.NewSimple[byte])
}

func FuzzSimpleDelete(f *testing.F) {
	fuzzDelete(f, btrie.NewSimple[byte])
}

func FuzzSimpleRange(f *testing.F) {
	fuzzRange(f, btrie.NewSimple[byte])
}
