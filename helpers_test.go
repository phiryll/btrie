package btrie_test

import (
	"bytes"
	"iter"
	"math/rand"
	"testing"

	"github.com/phiryll/btrie"
	"github.com/stretchr/testify/assert"
)

// This file contains things that help in writing tests.
// There are no top-level tests here, but the bulk of the testing code is.

type (
	Bounds = btrie.Bounds

	entry[V any] struct {
		Key   []byte
		Value V
	}
)

var (
	From = btrie.From
	all  = From(nil).To(nil)
)

func emptySeqInt(_ func(int) bool) {}

func emptyAdjInt(_ []int) iter.Seq[int] {
	return emptySeqInt
}

func collect[V any](itr iter.Seq2[[]byte, V]) []entry[V] {
	entries := []entry[V]{}
	for k, v := range itr {
		entries = append(entries, entry[V]{k, v})
	}
	return entries
}

// Other than edge cases, the most effective tests are essentially fuzz tests.
// The standard library fuzzing isn't quite sufficient in this case.
// Instead, these tests repeatedly call all OrderedBytesMap methods randomly,
// and compare the result to a reference.

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

func randomKey(random *rand.Rand) []byte {
	switch randLen := random.Float32(); {
	case randLen < 0.01:
		return randomBytes(1, random)
	case randLen < 0.5:
		return randomBytes(2, random)
	default:
		return randomBytes(3, random)
	}
}

func testOrderedBytesMap(t *testing.T, f func() btrie.OrderedBytesMap[byte], seed int64) {
	const opCount = 100000
	const rangeCount = 100

	bt := f()

	assert.Empty(t, collect(bt.Range(all)))
	random := rand.New(rand.NewSource(seed))
	ref := newReference()

	for range opCount {
		entry := entry[byte]{randomKey(random), randomByte(random)}
		switch randOp := random.Float32(); {
		case randOp < 0.5:
			t.Logf("Put %X:%v\n", entry.Key, entry.Value)
			expected, expectedOk := ref.Put(entry.Key, entry.Value)
			actual, actualOk := bt.Put(entry.Key, entry.Value)
			assert.Equal(t, expectedOk, actualOk, "Put %X:%v\n", entry.Key, entry.Value)
			assert.Equal(t, expected, actual, "Put %X:%v\n", entry.Key, entry.Value)
		case randOp < 0.6:
			t.Logf("Delete %X\n", entry.Key)
			expected, expectedOk := ref.Delete(entry.Key)
			actual, actualOk := bt.Delete(entry.Key)
			assert.Equal(t, expectedOk, actualOk, "Delete %X\n", entry.Key)
			assert.Equal(t, expected, actual, "Delete %X\n", entry.Key)
		default:
			t.Logf("Get %X\n", entry.Key)
			expected, expectedOk := ref.Get(entry.Key)
			actual, actualOk := bt.Get(entry.Key)
			assert.Equal(t, expectedOk, actualOk, "Get %X\n%s", entry.Key, bt)
			assert.Equal(t, expected, actual, "Get %X\n", entry.Key)
		}
	}

	t.Logf("Ref:\n%s\nActual:\n%s", ref, bt)
	assert.Equal(t, collect(ref.Range(all)), collect(bt.Range(all)))

	for range rangeCount {
		begin := randomKey(random)
		end := randomKey(random)
		if bytes.Equal(begin, end) {
			continue
		}
		if bytes.Compare(begin, end) > 0 {
			begin, end = end, begin
		}
		bounds := From(begin).To(end)
		t.Logf("Forward Range: %s\n", bounds)
		assert.Equal(t, collect(ref.Range(bounds)), collect(bt.Range(bounds)),
			"%s", bounds)
	}
}
