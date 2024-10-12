package btrie_test

import (
	"bytes"
	"iter"
	"math/bits"
	"math/rand"
	"testing"

	"github.com/stretchr/testify/assert"
)

// TODO: this is now really all fuzz tests, make them real and delete this file.

// This file contains things that help in writing tests.
// There are no top-level tests here, but the bulk of the testing code is.

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
	randLen := bits.Len(uint(random.Intn(1 << 8)))
	return randomBytes(randLen, random)
}

func testOrderedBytesMap(t *testing.T, f func() Obm, seed int64) {
	const opCount = 100000
	const rangeCount = 100
	bt := f()

	assert.Empty(t, collect(bt.Range(all)))
	assert.Empty(t, collect(bt.Range(reverseAll)))
	random := rand.New(rand.NewSource(seed))
	ref := newReference()

	testOps(t, ref, bt, opCount, random)
	t.Logf("Ref:\n%s\nActual:\n%s", ref, bt)

	assert.Equal(t, collect(ref.Range(all)), collect(bt.Range(all)))
	assert.Equal(t, collect(ref.Range(reverseAll)), collect(bt.Range(reverseAll)))
	testRange(t, ref, bt, rangeCount, random)
}

func testOps(t *testing.T, ref, bt Obm, count int, random *rand.Rand) {
	for range count {
		key := randomKey(random)
		value := randomByte(random)
		switch randOp := random.Float32(); {
		case randOp < 0.5:
			t.Logf("Put %X:%v\n", key, value)
			expected, expectedOk := ref.Put(key, value)
			actual, actualOk := bt.Put(key, value)
			assert.Equal(t, expectedOk, actualOk, "Put %X:%v\n", key, value)
			assert.Equal(t, expected, actual, "Put %X:%v\n", key, value)
		case randOp < 0.6:
			t.Logf("Delete %X\n", key)
			expected, expectedOk := ref.Delete(key)
			actual, actualOk := bt.Delete(key)
			assert.Equal(t, expectedOk, actualOk, "Delete %X\n", key)
			assert.Equal(t, expected, actual, "Delete %X\n", key)
		default:
			t.Logf("Get %X\n", key)
			expected, expectedOk := ref.Get(key)
			actual, actualOk := bt.Get(key)
			assert.Equal(t, expectedOk, actualOk, "Get %X\n%s", key, bt)
			assert.Equal(t, expected, actual, "Get %X\n", key)
		}
	}
}

func testRange(t *testing.T, ref, bt Obm, count int, random *rand.Rand) {
	for range count {
		begin := randomKey(random)
		end := randomKey(random)
		if bytes.Equal(begin, end) {
			continue
		}
		if bytes.Compare(begin, end) > 0 {
			begin, end = end, begin
		}
		bounds := From(begin).To(end)
		assert.Equal(t, collect(ref.Range(bounds)), collect(bt.Range(bounds)),
			"%s", bounds)
		bounds = From(end).DownTo(begin)
		assert.Equal(t, collect(ref.Range(bounds)), collect(bt.Range(bounds)),
			"%s", bounds)
		bounds = From(nil).To(begin)
		assert.Equal(t, collect(ref.Range(bounds)), collect(bt.Range(bounds)),
			"%s", bounds)
		bounds = From(begin).To(nil)
		assert.Equal(t, collect(ref.Range(bounds)), collect(bt.Range(bounds)),
			"%s", bounds)
		bounds = From(nil).DownTo(begin)
		assert.Equal(t, collect(ref.Range(bounds)), collect(bt.Range(bounds)),
			"%s", bounds)
		bounds = From(begin).DownTo(nil)
		assert.Equal(t, collect(ref.Range(bounds)), collect(bt.Range(bounds)),
			"%s", bounds)
	}
}
