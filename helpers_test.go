package btrie_test

import (
	"bytes"
	"math/rand"
	"testing"

	"github.com/phiryll/btrie"
	"github.com/stretchr/testify/assert"
)

// This file contains things that help in writing tests.
// There are no top-level tests here, but the bulk of the testing code is.

type (
	BTrie  = btrie.BTrie
	Cursor = btrie.Cursor

	Entry struct {
		Key, Value []byte
	}
)

func collect(c Cursor) []Entry {
	if refCursor, ok := c.(*cursor); ok {
		return refCursor.entries
	}
	entries := []Entry{}
	for c.HasNext() {
		k, v := c.Next()
		entries = append(entries, Entry{k, v})
	}
	return entries
}

// Other than edge cases, the most effective tests are essentially fuzz tests.
// The standard library fuzzing isn't quite sufficient in this case.
// Instead, these tests repeatedly call all BTrie methods randomly,
// and compare the result to a reference.

// TODO: expand this to cover before/at/after edge cases.
func testShortKey(t *testing.T, f func() BTrie) {
	bt := f()
	bt.Put([]byte{5}, []byte{0})
	assert.Equal(t,
		[]Entry{},
		collect(bt.Range([]byte{5, 0}, []byte{6})))
	assert.Equal(t,
		[]Entry{{[]byte{5}, []byte{0}}},
		collect(bt.Range([]byte{4}, []byte{5, 0})))
}

func randomBytes(n int, random *rand.Rand) []byte {
	b := make([]byte, n)
	_, _ = random.Read(b)
	return b
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

func testBTrie(t *testing.T, f func() BTrie, seed int64) {
	const opCount = 100000
	const rangeCount = 100

	testShortKey(t, f)

	bt := f()

	assert.Empty(t, collect(bt.Range(nil, nil)))
	random := rand.New(rand.NewSource(seed))
	ref := newReference()

	for i := 0; i < opCount; i++ {
		entry := Entry{randomKey(random), randomBytes(1, random)}
		switch randOp := random.Float32(); {
		case randOp < 0.5:
			expected := ref.Put(entry.Key, entry.Value)
			actual := bt.Put(entry.Key, entry.Value)
			assert.Equal(t, expected, actual)
		case randOp < 0.6:
			expected := ref.Delete(entry.Key)
			actual := bt.Delete(entry.Key)
			assert.Equal(t, expected, actual)
		default:
			expected := ref.Get(entry.Key)
			actual := bt.Get(entry.Key)
			assert.Equal(t, expected, actual)
		}
	}
	assert.Equal(t, collect(ref.Range(nil, nil)), collect(bt.Range(nil, nil)))

	for i := 0; i < rangeCount; i++ {
		begin := randomKey(random)
		end := randomKey(random)
		if bytes.Equal(begin, end) {
			assert.Empty(t, collect(bt.Range(begin, end)))
			continue
		}
		if bytes.Compare(begin, end) > 0 {
			begin, end = end, begin
		}
		assert.Equal(t, collect(ref.Range(begin, end)), collect(bt.Range(begin, end)),
			"[%X, %X)", begin, end)
	}
}
