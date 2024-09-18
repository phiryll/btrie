package btrie_test

import (
	"encoding/binary"
	"math/rand"
	"testing"

	"github.com/phiryll/btrie"
	"github.com/stretchr/testify/assert"
)

type (
	BTrie  = btrie.BTrie
	Entry  = btrie.Entry
	Cursor = btrie.Cursor
)

func collect(c Cursor) []Entry {
	entries := []Entry{}
	for c.HasNext() {
		entries = append(entries, c.Next())
	}
	return entries
}

func randomEntries(random *rand.Rand, n int) []Entry {
	// keys are of length 2 = 64K possibilities
	// values are of length 1
	entries := make([]Entry, n)
	for i := range entries {
		entries[i].Key = make([]byte, 2)
		_, _ = random.Read(entries[i].Key)
		entries[i].Value = []byte{byte(rand.Intn(256))}
	}
	return entries
}

func putEntries(bt BTrie) {
	bt.Put([]byte{37, 12, 89}, []byte{1})
	bt.Put([]byte{37, 12, 14}, []byte{3, 4})
	bt.Put([]byte{37, 12}, []byte{5, 6})
	bt.Put([]byte{37, 12, 89, 72, 5}, []byte{7})
	bt.Put([]byte{42, 69}, []byte{})
}

func TestSimple(t *testing.T) {
	t.Parallel()
	testBasic(t, btrie.NewSimple())
	testRandom(t, btrie.NewSimple())
}

func testBasic(t *testing.T, bt BTrie) {
	assert.Empty(t, collect(bt.Range(nil, nil)))
	putEntries(bt)
	assert.Equal(t, []byte{1}, bt.Put([]byte{37, 12, 89}, []byte{2}))

	assert.Nil(t, bt.Get([]byte{37, 12, 89, 72}))
	assert.Equal(t, []byte{7}, bt.Get([]byte{37, 12, 89, 72, 5}))
	assert.Equal(t, []byte{2}, bt.Get([]byte{37, 12, 89}))
	assert.Equal(t, []byte{5, 6}, bt.Get([]byte{37, 12}))
	assert.Equal(t, []byte{}, bt.Get([]byte{42, 69}))

	actual := collect(bt.Range(nil, nil))
	assert.Equal(t, []Entry{
		{[]byte{37, 12}, []byte{5, 6}},
		{[]byte{37, 12, 14}, []byte{3, 4}},
		{[]byte{37, 12, 89}, []byte{2}},
		{[]byte{37, 12, 89, 72, 5}, []byte{7}},
		{[]byte{42, 69}, []byte{}},
	}, actual, "actual: %v\n", actual)

	assert.Nil(t, bt.Delete([]byte{37, 12, 89, 72}))
	assert.Equal(t, []byte{2}, bt.Delete([]byte{37, 12, 89}))
	assert.Nil(t, bt.Delete([]byte{37, 12, 89}))
	assert.Equal(t, []byte{7}, bt.Delete([]byte{37, 12, 89, 72, 5}))
	assert.Equal(t, []byte{}, bt.Delete([]byte{42, 69}))
	assert.Equal(t, []byte{5, 6}, bt.Delete([]byte{37, 12}))
	assert.Equal(t, []byte{3, 4}, bt.Delete([]byte{37, 12, 14}))

	assert.Empty(t, collect(bt.Range(nil, nil)))
}

func testRandom(t *testing.T, bt BTrie) {
	random := rand.New(rand.NewSource(43729))
	entries := randomEntries(random, 10000)
	var ref reference
	for _, entry := range entries {
		expected := ref.Put(entry.Key, entry.Value)
		actual := bt.Put(entry.Key, entry.Value)
		assert.Equal(t, expected, actual)
	}
	assert.Equal(t, ref.All(), collect(bt.Range(nil, nil)))
	entries = randomEntries(random, 10000)
	for _, entry := range entries {
		expected := ref.Delete(entry.Key)
		actual := bt.Delete(entry.Key)
		assert.Equal(t, expected, actual)
	}
	assert.Equal(t, ref.All(), collect(bt.Range(nil, nil)))
}

type reference [1 << 16][]byte

func (r *reference) Put(key, value []byte) []byte {
	index := binary.BigEndian.Uint16(key)
	prev := r[index]
	r[index] = value
	return prev
}

func (r *reference) Get(key []byte) []byte {
	index := binary.BigEndian.Uint16(key)
	return r[index]
}

func (r *reference) Delete(key []byte) []byte {
	return r.Put(key, nil)
}

func (r *reference) Range(begin, end []byte) []Entry {
	var entries []Entry
	beginKey := binary.BigEndian.Uint16(begin)
	endKey := binary.BigEndian.Uint16(end)
	for i := beginKey; i < endKey; i++ {
		if r[i] == nil {
			continue
		}
		key := binary.BigEndian.AppendUint16(nil, i)
		entries = append(entries, Entry{key, r[i]})
	}
	return entries
}

func (r *reference) All() []Entry {
	var entries []Entry
	for i, value := range r {
		if value == nil {
			continue
		}
		key := binary.BigEndian.AppendUint16(nil, uint16(i))
		entries = append(entries, Entry{key, value})
	}
	return entries
}
