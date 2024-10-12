package btrie_test

import (
	"testing"

	"github.com/phiryll/btrie"
	"github.com/stretchr/testify/assert"
)

func TestSimple(t *testing.T) {
	t.Parallel()
	testOrderedBytesMap(t, btrie.NewSimple[byte], 290709)
}

func TestReference(t *testing.T) {
	t.Parallel()
	testOrderedBytesMap(t, newReference, 290709)
}

// Things that failed at one point or another during testing.

func TestFail1(t *testing.T) {
	t.Parallel()
	bt := btrie.NewSimple[byte]()
	bt.Put([]byte{5}, 0)
	assert.Equal(t,
		[]entry[byte]{},
		collect(bt.Range(From([]byte{5, 0}).To([]byte{6}))))
	assert.Equal(t,
		[]entry[byte]{{[]byte{5}, 0}},
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
		[]entry[byte]{},
		collect(bt.Range(From([]byte{0x50}).DownTo([]byte{0x15}))))
}

func TestFail5(t *testing.T) {
	t.Parallel()
	bt := btrie.NewSimple[byte]()
	bt.Put([]byte{0x50, 0xEF}, 45)
	assert.Equal(t,
		[]entry[byte]{{Key: []byte{0x50, 0xEF}, Value: 45}},
		collect(bt.Range(From([]byte{0xFD}).DownTo([]byte{0x3D}))))
}

func TestFail6(t *testing.T) {
	t.Parallel()
	bt := btrie.NewSimple[byte]()
	bt.Put([]byte{0x50, 0xEF}, 45)
	assert.Equal(t,
		[]entry[byte]{{Key: []byte{0x50, 0xEF}, Value: 45}},
		collect(bt.Range(From([]byte{0x51}).DownTo([]byte{0x50}))))
}

func TestFail7(t *testing.T) {
	// Failure is due to continuing iteration past false yield().
	// Failure requires the second Put.
	t.Parallel()
	bt := btrie.NewSimple[byte]()
	bt.Put([]byte{3}, 0)
	bt.Put([]byte{4}, 0)
	bounds := From([]byte{1}).To([]byte{2})
	assert.Equal(t, []entry[byte]{}, collect(bt.Range(bounds)))
}
