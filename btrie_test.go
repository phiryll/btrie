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

func TestSimpleShortKey(t *testing.T) {
	t.Parallel()
	testShortKey(t, btrie.NewSimple[byte])
}

func TestSimpleGet1(t *testing.T) {
	t.Parallel()
	bt := btrie.NewSimple[byte]()
	bt.Put([]byte{0xB3, 0x9C}, 184)

	// forgot to check isTerminal
	actual, actualOk := bt.Get([]byte{0xB3})
	assert.Equal(t, false, actualOk)
	assert.Equal(t, byte(0), actual)

	actual, actualOk = bt.Get([]byte{0xB3, 0x9C})
	assert.Equal(t, true, actualOk)
	assert.Equal(t, byte(184), actual)
}

func TestSimpleDelete1(t *testing.T) {
	t.Parallel()
	bt := btrie.NewSimple[byte]()
	bt.Put([]byte{0xB3, 0x9C}, 184)

	actual, actualOk := bt.Delete([]byte{0xB3})
	assert.Equal(t, false, actualOk)
	assert.Equal(t, byte(0), actual)

	// Make sure the subtree wasn't deleted.
	actual, actualOk = bt.Get([]byte{0xB3, 0x9C})
	assert.Equal(t, true, actualOk)
	assert.Equal(t, byte(184), actual)
}

// failing reverse range case found during testing
func TestSimpleRangeFail1(t *testing.T) {
	t.Parallel()
	t.Skip("TODO")
	bt := btrie.NewSimple[byte]()
	bt.Put([]byte{0x50, 0xEF}, 45)
	revRange := From([]byte{0x50}).DownTo([]byte{0x15})
	assert.Equal(t, []entry[byte]{}, collect(bt.Range(revRange)))
}

// failing reverse range case found during testing
func TestSimpleRangeFail2(t *testing.T) {
	t.Parallel()
	t.Skip("TODO")
	bt := btrie.NewSimple[byte]()
	bt.Put([]byte{0x50, 0xEF}, 45)
	revRange := From([]byte{0xFD}).DownTo([]byte{0x3D})
	assert.Equal(t, []entry[byte]{{Key: []byte{0x50, 0xEF}, Value: 45}}, collect(bt.Range(revRange)))
}

// failing reverse range case found during testing
func TestSimpleRangeFail3(t *testing.T) {
	t.Parallel()
	t.Skip("TODO")
	bt := btrie.NewSimple[byte]()
	bt.Put([]byte{0x50, 0xEF}, 45)
	revRange := From([]byte{0x51}).DownTo([]byte{0x50})
	assert.Equal(t, []entry[byte]{{Key: []byte{0x50, 0xEF}, Value: 45}}, collect(bt.Range(revRange)))
}

// failing range case found during testing
func TestSimpleRangeFail4(t *testing.T) {
	// Failure is due to continuing iteration past false yield().
	// Failure requires the second Put.
	t.Parallel()
	bt := btrie.NewSimple[byte]()
	bt.Put([]byte{3}, 0)
	bt.Put([]byte{4}, 0)
	bounds := From([]byte{1}).To([]byte{2})
	assert.Equal(t, []entry[byte]{}, collect(bt.Range(bounds)))
}
