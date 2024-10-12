package btrie_test

import (
	"testing"

	"github.com/phiryll/btrie"
	"github.com/stretchr/testify/assert"
)

func TestSimple(t *testing.T) {
	t.Parallel()
	t.Skip("TODO")
	testOrderedBytesMap(t, btrie.NewSimple[byte], 290709)
}

func TestReference(t *testing.T) {
	t.Parallel()
	testOrderedBytesMap(t, newReference, 290709)
}

// failing reverse range case found during testing
func TestSimpleRangeFail(t *testing.T) {
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
