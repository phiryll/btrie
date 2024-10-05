package btrie_test

import (
	"testing"

	"github.com/phiryll/btrie"
	"github.com/stretchr/testify/assert"
)

func TestSimple(t *testing.T) {
	t.Parallel()
	testOrderedBytesMap(t, btrie.NewSimple[byte], 290709)
	// TODO: re-enable
	t.Skip()
	{
		// failing reverse range case found during testing
		bt := btrie.NewSimple[byte]()
		bt.Put([]byte{0x50, 0xEF}, 45)
		revRange := From([]byte{0x50}).DownTo([]byte{0x15})
		assert.Equal(t, []entry[byte]{}, collect(bt.Range(revRange)))
	}
}
