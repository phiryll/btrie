package btrie_test

import (
	"testing"

	"github.com/phiryll/btrie"
	"github.com/stretchr/testify/assert"
)

type (
	BTrie  = btrie.BTrie
	Entry  = btrie.Entry
	Cursor = btrie.Cursor
)

func TestSimple(t *testing.T) {
	t.Parallel()
	assert.True(t, true)
}
