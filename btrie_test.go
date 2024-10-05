package btrie_test

import (
	"testing"

	"github.com/phiryll/btrie"
)

func TestSimple(t *testing.T) {
	t.Parallel()
	testBTrie(t, btrie.DeprNewSimple[[]byte], 290709)
}
