package btrie_test

import (
	"iter"
	"testing"

	"github.com/phiryll/btrie"
	"github.com/stretchr/testify/assert"
)

// adjInt returns a simple adjacency function for testing pre-order traversals.
// If k <= limit, children(k) == [4*k+1, 4*k+2, 4*k+3].
// If k > limit, children(k) == [].
func adjInt(limit int) func(int) iter.Seq[int] {
	if limit < 0 {
		panic("limit must be non-negative")
	}
	return func(parent int) iter.Seq[int] {
		if parent > limit {
			return btrie.EmptySeq
		}
		return func(yield func(int) bool) {
			for child := 4*parent + 1; child < 4*parent+4; child++ {
				if !yield(child) {
					return
				}
			}
		}
	}
}

// Paths for the pre-order traversal of adjInt(10) rooted at 0.
var expectedPaths = [][]int{
	{0},
	{0, 1},
	{0, 1, 5},
	{0, 1, 5, 21},
	{0, 1, 5, 22},
	{0, 1, 5, 23},
	{0, 1, 6},
	{0, 1, 6, 25},
	{0, 1, 6, 26},
	{0, 1, 6, 27},
	{0, 1, 7},
	{0, 1, 7, 29},
	{0, 1, 7, 30},
	{0, 1, 7, 31},
	{0, 2},
	{0, 2, 9},
	{0, 2, 9, 37},
	{0, 2, 9, 38},
	{0, 2, 9, 39},
	{0, 2, 10},
	{0, 2, 10, 41},
	{0, 2, 10, 42},
	{0, 2, 10, 43},
	{0, 2, 11},
	{0, 3},
	{0, 3, 13},
	{0, 3, 14},
	{0, 3, 15},
}

func preOrderPaths(root int, adj func(int) iter.Seq[int]) [][]int {
	paths := [][]int{}
	for path := range btrie.TestingPreOrder(root, adj) {
		paths = append(paths, path)
	}
	return paths
}

func TestPreOrder(t *testing.T) {
	t.Parallel()
	assert.Equal(t, [][]int{{0}}, preOrderPaths(0, btrie.TestingEmptyAdj))
	assert.Equal(t, [][]int{{0}, {0, 1}, {0, 2}, {0, 3}}, preOrderPaths(0, adjInt(0)))
	assert.Equal(t, expectedPaths, preOrderPaths(0, adjInt(10)))
	assert.Equal(t, [][]int{{42}}, preOrderPaths(42, btrie.TestingEmptyAdj))
	assert.Equal(t, [][]int{{42}, {42, 169}, {42, 170}, {42, 171}}, preOrderPaths(42, adjInt(50)))
}
