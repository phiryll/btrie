package btrie_test

import (
	"iter"
	"slices"
	"testing"

	"github.com/phiryll/btrie"
	"github.com/stretchr/testify/assert"
)

// adjInt returns a simple adjacency function for testing traversals.
// If k <= limit, children(k) == [4*k+1, 4*k+2, 4*k+3].
// If k > limit, children(k) == [].
func adjInt(limit int) func([]int) iter.Seq[int] {
	if limit < 0 {
		panic("limit must be non-negative")
	}
	return func(path []int) iter.Seq[int] {
		last := path[len(path)-1]
		if last > limit {
			return emptySeqInt
		}
		return func(yield func(int) bool) {
			for child := 4*last + 1; child < 4*last+4; child++ {
				if !yield(child) {
					return
				}
			}
		}
	}
}

// Paths for the pre-order traversal of adjInt(10) rooted at 0.
var expectedPreOrderPaths = [][]int{
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

// Paths for the post-order traversal of adjInt(10) rooted at 0.
var expectedPostOrderPaths = [][]int{
	{0, 1, 5, 21},
	{0, 1, 5, 22},
	{0, 1, 5, 23},
	{0, 1, 5},
	{0, 1, 6, 25},
	{0, 1, 6, 26},
	{0, 1, 6, 27},
	{0, 1, 6},
	{0, 1, 7, 29},
	{0, 1, 7, 30},
	{0, 1, 7, 31},
	{0, 1, 7},
	{0, 1},
	{0, 2, 9, 37},
	{0, 2, 9, 38},
	{0, 2, 9, 39},
	{0, 2, 9},
	{0, 2, 10, 41},
	{0, 2, 10, 42},
	{0, 2, 10, 43},
	{0, 2, 10},
	{0, 2, 11},
	{0, 2},
	{0, 3, 13},
	{0, 3, 14},
	{0, 3, 15},
	{0, 3},
	{0},
}

func preOrderPaths(root int, adj btrie.TestingAdjFunction) [][]int {
	paths := [][]int{}
	for path := range btrie.TestingPreOrder(root, adj) {
		paths = append(paths, slices.Clone(path))
	}
	return paths
}

func postOrderPaths(root int, adj btrie.TestingAdjFunction) [][]int {
	paths := [][]int{}
	for path := range btrie.TestingPostOrder(root, adj) {
		paths = append(paths, slices.Clone(path))
	}
	return paths
}

func TestPreOrder(t *testing.T) {
	t.Parallel()
	assert.Equal(t, [][]int{{0}}, preOrderPaths(0, emptyAdjInt))
	assert.Equal(t, [][]int{{0}, {0, 1}, {0, 2}, {0, 3}}, preOrderPaths(0, adjInt(0)))
	assert.Equal(t, expectedPreOrderPaths, preOrderPaths(0, adjInt(10)))
	assert.Equal(t, [][]int{{42}}, preOrderPaths(42, emptyAdjInt))
	assert.Equal(t, [][]int{{42}, {42, 169}, {42, 170}, {42, 171}}, preOrderPaths(42, adjInt(50)))
}

func TestPostOrder(t *testing.T) {
	t.Parallel()
	assert.Equal(t, [][]int{{0}}, postOrderPaths(0, emptyAdjInt))
	assert.Equal(t, [][]int{{0, 1}, {0, 2}, {0, 3}, {0}}, postOrderPaths(0, adjInt(0)))
	assert.Equal(t, expectedPostOrderPaths, postOrderPaths(0, adjInt(10)))
	assert.Equal(t, [][]int{{42}}, postOrderPaths(42, emptyAdjInt))
	assert.Equal(t, [][]int{{42, 169}, {42, 170}, {42, 171}, {42}}, postOrderPaths(42, adjInt(50)))
}
