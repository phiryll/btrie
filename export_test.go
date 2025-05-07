package btrie

// Things that need to be exported for testing, but should not be part of the public API.
// The identifiers are in the btrie package, but the filename ends in _test.go,
// preventing their inclusion in the public API.

var (
	TestingChildBounds    = (*Bounds).childBounds
	TestingPreOrder       = preOrder[int]
	TestingPostOrder      = postOrder[int]
	TestingPreOrderPaths  = preOrderPaths[int]
	TestingPostOrderPaths = postOrderPaths[int]
)

type (
	Cloneable[V any] interface {
		BTrie[V]
		Clone() Cloneable[V]
	}

	TestingAdjFunction     = adjFunction[int]
	TestingTraverser       = traverser[int]
	TestingPathAdjFunction = pathAdjFunction[int]
	TestingPathTraverser   = pathTraverser[int]
)

// Assumes V is not a reference type.
func (n *ptrTrieNode[V]) Clone() Cloneable[V] {
	return clonePointerTrie(n)
}

func clonePointerTrie[V any](n *ptrTrieNode[V]) *ptrTrieNode[V] {
	clone := *n
	clone.children = make([]*ptrTrieNode[V], len(n.children))
	for i, child := range n.children {
		clone.children[i] = clonePointerTrie(child)
	}
	return &clone
}

// Assumes V is not a reference type.
func (n *arrayTrieNode[V]) Clone() Cloneable[V] {
	return cloneArrayTrie(n)
}

func cloneArrayTrie[V any](n *arrayTrieNode[V]) *arrayTrieNode[V] {
	if n == nil {
		return nil
	}
	clone := *n
	if n.children != nil {
		clone.children = &[256]*arrayTrieNode[V]{}
		for i, child := range n.children {
			if child != nil {
				clone.children[i] = cloneArrayTrie(child)
			}
		}
	}
	return &clone
}
