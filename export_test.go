package btrie

// Things that need to be exported for testing, but should not be part of the public API.
// The identifiers are in the btrie package, but the filename ends in _test.go,
// preventing their inclusion in the public API.

var (
	TestingKeyName        = keyName
	TestingChildBounds    = Bounds.childBounds
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
func (n *node[V]) Clone() Cloneable[V] {
	return clonePointerTrie(n)
}

func clonePointerTrie[V any](n *node[V]) *node[V] {
	clone := *n
	clone.children = make([]*node[V], len(n.children))
	for i, child := range n.children {
		clone.children[i] = clonePointerTrie(child)
	}
	return &clone
}
