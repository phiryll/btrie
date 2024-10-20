package btrie

// Things that need to be exported for testing, but should not be part of the public API.
// The identifiers are in the btrie package, but the filename ends in _test.go,
// preventing their inclusion in the public API.

var (
	TestingKeyName     = keyName
	TestingPreOrder    = preOrder[int]
	TestingPostOrder   = postOrder[int]
	TestingChildBounds = Bounds.childBounds
)

type (
	TestingAdjFunction = adjFunction[int]
	TestingTraverser   = traverser[int]

	Cloneable[V any] interface {
		BTrie[V]
		Clone() Cloneable[V]
	}
)

// Assumes V is not a reference type.
func (n *node[V]) Clone() Cloneable[V] {
	return clonePointerTrie(n)
}

func clonePointerTrie[V any](n *node[V]) *node[V] {
	clone := n
	for i, child := range n.children {
		clone.children[i] = clonePointerTrie(child)
	}
	return clone
}
