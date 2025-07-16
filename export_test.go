package kv

// Things that need to be exported for testing, but should not be part of the public API.
// The identifiers are in the kv package, but the filename ends in _test.go,
// preventing their inclusion in the public API.

type (
	TestingAdjFunction     = adjFunction[int]
	TestingTraverser       = traverser[int]
	TestingPathAdjFunction = pathAdjFunction[int]
	TestingPathTraverser   = pathTraverser[int]
)

var (
	TestingChildBounds    = (*Bounds).childBounds
	TestingPreOrder       = preOrder[int]
	TestingPostOrder      = postOrder[int]
	TestingPreOrderPaths  = preOrderPaths[int]
	TestingPostOrderPaths = postOrderPaths[int]
)
