package btrie

// Things that need to be exported for testing, but should not be part of the public API.
// The identifiers are in the btrie package, but the filename ends in _test.go,
// preventing their inclusion in the public API.

var (
	TestingPreOrder  = preOrder[int]
	TestingPostOrder = postOrder[int]
)

type (
	TestingAdjFunction = adjFunction[int]
)
