package btrie

import (
	"iter"
)

// Traversers returning nodes.

// An adjacency function from nodes to adjacent nodes.
// Adjacency functions should be idempotent.
type adjFunction[T any] func(T) iter.Seq[T]

// A traverser returns a sequence of nodes given a root node and an adjacency function.
// Traversers should be idempotent.
type traverser[T any] func(T, adjFunction[T]) iter.Seq[T]

func preOrder[T any](root T, adj adjFunction[T]) iter.Seq[T] {
	return func(yield func(T) bool) {
		preOrderRecurse(root, adj, yield)
	}
}

// Returns true if done (some yield has returned false).
func preOrderRecurse[T any](node T, adj adjFunction[T], yield func(T) bool) bool {
	if !yield(node) {
		return true
	}
	for adjNode := range adj(node) {
		if preOrderRecurse(adjNode, adj, yield) {
			return true
		}
	}
	return false
}

func postOrder[T any](root T, adj adjFunction[T]) iter.Seq[T] {
	return func(yield func(T) bool) {
		postOrderRecurse(root, adj, yield)
	}
}

// Returns true if done (some yield has returned false).
func postOrderRecurse[T any](node T, adj adjFunction[T], yield func(T) bool) bool {
	for adjNode := range adj(node) {
		if postOrderRecurse(adjNode, adj, yield) {
			return true
		}
	}
	return !yield(node)
}

// Traversers returning paths.

// An adjacency function from a path to nodes adjacent to the path's end.
// Adjacency functions should be idempotent.
type pathAdjFunction[T any] func([]T) iter.Seq[T]

// A pathTraverser returns a sequence of paths given a root node and a pathAdjFunction.
// Traversers should be idempotent.
type pathTraverser[T any] func(T, pathAdjFunction[T]) iter.Seq[[]T]

// The elements of the returned sequence reference a volatile internal slice,
// clone it if you need it after a step in the iteration.
func preOrderPaths[T any](root T, pathAdj pathAdjFunction[T]) iter.Seq[[]T] {
	return func(yield func([]T) bool) {
		preOrderPathsRecurse([]T{root}, pathAdj, yield)
	}
}

// Returns true if done (some yield has returned false).
func preOrderPathsRecurse[T any](path []T, pathAdj pathAdjFunction[T], yield func([]T) bool) bool {
	if !yield(path) {
		return true
	}
	for adjNode := range pathAdj(path) {
		if preOrderPathsRecurse(append(path, adjNode), pathAdj, yield) {
			return true
		}
	}
	return false
}

// The elements of the returned sequence reference a volatile internal slice,
// clone it if you need it after a step in the iteration.
func postOrderPaths[T any](root T, pathAdj pathAdjFunction[T]) iter.Seq[[]T] {
	return func(yield func([]T) bool) {
		postOrderPathsRecurse([]T{root}, pathAdj, yield)
	}
}

// Returns true if done (some yield has returned false).
func postOrderPathsRecurse[T any](path []T, pathAdj pathAdjFunction[T], yield func([]T) bool) bool {
	for adjNode := range pathAdj(path) {
		if postOrderPathsRecurse(append(path, adjNode), pathAdj, yield) {
			return true
		}
	}
	return !yield(path)
}
