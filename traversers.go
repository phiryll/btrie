package btrie

import "iter"

// The returned sequence references a volatile internal slice,
// clone it if you need it after a step in the iteration.
func preOrder[T any](root T, adj func(T) iter.Seq[T]) iter.Seq[[]T] {
	return func(yield func([]T) bool) {
		preOrderRecurse([]T{root}, adj, yield)
	}
}

// Returns a value of true if done (some yield has returned false).
func preOrderRecurse[T any](path []T, adj func(T) iter.Seq[T], yield func([]T) bool) bool {
	if !yield(path) {
		return true
	}
	for node := range adj(path[len(path)-1]) {
		if preOrderRecurse(append(path, node), adj, yield) {
			return true
		}
	}
	return false
}
