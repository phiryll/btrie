package btrie

import (
	"iter"
)

// Things defined here could be more specificly []byte,
// but I'm not sure that gains anything.
// However, a stack[byte] can be rearranged to save memory.

// TODO: return a sequence of things with both a path and a pruning method.
// But only if a typical impl would need that to more quickly get to Bounds.Begin.

// stack is a simple immutable singly-linked list used by preOrder.
type stack[T any] struct {
	value T
	next  *stack[T]
	size  int
}

func (s *stack[T]) with(value T) *stack[T] {
	return &stack[T]{value, s, s.size + 1}
}

func (s *stack[T]) path() []T {
	nodes := make([]T, s.size)
	for i := s.size - 1; i >= 0; i-- {
		nodes[i] = s.value
		s = s.next
	}
	return nodes
}

func preOrder[T any](root T, adj func(T) iter.Seq[T]) iter.Seq[[]T] {
	return func(yield func([]T) bool) {
		preOrderTraverse(&stack[T]{root, nil, 1}, adj, yield)
	}
}

// Returns a value of true if done (some yield has returned false).
func preOrderTraverse[T any](s *stack[T], adj func(T) iter.Seq[T], yield func([]T) bool) bool {
	if !yield(s.path()) {
		return true
	}
	for node := range adj(s.value) {
		if preOrderTraverse(s.with(node), adj, yield) {
			return true
		}
	}
	return false
}
