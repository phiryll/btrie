package btrie

import (
	"fmt"
	"iter"
	"slices"
	"strings"
)

// A simple implementation, all pointers.

// sub-packages?, because it's helpful to reuse struct names like "root".
// maybe later, at the second implementation.

// NewSimple returns a new, absurdly simple, and badly coded OrderedBytesMap.
// This is purely for fleshing out the unit tests, benchmarks, and fuzz tests.
func NewSimple[V any]() OrderedBytesMap[V] {
	var zero V
	return &node[V]{zero, nil, 0, false}
}

type node[V any] struct {
	value      V // valid only if isTerminal is true
	children   []*node[V]
	keyByte    byte
	isTerminal bool
}

func (n *node[V]) Put(key []byte, value V) (V, bool) {
	if key == nil {
		panic("key must be non-nil")
	}
	var zero V
	for i, keyByte := range key {
		index, found := n.search(keyByte)
		if !found {
			k := len(key) - 1
			child := &node[V]{value, nil, key[k], true}
			for k--; k >= i; k-- {
				child = &node[V]{zero, []*node[V]{child}, key[k], false}
			}
			n.children = append(n.children, child)
			copy(n.children[index+1:], n.children[index:])
			n.children[index] = child
			return zero, false
		}
		n = n.children[index]
	}
	// n = found key, replace value
	prev := zero
	var ok bool
	if n.isTerminal {
		prev = n.value
		ok = true
	}
	n.value = value
	n.isTerminal = true
	return prev, ok
}

func (n *node[V]) Get(key []byte) (V, bool) {
	if key == nil {
		panic("key must be non-nil")
	}
	var zero V
	for _, keyByte := range key {
		index, found := n.search(keyByte)
		if !found {
			return zero, false
		}
		n = n.children[index]
	}
	// n = found key
	if n.isTerminal {
		return n.value, true
	}
	return zero, false
}

func (n *node[V]) Delete(key []byte) (V, bool) {
	if key == nil {
		panic("key must be non-nil")
	}
	var zero V
	type step struct {
		n     *node[V]
		index int // index in n.children where the next keyByte is
	}
	path := []step{}
	for _, keyByte := range key {
		index, found := n.search(keyByte)
		if !found {
			return zero, false
		}
		path = append(path, step{n, index})
		n = n.children[index]
	}
	// n = found key, path goes from root to n's parent
	if !n.isTerminal {
		return zero, false
	}
	prev := n.value
	n.value = zero
	n.isTerminal = false
	// Remove nodes from the tail of path if possible.
	for _, parent := range slices.Backward(path) {
		// if n can't be removed from parent, the loop is done
		if n.isTerminal || len(n.children) > 0 {
			break
		}
		n = parent.n
		index := parent.index
		n.children = append(n.children[:index], n.children[index+1:]...)
	}
	return prev, true
}

func (n *node[V]) Range(bounds Bounds) iter.Seq2[[]byte, V] {
	bounds = bounds.Clone()
	var pathItr iter.Seq[[]*node[V]]
	if bounds.IsReverse() {
		pathItr = postOrder(n, reverseChildAdj[V](bounds))
	} else {
		pathItr = preOrder(n, forwardChildAdj[V](bounds))
	}
	return func(yield func([]byte, V) bool) {
		// path is a []*node[V], with path[0] being the root
		for path := range pathItr {
			key := []byte{}
			for _, n := range path[1:] {
				key = append(key, n.keyByte)
			}
			cmp := bounds.Compare(key)
			if cmp < 0 {
				continue
			}
			if cmp > 0 {
				return
			}
			last := path[len(path)-1]
			if last.isTerminal && !yield(key, last.value) {
				return
			}
		}
	}
}

// Sometimes a child is not within the bounds, but one of its descendants is.
func forwardChildAdj[V any](bounds Bounds) adjFunction[*node[V]] {
	return func(path []*node[V]) iter.Seq[*node[V]] {
		key := []byte{}
		for _, n := range path[1:] {
			key = append(key, n.keyByte)
		}
		start, stop, ok := bounds.childBounds(key)
		if !ok {
			return emptySeq
		}
		last := path[len(path)-1]
		return func(yield func(*node[V]) bool) {
			for i := range len(last.children) {
				keyByte := last.children[i].keyByte
				if keyByte < start {
					continue
				}
				if keyByte > stop {
					return
				}
				if !yield(last.children[i]) {
					return
				}
			}
		}
	}
}

// Sometimes a child is not within the bounds, but one of its descendants is.
func reverseChildAdj[V any](bounds Bounds) adjFunction[*node[V]] {
	return func(path []*node[V]) iter.Seq[*node[V]] {
		key := []byte{}
		for _, n := range path[1:] {
			key = append(key, n.keyByte)
		}
		start, stop, ok := bounds.childBounds(key)
		if !ok {
			return emptySeq
		}
		last := path[len(path)-1]
		return func(yield func(*node[V]) bool) {
			for i := len(last.children) - 1; i >= 0; i-- {
				keyByte := last.children[i].keyByte
				if keyByte > start {
					continue
				}
				if keyByte < stop {
					return
				}
				if !yield(last.children[i]) {
					return
				}
			}
		}
	}
}

func (n *node[V]) String() string {
	var s strings.Builder
	n.printNode(&s, "")
	return s.String()
}

//nolint:revive
func (n *node[V]) printNode(s *strings.Builder, indent string) {
	if indent == "" {
		s.WriteString("[]")
	} else {
		fmt.Fprintf(s, "%s%X", indent, n.keyByte)
	}
	if n.isTerminal {
		fmt.Fprintf(s, ": %v\n", n.value)
	} else {
		s.WriteString("\n")
	}
	for _, child := range n.children {
		child.printNode(s, indent+"  ")
	}
}

func (n *node[V]) search(byt byte) (int, bool) {
	// This is weirdly slightly faster than sort.Search.
	for i := range len(n.children) {
		keyByte := n.children[i].keyByte
		if byt == keyByte {
			return i, true
		}
		if byt < keyByte {
			return i, false
		}
	}
	return len(n.children), false
}
