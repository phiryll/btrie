package btrie

import (
	"fmt"
	"iter"
	"sort"
	"strings"
)

// A simple implementation, all pointers.

// sub-packages?, because it's helpful to reuse struct names like "root".
// maybe later, at the second implementation.

// no way to distinguish the root from an internal node,
// so top-level functions must handle it differently.

// NewSimple returns a new, absurdly simple, and badly coded OrderedBytesMap.
// This is purely for fleshing out the unit tests, benchmarks, and fuzz tests.
// TODO: allow empty keys.
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
	if len(key) == 0 {
		panic("key must be non-empty")
	}
	// TODO: this does not need to recurse
	return n.put(key, value)
}

func (n *node[V]) put(key []byte, value V) (V, bool) {
	var zero V
	index, found := n.search(key[0])
	if len(key) == 1 {
		if !found {
			n.insert(index, &node[V]{value, nil, key[0], true})
			return zero, false
		}
		child := n.children[index]
		if child.isTerminal {
			oldValue := child.value
			child.value = value
			return oldValue, true
		}
		child.value = value
		child.isTerminal = true
		return zero, false
	}
	if found {
		return n.children[index].put(key[1:], value)
	}
	child := node[V]{zero, nil, key[0], false}
	n.insert(index, &child)
	prev := &child
	for _, b := range key[1:] {
		next := node[V]{zero, nil, b, false}
		prev.children = []*node[V]{&next}
		prev = &next
	}
	prev.value = value
	prev.isTerminal = true
	return zero, false
}

func (n *node[V]) Get(key []byte) (V, bool) {
	// TODO: this does not need to recurse
	var zero V
	index, found := n.search(key[0])
	if !found {
		return zero, false
	}
	child := n.children[index]
	if len(key) == 1 {
		if child.isTerminal {
			return child.value, true
		}
		return zero, false
	}
	return child.Get(key[1:])
}

func (n *node[V]) Delete(key []byte) (V, bool) {
	// TODO: this does not need to recurse
	_, value, ok := n.deleteKey(key)
	return value, ok
}

// Returns first arg true iff n should be deleted from its parent.
// Last arg is whether the returned value exists.
func (n *node[V]) deleteKey(key []byte) (bool, V, bool) {
	var zero V
	index, found := n.search(key[0])
	if !found {
		return false, zero, false
	}
	child := n.children[index]
	var removeChild bool
	var prev V
	var ok bool
	if len(key) == 1 {
		if child.isTerminal {
			ok = true
			prev = child.value
			child.value = zero
			child.isTerminal = false
		}
		removeChild = len(child.children) == 0
	} else {
		removeChild, prev, ok = child.deleteKey(key[1:])
	}
	if removeChild {
		n.children = append(n.children[:index], n.children[index+1:]...)
	}
	return len(n.children) == 0 && !n.isTerminal, prev, ok
}

func (n *node[V]) Range(bounds *Bounds) iter.Seq2[[]byte, V] {
	bounds = bounds.Clone()
	var pathItr iter.Seq[[]*node[V]]
	if bounds.Reverse {
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
			if !last.isTerminal {
				continue
			}
			if !yield(key, last.value) {
				return
			}
		}
	}
}

// Sometimes a child is not within the bounds, but one of its descendants is.
func forwardChildAdj[V any](bounds *Bounds) adjFunction[*node[V]] {
	return func(path []*node[V]) iter.Seq[*node[V]] {
		key := []byte{}
		for _, n := range path[1:] {
			key = append(key, n.keyByte)
		}
		last := path[len(path)-1]
		start, stop, ok := bounds.childBounds(key)
		if !ok {
			return emptySeq
		}
		return func(yield func(*node[V]) bool) {
			for _, child := range last.children {
				keyByte := child.keyByte
				if keyByte < start {
					continue
				}
				if keyByte > stop {
					return
				}
				if !yield(child) {
					return
				}
			}
		}
	}
}

// Sometimes a child is not within the bounds, but one of its descendants is.
func reverseChildAdj[V any](bounds *Bounds) adjFunction[*node[V]] {
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

func (n *node[V]) printNode(s *strings.Builder, indent string) {
	if indent == "" {
		if n.isTerminal {
			s.WriteString(fmt.Sprintf("[]: %v\n", n.value))
		} else {
			s.WriteString("[]\n")
		}
	} else {
		if n.isTerminal {
			s.WriteString(fmt.Sprintf("%s%X: %v\n", indent, n.keyByte, n.value))
		} else {
			s.WriteString(fmt.Sprintf("%s%X\n", indent, n.keyByte))
		}
	}
	for _, child := range n.children {
		child.printNode(s, indent+"  ")
	}
}

func (n *node[V]) search(byt byte) (int, bool) {
	index := sort.Search(len(n.children), func(i int) bool {
		return byt <= n.children[i].keyByte
	})
	if index < len(n.children) && byt == n.children[index].keyByte {
		return index, true
	}
	return index, false
}

func (n *node[V]) insert(i int, child *node[V]) {
	var temp node[V]
	n.children = append(n.children, &temp)
	copy(n.children[i+1:], n.children[i:])
	n.children[i] = child
}
