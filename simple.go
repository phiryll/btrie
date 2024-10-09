package btrie

import (
	"fmt"
	"iter"
	"slices"
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

func (n *node[V]) Get(key []byte) (V, bool) {
	// TODO: this does not need to recurse
	index, found := n.search(key[0])
	if !found {
		var zero V
		return zero, false
	}
	if len(key) == 1 {
		return n.children[index].value, true
	}
	return n.children[index].Get(key[1:])
}

func (n *node[V]) Delete(key []byte) (V, bool) {
	// TODO: this does not need to recurse
	_, value, ok := n.deleteKey(key)
	return value, ok
}

func childIter[V any](n *node[V]) iter.Seq[*node[V]] {
	return slices.Values(n.children)
}

func reverseChildIter[V any](n *node[V]) iter.Seq[*node[V]] {
	return func(yield func(*node[V]) bool) {
		for i := len(n.children) - 1; i >= 0; i-- {
			if !yield(n.children[i]) {
				return
			}
		}
	}
}

func (n *node[V]) Range(bounds *Bounds) iter.Seq2[[]byte, V] {
	bounds = bounds.Clone()
	adj := childIter[V]
	if bounds.Reverse {
		adj = reverseChildIter[V]
	}
	return func(yield func([]byte, V) bool) {
		// path is a []*node[V], with path[0] being the root
		for path := range preOrder(n, adj) {
			var key []byte
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
			end := path[len(path)-1]
			if !end.isTerminal {
				continue
			}
			if !yield(key, end.value) {
				return
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
	v := "DNE"
	if n.isTerminal {
		v = fmt.Sprintf("%v", n.value)
	}
	//nolint:revive
	fmt.Fprintf(s, "%s%d: %s\n", indent, n.keyByte, v)
	for _, child := range n.children {
		child.printNode(s, indent+"  ")
	}
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

// returns true iff n should be deleted from its parent.
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
		prev = child.value
		child.value = zero
		child.isTerminal = false
		ok = true
		removeChild = len(child.children) == 0
	} else {
		removeChild, prev, ok = child.deleteKey(key[1:])
	}
	if removeChild {
		n.children = append(n.children[:index], n.children[index+1:]...)
	}
	return len(n.children) == 0 && !n.isTerminal, prev, ok
}
