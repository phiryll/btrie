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

type entry[V any] struct {
	Value V
	Key   []byte
}

func (n *node[V]) Range(bounds *Bounds) iter.Seq2[[]byte, V] {
	if bounds.Reverse {
		panic("unimplemented")
	}
	return func(yield func([]byte, V) bool) {
		if len(n.children) == 0 {
			return
		}
		// TODO: make this lazy and non-recursive - DFS
		var entries []entry[V]
		// this if catches an empty end
		if bounds.End == nil {
			n.rangeFrom([]byte{}, bounds.Begin, &entries)
		} else {
			n.rangeBetween([]byte{}, bounds.Begin, bounds.End, &entries)
		}
		for _, e := range entries {
			if !yield(e.Key, e.Value) {
				return
			}
		}
	}
}

func (*node[V]) Cursor(_ *Bounds) iter.Seq[Pos[V]] {
	panic("unimplemented")
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

func with(prefix []byte, b byte) []byte {
	return append(append([]byte{}, prefix...), b)
}

// begin <= entries < end.
func (n *node[V]) rangeBetween(prefix, begin, end []byte, entries *[]entry[V]) {
	// invariant: end is not empty
	if len(begin) == 0 {
		n.rangeTo(prefix, end, entries)
		return
	}
	beginIndex, beginFound := n.search(begin[0])
	if begin[0] == end[0] {
		// len(end) > 1, because otherwise begin >= end
		if beginFound {
			child := n.children[beginIndex]
			child.rangeBetween(with(prefix, child.keyByte), begin[1:], end[1:], entries)
		}
		// the entire search range is not present
		return
	}
	// begin[0] < end[0]
	if beginFound {
		child := n.children[beginIndex]
		child.rangeFrom(with(prefix, child.keyByte), begin[1:], entries)
		beginIndex++
	}
	endIndex, endFound := n.search(end[0])
	for _, child := range n.children[beginIndex:endIndex] {
		child.descendants(with(prefix, child.keyByte), entries)
	}
	if endFound {
		child := n.children[endIndex]
		child.rangeTo(with(prefix, child.keyByte), end[1:], entries)
	}
}

// begin <= entries.
func (n *node[V]) rangeFrom(prefix, begin []byte, entries *[]entry[V]) {
	if len(begin) == 0 {
		n.descendants(prefix, entries)
		return
	}
	index, found := n.search(begin[0])
	if found {
		child := n.children[index]
		child.rangeFrom(with(prefix, child.keyByte), begin[1:], entries)
		index++
	}
	for _, child := range n.children[index:] {
		child.descendants(with(prefix, child.keyByte), entries)
	}
}

// entries < end.
func (n *node[V]) rangeTo(prefix, end []byte, entries *[]entry[V]) {
	if len(end) == 0 {
		return
	}
	if n.isTerminal {
		*entries = append(*entries, entry[V]{n.value, prefix})
	}
	index, found := n.search(end[0])
	for _, child := range n.children[:index] {
		child.descendants(with(prefix, child.keyByte), entries)
	}
	if found {
		child := n.children[index]
		child.rangeTo(with(prefix, child.keyByte), end[1:], entries)
	}
}

func (n *node[V]) descendants(prefix []byte, entries *[]entry[V]) {
	if n.isTerminal {
		*entries = append(*entries, entry[V]{n.value, prefix})
	}
	for _, child := range n.children {
		child.descendants(with(prefix, child.keyByte), entries)
	}
}
