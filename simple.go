package btrie

import (
	"bytes"
	"fmt"
	"sort"
	"strings"
)

// A simple implementation, all pointers.

// sub-packages?, because I kind of want to reuse struct names like "root".
// maybe later, at the second implementation.

// no way to distinguish the root from an internal node,
// so top-level functions must handle it differently.

type node struct {
	children []*node
	value    []byte // non-nil only if this is a terminal node
	keyByte  byte
}

type cursor struct {
	entries []Entry
	index   int
}

func (n *node) String() string {
	var s strings.Builder
	n.printNode(&s, "")
	return s.String()
}

func (n *node) printNode(s *strings.Builder, indent string) {
	v := "nil"
	if n.value != nil {
		v = fmt.Sprintf("%v", n.value)
	}
	//nolint:revive
	fmt.Fprintf(s, "%s%d: %s\n", indent, n.keyByte, v)
	for _, child := range n.children {
		child.printNode(s, indent+"  ")
	}
}

func (n *node) Put(key, value []byte) []byte {
	if len(key) == 0 {
		panic("key must be non-empty")
	}
	if value == nil {
		panic("value must be non-nil")
	}
	// TODO: this does not need to recurse
	return n.put(key, value)
}

func (n *node) Get(key []byte) []byte {
	// TODO: this does not need to recurse
	index, found := n.search(key[0])
	if !found {
		return nil
	}
	if len(key) == 1 {
		return n.children[index].value
	}
	return n.children[index].Get(key[1:])
}

func (n *node) Delete(key []byte) []byte {
	// TODO: this does not need to recurse
	_, value := n.deleteKey(key)
	return value
}

func (n *node) Range(begin, end []byte) Cursor {
	var entries []Entry
	// TODO: make this lazy and non-recursive - DFS

	// this if catches an empty end
	if len(n.children) == 0 ||
		(end != nil && bytes.Compare(begin, end) >= 0) {
		return &cursor{entries, 0}
	}
	if end == nil {
		n.rangeFrom([]byte{}, begin, &entries)
	} else {
		n.rangeBetween([]byte{}, begin, end, &entries)
	}
	return &cursor{entries, 0}
}

func (c *cursor) HasNext() bool {
	return c.index < len(c.entries)
}

func (c *cursor) Next() Entry {
	entry := c.entries[c.index]
	c.index++
	return entry
}

func (n *node) put(key, value []byte) []byte {
	index, found := n.search(key[0])
	if len(key) == 1 {
		if found {
			child := n.children[index]
			oldValue := child.value
			child.value = value
			return oldValue
		}
		n.insert(index, &node{nil, value, key[0]})
		return nil
	}
	if found {
		return n.children[index].put(key[1:], value)
	}
	child := node{nil, nil, key[0]}
	n.insert(index, &child)
	prev := &child
	for _, b := range key[1:] {
		next := node{nil, nil, b}
		prev.children = []*node{&next}
		prev = &next
	}
	prev.value = value
	return nil
}

func (n *node) search(byt byte) (int, bool) {
	index := sort.Search(len(n.children), func(i int) bool {
		return byt <= n.children[i].keyByte
	})
	if index < len(n.children) && byt == n.children[index].keyByte {
		return index, true
	}
	return index, false
}

func (n *node) insert(i int, child *node) {
	var temp node
	n.children = append(n.children, &temp)
	copy(n.children[i+1:], n.children[i:])
	n.children[i] = child
}

// returns true iff n should be deleted from its parent.
func (n *node) deleteKey(key []byte) (bool, []byte) {
	index, found := n.search(key[0])
	if !found {
		return false, nil
	}
	child := n.children[index]
	var value []byte
	var removeChild bool
	if len(key) == 1 {
		value = child.value
		child.value = nil
		removeChild = len(child.children) == 0
	} else {
		removeChild, value = child.deleteKey(key[1:])
	}
	if removeChild {
		n.children = append(n.children[:index], n.children[index+1:]...)
	}
	return len(n.children) == 0 && n.value == nil, value
}

func with(prefix []byte, b byte) []byte {
	return append(append([]byte{}, prefix...), b)
}

// begin <= entries < end.
func (n *node) rangeBetween(prefix, begin, end []byte, entries *[]Entry) {
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
func (n *node) rangeFrom(prefix, begin []byte, entries *[]Entry) {
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
func (n *node) rangeTo(prefix, end []byte, entries *[]Entry) {
	if len(end) == 0 {
		return
	}
	if n.value != nil {
		*entries = append(*entries, Entry{prefix, n.value})
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

func (n *node) descendants(prefix []byte, entries *[]Entry) {
	if n.value != nil {
		*entries = append(*entries, Entry{prefix, n.value})
	}
	for _, child := range n.children {
		child.descendants(with(prefix, child.keyByte), entries)
	}
}
