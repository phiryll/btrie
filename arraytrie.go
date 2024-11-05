package btrie

import (
	"bytes"
	"fmt"
	"iter"
	"strings"
)

type arrayTrieNode[V any] struct {
	value      V                       // valid only if isTerminal is true
	children   *[256]*arrayTrieNode[V] // only non-nil if there are children
	isTerminal bool
}

// Returns 0, 1, or 2 (for >= 2).
func (n *arrayTrieNode[V]) cardinality() int {
	if n.children == nil {
		return 0
	}
	var count int
	for _, child := range n.children {
		if child == nil {
			continue
		}
		if count >= 1 {
			//nolint:mnd
			return 2
		}
		count++
	}
	return count
}

// NewArrayTrie returns a new BTrie with pointers to children stored in arrays.
func NewArrayTrie[V any]() BTrie[V] {
	var zero V
	return &arrayTrieNode[V]{zero, nil, false}
}

func (n *arrayTrieNode[V]) Get(key []byte) (V, bool) {
	if key == nil {
		panic("key must be non-nil")
	}
	var zero V
	for _, keyByte := range key {
		if n.children == nil {
			return zero, false
		}
		n = n.children[keyByte]
		if n == nil {
			return zero, false
		}
	}
	// n = found key
	if n.isTerminal {
		return n.value, true
	}
	return zero, false
}

func (n *arrayTrieNode[V]) Put(key []byte, value V) (V, bool) {
	if key == nil {
		panic("key must be non-nil")
	}
	var zero V
	for i, keyByte := range key {
		if n.children == nil {
			n.children = &[256]*arrayTrieNode[V]{}
		}
		if n.children[keyByte] == nil {
			child := &arrayTrieNode[V]{value, nil, true}
			for k := len(key) - 1; k > i; k-- {
				parent := &arrayTrieNode[V]{zero, &[256]*arrayTrieNode[V]{}, false}
				parent.children[key[k]] = child
				child = parent
			}
			n.children[keyByte] = child
			return zero, false
		}
		n = n.children[keyByte]
	}
	// n = found key, replace value
	if n.isTerminal {
		prev := n.value
		n.value = value
		return prev, true
	}
	n.value = value
	n.isTerminal = true
	return zero, false
}

func (n *arrayTrieNode[V]) Delete(key []byte) (V, bool) {
	if key == nil {
		panic("key must be non-nil")
	}
	var zero V
	// If the deleted node has no children, remove the subtree rooted at prune.children[pruneIndex].
	var prune *arrayTrieNode[V]
	var pruneIndex byte
	for i, keyByte := range key {
		if n.children == nil || n.children[keyByte] == nil {
			return zero, false
		}
		// If either n is the root, or n has a value, or n has more than one child, n itself cannot be pruned.
		// If so, move the maybe-pruned subtree to n.children[index].
		if i == 0 || n.isTerminal || n.cardinality() > 1 {
			prune, pruneIndex = n, keyByte
		}
		n = n.children[keyByte]
	}
	// n = found key
	if !n.isTerminal {
		return zero, false
	}
	prev := n.value
	n.value = zero
	n.isTerminal = false
	if len(key) > 0 && n.cardinality() == 0 {
		prune.children[pruneIndex] = nil
	}
	return prev, true
}

// An iter.Seq of these is returned from the adjFunction used internally by Range.
// key = path from root to node
// It is cached here for efficiency, otherwise an iter.Seq of []*arrayTrieNode[V] would be used directly.
// Note that the key must be cloned when yielded from Range.
type arrayTrieRangePath[V any] struct {
	node *arrayTrieNode[V]
	key  []byte
}

func (n *arrayTrieNode[V]) Range(bounds Bounds) iter.Seq2[[]byte, V] {
	bounds = bounds.Clone()
	root := arrayTrieRangePath[V]{n, []byte{}}
	var pathItr iter.Seq[*arrayTrieRangePath[V]]
	if bounds.IsReverse() {
		pathItr = postOrder(&root, arrayTrieReverseAdj[V](bounds))
	} else {
		pathItr = preOrder(&root, arrayTrieForwardAdj[V](bounds))
	}
	return func(yield func([]byte, V) bool) {
		for path := range pathItr {
			cmp := bounds.Compare(path.key)
			if cmp < 0 {
				continue
			}
			if cmp > 0 {
				return
			}
			if path.node.isTerminal && !yield(bytes.Clone(path.key), path.node.value) {
				return
			}
		}
	}
}

func arrayTrieForwardAdj[V any](bounds Bounds) adjFunction[*arrayTrieRangePath[V]] {
	// Sometimes a child is not within the bounds, but one of its descendants is.
	return func(path *arrayTrieRangePath[V]) iter.Seq[*arrayTrieRangePath[V]] {
		if path.node.children == nil {
			return emptySeq
		}
		start, stop, ok := bounds.childBounds(path.key)
		if !ok {
			// Unreachable because of how the trie is traversed forward.
			panic("unreachable")
		}
		return func(yield func(*arrayTrieRangePath[V]) bool) {
			for i, child := range path.node.children[start : int(stop)+1] {
				if child == nil {
					continue
				}
				if !yield(&arrayTrieRangePath[V]{child, append(path.key, start+byte(i))}) {
					return
				}
			}
		}
	}
}

func arrayTrieReverseAdj[V any](bounds Bounds) adjFunction[*arrayTrieRangePath[V]] {
	// Sometimes a child is not within the bounds, but one of its descendants is.
	return func(path *arrayTrieRangePath[V]) iter.Seq[*arrayTrieRangePath[V]] {
		if path.node.children == nil {
			return emptySeq
		}
		start, stop, ok := bounds.childBounds(path.key)
		if !ok {
			return emptySeq
		}
		return func(yield func(*arrayTrieRangePath[V]) bool) {
			children := path.node.children[stop : int(start)+1]
			for i := len(children) - 1; i >= 0; i-- {
				child := children[i]
				if child == nil {
					continue
				}
				if !yield(&arrayTrieRangePath[V]{child, append(path.key, stop+byte(i))}) {
					return
				}
			}
		}
	}
}

func (n *arrayTrieNode[V]) String() string {
	var s strings.Builder
	n.printNode(&s, 0, "")
	return s.String()
}

//nolint:revive
func (n *arrayTrieNode[V]) printNode(s *strings.Builder, keyByte byte, indent string) {
	if indent == "" {
		s.WriteString("[]")
	} else {
		fmt.Fprintf(s, "%s%02X", indent, keyByte)
	}
	if n.isTerminal {
		fmt.Fprintf(s, ": %v\n", n.value)
	} else {
		s.WriteString("\n")
	}
	if n.children == nil {
		return
	}
	for i, child := range n.children {
		if child != nil {
			child.printNode(s, byte(i), indent+"  ")
		}
	}
}
