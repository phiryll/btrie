package btrie

import (
	"bytes"
	"fmt"
	"iter"
	"strings"
)

type ptrTrieNode[V any] struct {
	value      V // valid only if isTerminal is true
	children   []*ptrTrieNode[V]
	keyByte    byte
	isTerminal bool
}

// NewPointerTrie returns a new, absurdly simple, and badly coded BTrie.
// Pointers to children are stored densely in slices.
// This is purely for fleshing out the unit tests, benchmarks, and fuzz tests.
func NewPointerTrie[V any]() BTrie[V] {
	var zero V
	return &ptrTrieNode[V]{zero, nil, 0, false}
}

func (n *ptrTrieNode[V]) Get(key []byte) (V, bool) {
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

func (n *ptrTrieNode[V]) Put(key []byte, value V) (V, bool) {
	if key == nil {
		panic("key must be non-nil")
	}
	var zero V
	for i, keyByte := range key {
		index, found := n.search(keyByte)
		if !found {
			k := len(key) - 1
			child := &ptrTrieNode[V]{value, nil, key[k], true}
			for k--; k >= i; k-- {
				child = &ptrTrieNode[V]{zero, []*ptrTrieNode[V]{child}, key[k], false}
			}
			n.children = append(n.children, child)
			copy(n.children[index+1:], n.children[index:])
			n.children[index] = child
			return zero, false
		}
		n = n.children[index]
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

func (n *ptrTrieNode[V]) Delete(key []byte) (V, bool) {
	if key == nil {
		panic("key must be non-nil")
	}
	var zero V
	// Treating the root key as a special case makes the code below simpler wrt pruning.
	if len(key) == 0 {
		if !n.isTerminal {
			return zero, false
		}
		prev := n.value
		n.value = zero
		n.isTerminal = false
		return prev, true
	}

	// If the deleted node has no children, remove the subtree rooted at prune.children[pruneIndex].
	prune, pruneIndex := n, 0
	for _, keyByte := range key {
		index, found := n.search(keyByte)
		if !found {
			return zero, false
		}
		// If either n has a value or more than one child, n itself cannot be pruned.
		// If so, move the maybe-pruned subtree to n.children[index].
		if n.isTerminal || len(n.children) > 1 {
			prune, pruneIndex = n, index
		}
		n = n.children[index]
	}
	// n = found key
	if !n.isTerminal {
		return zero, false
	}
	prev := n.value
	n.value = zero
	n.isTerminal = false
	if len(n.children) == 0 {
		prune.children = append(prune.children[:pruneIndex], prune.children[pruneIndex+1:]...)
	}
	return prev, true
}

// An iter.Seq of these is returned from the adjFunction used internally by Range.
// key = path from root to node
// It is cached here for efficiency, otherwise an iter.Seq of []*ptrTrieNode[V] would be used directly.
// Note that the key must be cloned when yielded from Range.
type ptrTrieRangePath[V any] struct {
	node *ptrTrieNode[V]
	key  []byte
}

func (n *ptrTrieNode[V]) Range(bounds Bounds) iter.Seq2[[]byte, V] {
	bounds = bounds.Clone()
	root := ptrTrieRangePath[V]{n, []byte{}}
	var pathItr iter.Seq[*ptrTrieRangePath[V]]
	if bounds.IsReverse() {
		pathItr = postOrder(&root, ptrTrieReverseAdj[V](bounds))
	} else {
		pathItr = preOrder(&root, ptrTrieForwardAdj[V](bounds))
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

func ptrTrieForwardAdj[V any](bounds Bounds) adjFunction[*ptrTrieRangePath[V]] {
	// Sometimes a child is not within the bounds, but one of its descendants is.
	return func(path *ptrTrieRangePath[V]) iter.Seq[*ptrTrieRangePath[V]] {
		start, stop, ok := bounds.childBounds(path.key)
		if !ok {
			// Unreachable because of how the trie is traversed forward.
			panic("unreachable")
		}
		return func(yield func(*ptrTrieRangePath[V]) bool) {
			for _, child := range path.node.children {
				keyByte := child.keyByte
				if keyByte < start {
					continue
				}
				if keyByte > stop {
					return
				}
				if !yield(&ptrTrieRangePath[V]{child, append(path.key, keyByte)}) {
					return
				}
			}
		}
	}
}

func ptrTrieReverseAdj[V any](bounds Bounds) adjFunction[*ptrTrieRangePath[V]] {
	// Sometimes a child is not within the bounds, but one of its descendants is.
	return func(path *ptrTrieRangePath[V]) iter.Seq[*ptrTrieRangePath[V]] {
		start, stop, ok := bounds.childBounds(path.key)
		if !ok {
			return emptySeq
		}
		return func(yield func(*ptrTrieRangePath[V]) bool) {
			for i := len(path.node.children) - 1; i >= 0; i-- {
				child := path.node.children[i]
				keyByte := child.keyByte
				if keyByte > start {
					continue
				}
				if keyByte < stop {
					return
				}
				if !yield(&ptrTrieRangePath[V]{child, append(path.key, keyByte)}) {
					return
				}
			}
		}
	}
}

func (n *ptrTrieNode[V]) String() string {
	var s strings.Builder
	n.printNode(&s, "")
	return s.String()
}

//nolint:revive
func (n *ptrTrieNode[V]) printNode(s *strings.Builder, indent string) {
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

func (n *ptrTrieNode[V]) search(byt byte) (int, bool) {
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
