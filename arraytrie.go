package kv

import (
	"bytes"
	"fmt"
	"iter"
	"strings"
)

type arrayTrieNode[V any] struct {
	children    *[256]*arrayTrieNode[V] // only non-nil if there are children
	root        Optional[V]
	numChildren uint16 // possible values 0-256, so need the extra byte
}

// NewArrayTrie returns a new BTrie with pointers to children stored in arrays.
//
//nolint:iface
func NewArrayTrie[V any]() BTrie[V] {
	return &arrayTrieNode[V]{}
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
	return n.root.Get()
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
			child := &arrayTrieNode[V]{nil, OptionalOf(value), 0}
			for k := len(key) - 1; k > i; k-- {
				parent := &arrayTrieNode[V]{&[256]*arrayTrieNode[V]{}, Optional[V]{}, 1}
				parent.children[key[k]] = child
				child = parent
			}
			n.children[keyByte] = child
			n.numChildren++
			return zero, false
		}
		n = n.children[keyByte]
	}
	// n = found key, replace value
	return n.root.Set(value)
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
		// If either n is the root, or n has a value, or n has more than one child, then n itself cannot be pruned.
		// If so, move the maybe-pruned subtree to n.children[index].
		if i == 0 || !n.root.IsEmpty() || n.numChildren > 1 {
			prune, pruneIndex = n, keyByte
		}
		n = n.children[keyByte]
	}
	// n = found key
	value, ok := n.root.Clear()
	if ok && len(key) > 0 && n.children == nil {
		prune.children[pruneIndex] = nil
		prune.numChildren--
	}
	return value, ok
}

// An iter.Seq of these is returned from the adjFunction used internally by Range.
// key = path from root to node
// It is cached here for efficiency, otherwise an iter.Seq of []*arrayTrieNode[V] would be used directly.
// Note that the key must be cloned when yielded from Range.
type arrayTrieRangePath[V any] struct {
	node *arrayTrieNode[V]
	key  []byte
}

func (n *arrayTrieNode[V]) Range(bounds *Bounds) iter.Seq2[[]byte, V] {
	bounds = bounds.Clone()
	root := arrayTrieRangePath[V]{n, []byte{}}
	var pathItr iter.Seq[*arrayTrieRangePath[V]]
	if bounds.IsReverse {
		pathItr = postOrder(&root, arrayTrieReverseAdj[V](bounds))
	} else {
		pathItr = preOrder(&root, arrayTrieForwardAdj[V](bounds))
	}
	return func(yield func([]byte, V) bool) {
		for path := range pathItr {
			cmp := bounds.CompareKey(path.key)
			if cmp < 0 {
				continue
			}
			if cmp > 0 {
				return
			}
			if value, ok := path.node.root.Get(); ok && !yield(bytes.Clone(path.key), value) {
				return
			}
		}
	}
}

func arrayTrieForwardAdj[V any](bounds *Bounds) adjFunction[*arrayTrieRangePath[V]] {
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
			count := path.node.numChildren
			for i, child := range path.node.children[start : int(stop)+1] {
				if child == nil {
					continue
				}
				if !yield(&arrayTrieRangePath[V]{child, append(path.key, start+byte(i))}) {
					return
				}
				count--
				if count == 0 {
					return
				}
			}
		}
	}
}

func arrayTrieReverseAdj[V any](bounds *Bounds) adjFunction[*arrayTrieRangePath[V]] {
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
			count := path.node.numChildren
			for i := len(children) - 1; i >= 0; i-- {
				child := children[i]
				if child == nil {
					continue
				}
				if !yield(&arrayTrieRangePath[V]{child, append(path.key, stop+byte(i))}) {
					return
				}
				count--
				if count == 0 {
					return
				}
			}
		}
	}
}

func (n *arrayTrieNode[V]) String() string {
	var s strings.Builder
	s.WriteString("{")
	if value, ok := n.root.Get(); ok {
		fmt.Fprintf(&s, ":%v, ", value)
	}
	if n.children != nil {
		for i, child := range n.children {
			if child != nil {
				fmt.Fprintf(&s, "%02X:%s, ", byte(i), child)
			}
		}
	}
	s.WriteString("}")
	return s.String()
}
