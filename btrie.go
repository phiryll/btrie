// Package btrie defines interfaces for and provides multiple implementations of a binary trie.
package btrie

// nil keys and values are not allowed, which allows nil to mean "does not exist".
// empty keys are not allowed, must be at least one byte, at least for now.

// see if it's worthwhile to upgrade to go 1.23 to use the iter package
// probably a good idea to at least follow the structure and naming conventions

// How do you get a handle to a specific version?
// This isn't necessary for the neither persistent nor thread-safe variant.
// This argues for some other interface.
// Also, should read-only have another interface, or do operations panic?
// I think another interface.

// Is each Put a new version, or is there some concept of transaction?

// Implement, in order???
// - neither persistent nor thread-safe
//   as simple as possible, linked nodes first, then tables
//   this is first to get most of the API right
// - thread-safe
//   does not allow reading previous versions,
//   but does allow continuing to read a version that you have the handle of
//   so the operations aren't thread-safe, but the entire snapshot is
// - fully persistent and thread-safe
//   every version can accessed and be modified
//   this is only useful in my personal use cases if there's a merge operation,
//   which would make this confluently persistent
// - partially persistent and thread-safe
//   all versions can be accessed but only newest can be modified

// BTrie is ....
type BTrie interface {
	// Potential methods, still fleshing this out.

	Put(key, value []byte) (previous []byte)
	Get(key []byte) []byte
	Delete(key []byte) (previous []byte)

	// if begin > end, go backwards, or separate method?
	Range(begin, end []byte) Cursor

	// PutIfAbsent(key, value []byte) (previous []byte)
	// DeleteIfPresent(key, value []byte) (found bool)
}

// Cursor is what is returned by [BTrie.Range].
type Cursor interface {
	HasNext() bool
	Next() Entry
}

// Entry is what is returned by [Cursor.Next].
type Entry struct {
	Key   []byte
	Value []byte
}

// NewSimple returns a new, absurdly simple, and badly coded BTrie.
// This is purely for fleshing out the unit tests, benchmarks, and fuzz tests.
func NewSimple() BTrie {
	return &node{nil, nil, 0}
}
