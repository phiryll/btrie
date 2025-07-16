package kv_test

import (
	"bytes"
	"fmt"
	"iter"
	"slices"
	"strings"
	"testing"

	"github.com/phiryll/kv"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

type (
	ByteStore = kv.Store[byte]
	Bounds    = kv.Bounds
	keySet    = [][]byte // instances will generally have unique keys

	// Used when benchmarking to force state changes.
	// Currently only implemented by reference.
	dirtyable interface {
		makeDirty()
		refresh()
	}

	implDef struct {
		name    string
		factory func() ByteStore
	}

	// A description of a store to be tested or benchmarked.
	// Some fields may not be populated depending on the use case.
	//
	// For benchmarking, there is one storeConfig for each corpus used by all benchmarks,
	// shared by all store implementations.
	storeConfig struct {
		name string
		size int
		ref  *reference
	}

	// A named store created by [storeUnderTest.def], with entries from [storeUnderTest.config].
	storeUnderTest struct {
		name   string
		def    *implDef
		config *storeConfig
		ByteStore
	}
)

const (
	zero = byte(0)
)

var (
	implDefs = []*implDef{
		{"impl=reference", func() ByteStore { return newReference() }},
		{"impl=pointer-trie", kv.NewPointerTrie[byte]},
		{"impl=array-trie", kv.NewArrayTrie[byte]},
	}

	From       = kv.From
	forwardAll = From(nil).To(nil)
	reverseAll = From(nil).DownTo(nil)

	// Keys used to build test stores.
	// These are in lexicographical order.
	testPresentKeys = keySet{
		{},
		{0},
		{0x23},
		{0x23, 0},
		{0x23, 0xA5},
		{0x23, 0xA6},
		{0xC5},
		{0xC5, 0},
		{0xC5, 0x42},
		{0xC5, 0x43},
	}
)

// nextKey and prevKey may get promoted to exported functions if there's a good use case.
// Probably only if allowing an in-progress iteration to be advanced (seek).

func nextKey(key []byte) []byte {
	if key == nil {
		panic("key must be non-nil")
	}
	result := make([]byte, len(key)+1)
	copy(result, key)
	result[len(key)] = 0x00
	return result
}

// This may not return the immediate predecessor, since a unique one might not exist.
// For example, {A, B, 0xFF} < {A, B, 0xFF, 0xFF} < ... < {A, B+1}.
func prevKey(key []byte) []byte {
	keyLen := len(key)
	lastByte := key[keyLen-1]
	if lastByte == 0x00 {
		// Return the key with the last byte removed.
		result := make([]byte, keyLen-1)
		copy(result, key)
		return result
	}
	// Return the key with the last byte decremented and an 0xFF added.
	result := make([]byte, keyLen+1)
	copy(result, key[:keyLen-1])
	result[keyLen-1] = lastByte - 1
	result[keyLen] = 0xFF
	return result
}

func TestNextKey(t *testing.T) {
	t.Parallel()
	assert.Panics(t, func() {
		nextKey(nil)
	})
	for _, tt := range []struct {
		key     []byte
		nextKey []byte
	}{
		{[]byte{}, []byte{0}},
		{[]byte{0x23, 0x87, 0x00}, []byte{0x23, 0x87, 0x00, 0x00}},
		{[]byte{0x23, 0x87, 0x12}, []byte{0x23, 0x87, 0x12, 0x00}},
		{[]byte{0x23, 0x87, 0xFF}, []byte{0x23, 0x87, 0xFF, 0x00}},
	} {
		copyKey := append([]byte{}, tt.key...)
		assert.Equal(t, tt.nextKey, nextKey(tt.key))
		// make sure nextKey didn't mutate its arg
		assert.Equal(t, copyKey, tt.key)
	}
}

func TestPrevKey(t *testing.T) {
	t.Parallel()
	assert.Panics(t, func() {
		prevKey(nil)
	})
	assert.Panics(t, func() {
		prevKey([]byte{})
	})
	for _, tt := range []struct {
		key     []byte
		prevKey []byte
	}{
		{[]byte{0}, []byte{}},
		{[]byte{0x23, 0x87, 0x00, 0x00}, []byte{0x23, 0x87, 0x00}},
		{[]byte{0x23, 0x87, 0x00}, []byte{0x23, 0x87}},
		{[]byte{0x23, 0x87, 0x12}, []byte{0x23, 0x87, 0x11, 0xFF}},
		{[]byte{0x23, 0x87, 0xFF}, []byte{0x23, 0x87, 0xFE, 0xFF}},
	} {
		copyKey := append([]byte{}, tt.key...)
		assert.Equal(t, tt.prevKey, prevKey(tt.key))
		// make sure prevKey didn't mutate its arg
		assert.Equal(t, copyKey, tt.key)
	}
}

// Recreates s.store.
func (s *storeUnderTest) resetFromConfig() {
	store := s.def.factory()
	for k, v := range s.config.ref.Asc(nil, nil) {
		store.Set(k, v)
	}
	s.ByteStore = store
}

func singleton[V any](value V) iter.Seq[V] {
	return func(yield func(V) bool) {
		yield(value)
	}
}

// Returns an iterator over the keys of itr.
func keyIter[K, V any](itr iter.Seq2[K, V]) iter.Seq[K] {
	return func(yield func(K) bool) {
		for k := range itr {
			if !yield(k) {
				return
			}
		}
	}
}

// Assumes a < b. Returns a copy of a if no distinct midpoint is possible.
func midPoint(a, b []byte) []byte {
	mid := make([]byte, len(a))
	copy(mid, a)
	for i := range a {
		if a[i] == b[i] {
			continue
		}
		if a[i]+1 == b[i] {
			// Keep mid[i] = a[i], guaranteeing mid < b. Increase the remaining bytes.
			for j := i + 1; j < len(a); j++ {
				mid[j] += (0xFF - a[j]) / 2
			}
			// Handles the case when the remaining bytes were all 0xFE or 0xFF.
			return append(mid, 0x00)
		}
		// a[i] and b[i] differ by at least 2
		mid[i] += (b[i] - a[i]) / 2
		return mid
	}
	// If we reach here, a is a prefix of b.
	for _, byt := range b[len(a):] {
		mid = append(mid, byt/2)
	}
	// This happens if b is a followed by only zeroes. Truncate mid a bit.
	if bytes.Equal(mid, b) {
		return mid[:(len(a)+len(b))/2]
	}
	return mid
}

// Returns an ascending sequence of keys not in the given store.
// The logic is roughly:
//
//	yield empty key
//	for key in store.Asc(nil, nil) {
//		yield midPoint between the last yielded key and key
//		yield prevKey(key)
//		yield nextKey(key)
//	}
//	yield midPoint between the largest key and {FF, FF, ...}
//	yield {FF, FF, ...}
//
//nolint:gocognit
func absentKeys[V any](store kv.Store[V]) iter.Seq[[]byte] {
	return func(yield func([]byte) bool) {
		// The previously yielded key.
		lastYield := []byte{}
		if _, ok := store.Get(lastYield); !ok {
			if !yield(lastYield) {
				return
			}
		}
		for k := range store.Range(forwardAll) {
			// Compute the midpoint between lastYield and k, for some definition of "midpoint".
			// Note that lastYield < k, unless k is empty, because...
			// (the code below assumes this is true)
			//
			// If no keys have been yielded yet, this is trivially true (lastYield is empty).
			// So assume a key has been yielded,
			// although lastYield could still be empty if store.Asc starts with {[0], [0, 0], ...}).
			// There is no key X such that K < X < next(K) for any key K (follows from nextKey's implementation).
			//
			// Let K0 be the previously returned key from the store.Asc iterator, and K1 the current key. K0 < K1.
			// K0 exists, because some key has been yielded.
			// lastYield <= nextKey(K0), because that's the largest of the 3 possible keys (mid, prev, next).
			// K0 < nextKey(K0) <= K1, because there are no keys between K0 and nextKey(K0).
			// So we have lastYield <= nextKey(K0) <= K1, and we're trying to show lastYield < K1.
			// Equivalently, we're trying to show lastYield != K1.
			// But lastYield can't be K1, since no keys present in store are ever yielded.
			keys := [][]byte{nextKey(k)}
			if len(k) > 0 {
				keys = append(keys, midPoint(lastYield, k), prevKey(k))
			}
			slices.SortFunc(keys, bytes.Compare)
			for _, key := range keys {
				if bytes.Compare(key, lastYield) <= 0 {
					continue
				}
				if _, ok := store.Get(key); ok {
					continue
				}
				if !yield(key) {
					return
				}
				lastYield = key
			}
		}
		final := bytes.Repeat([]byte{0xFF}, len(lastYield)+1)
		if !yield(midPoint(lastYield, final)) {
			return
		}
		yield(final)
	}
}

// Returns present + absent + +/-Inf, sorted.
func boundKeys[V any](store kv.Store[V]) [][]byte {
	keys := slices.Collect(keyIter(store.Range(forwardAll)))
	keys = slices.AppendSeq(keys, absentKeys(store))
	slices.SortFunc(keys, bytes.Compare)
	keys = slices.CompactFunc(keys, bytes.Equal)
	nilKey := []byte(nil)
	keys = append([][]byte{nilKey}, keys...)
	keys = append(keys, nilKey)
	return keys
}

// keys must be sorted.
func rangePairs(keys [][]byte) iter.Seq2[[]byte, []byte] {
	return func(yield func([]byte, []byte) bool) {
		for i, low := range keys {
			for _, high := range keys[i+1:] {
				if !yield(low, high) {
					return
				}
			}
		}
	}
}

func subsequences(n int) iter.Seq2[string, iter.Seq[int]] {
	// Limiting this because 32 would result in 2^32 subsequences.
	if n >= 32 {
		panic(fmt.Sprintf("%d is too large", n))
	}
	return func(yieldSubSeq func(string, iter.Seq[int]) bool) {
		// keyBits loops through bit pattern, aka keyBits, for n bits.
		// Each 1 bit corresponds to an element of this subsequence.
		for keyBits := range 1 << n {
			name := fmt.Sprintf("%0*b", n, keyBits)
			subSeq := func(yieldValue func(int) bool) {
				mask := 0x01
				for i := range n {
					if keyBits&mask != 0 && !yieldValue(i) {
						return
					}
					mask <<= 1
				}
			}
			if !yieldSubSeq(name, subSeq) {
				return
			}
		}
	}
}

func createTestStoreConfig(corpusName string, maxSize int, itr iter.Seq2[[]byte, byte]) *storeConfig {
	size := 0
	ref := newReference()
	for k, v := range itr {
		if _, ok := ref.Set(k, v); !ok {
			size++
		}
		if maxSize > 0 && size == maxSize {
			break
		}
	}
	ref.refresh()
	return &storeConfig{
		fmt.Sprintf("corpus=%s/size=%d", corpusName, size),
		size,
		ref,
	}
}

// storeConfigs for all possible subsequences of presentKeys.
func createTestStoreConfigs() iter.Seq[*storeConfig] {
	return func(yield func(*storeConfig) bool) {
		for name, indexIter := range subsequences(len(testPresentKeys)) {
			keyValueItr := func(yieldKeyValue func([]byte, byte) bool) {
				for i := range indexIter {
					if !yieldKeyValue(testPresentKeys[i], byte(i)) {
						return
					}
				}
			}
			if !yield(createTestStoreConfig(name, -1, keyValueItr)) {
				return
			}
		}
	}
}

func createStoresUnderTest(storeConfigs iter.Seq[*storeConfig]) iter.Seq[*storeUnderTest] {
	return func(yield func(*storeUnderTest) bool) {
		for config := range storeConfigs {
			for _, def := range implDefs {
				store := storeUnderTest{config.name + "/" + def.name, def, config, nil}
				store.resetFromConfig()
				if !yield(&store) {
					return
				}
			}
		}
	}
}

/*
func assertPresent(t *testing.T, key []byte, value byte, store ByteStore) {
	actual, ok := store.Get(key)
	assert.True(t, ok)
	assert.Equal(t, value, actual)
	for k, v := range store.Range(forwardAll) {
		if bytes.Equal(key, k) {
			assert.Equal(t, value, v)
			break
		}
	}
	for k, v := range store.Range(reverseAll) {
		if bytes.Equal(key, k) {
			assert.Equal(t, value, v)
			break
		}
	}
}
*/

func assertAbsent(t *testing.T, key []byte, store ByteStore) {
	actual, ok := store.Get(key)
	assert.False(t, ok)
	assert.Equal(t, zero, actual)
	actual, ok = store.Delete(key)
	assert.False(t, ok)
	assert.Equal(t, zero, actual)
	for k := range store.Range(forwardAll) {
		assert.NotEqual(t, key, k)
	}
	for k := range store.Range(reverseAll) {
		assert.NotEqual(t, key, k)
	}
}

// Test that store contains only the key/value pairs in entries,
// and that Range(forward/reverse) returns them in the correct order.
func assertSame(t *testing.T, expected *reference, actual ByteStore) {
	for k, v := range expected.All() {
		actual, ok := actual.Get(k)
		assert.True(t, ok)
		assert.Equal(t, v, actual)
	}
	assertItersEqual(t, expected.Range(forwardAll), actual.Range(forwardAll))
	assertItersEqual(t, expected.Range(reverseAll), actual.Range(reverseAll))
}

func TestNilArgPanics(t *testing.T) {
	t.Parallel()
	for _, def := range implDefs {
		t.Run(def.name, func(t *testing.T) {
			t.Parallel()
			store := def.factory()
			assert.Panics(t, func() {
				store.Set(nil, 0)
			})
			assert.Panics(t, func() {
				store.Get(nil)
			})
			assert.Panics(t, func() {
				store.Delete(nil)
			})
			assert.Panics(t, func() {
				store.Range(nil)
			})
		})
	}
}

// Tests Get/Set/Delete/Range with a specific key and store, which should not contain key.
// The store should be the same after invoking this function.
// Assumes store.Range(forwardAll) works.
func testKey(t *testing.T, key []byte, store ByteStore) {
	const value = byte(43)
	const replacement = byte(57)
	ref := newReference()
	for k, v := range store.Range(forwardAll) {
		ref.Set(k, v)
	}
	_, ok := ref.Get(key)
	require.False(t, ok)

	assertAbsent(t, key, store)
	assertSame(t, ref, store)

	ref.Set(key, value)
	actual, ok := store.Set(key, value)
	assert.False(t, ok)
	assert.Equal(t, zero, actual)
	assertSame(t, ref, store)

	ref.Set(key, replacement)
	actual, ok = store.Set(key, replacement)
	assert.True(t, ok)
	assert.Equal(t, value, actual)
	assertSame(t, ref, store)

	ref.Delete(key)
	actual, ok = store.Delete(key)
	assert.True(t, ok)
	assert.Equal(t, replacement, actual)
	assertAbsent(t, key, store)
	assertSame(t, ref, store)
}

// The empty key is often a special case in an implementation.
func TestEmptyKey(t *testing.T) {
	t.Parallel()
	key := []byte{}
	for _, def := range implDefs {
		t.Run(def.name, func(t *testing.T) {
			t.Parallel()
			store := def.factory()
			testKey(t, key, store)
			store.Set([]byte{43, 15}, 94)
			store.Set([]byte{126, 73, 12}, 45)
			testKey(t, key, store)
		})
	}
}

// If String() exists, make sure it doesn't crash.
func TestString(t *testing.T) {
	t.Parallel()
	for _, def := range implDefs {
		t.Run(def.name, func(t *testing.T) {
			t.Parallel()
			store := def.factory()
			if sStore, ok := store.(fmt.Stringer); ok {
				assert.NotPanics(t, func() { _ = sStore.String() })
				store.Set([]byte{}, 73)
				assert.NotPanics(t, func() { _ = sStore.String() })
				store.Set([]byte{43, 15}, 94)
				store.Set([]byte{126, 73, 12}, 45)
				assert.NotPanics(t, func() { _ = sStore.String() })
			}
		})
	}
}

func checkFprint[V any](t *testing.T, expected string, seq iter.Seq2[[]byte, V]) {
	t.Helper()
	var s strings.Builder
	n, err := kv.Fprint(&s, seq)
	require.NoError(t, err)
	assert.Equal(t, n, s.Len())
	assert.Equal(t, expected, s.String())
}

func TestFprint(t *testing.T) {
	t.Parallel()
	for _, def := range implDefs {
		t.Run(def.name, func(t *testing.T) {
			t.Parallel()
			store := def.factory()
			checkFprint(t, "", store.Range(From(nil).To(nil)))
			checkFprint(t, "", store.Range(From(nil).DownTo(nil)))

			store.Set([]byte{8, 6, 2}, 45)
			store.Set([]byte{1, 2}, 47)
			store.Set([]byte{8, 6, 5}, 53)
			store.Set([]byte{}, 35)
			store.Set([]byte{1, 1, 7, 3, 12}, 16)
			store.Set([]byte{8, 6, 4, 2}, 71)
			store.Set([]byte{1, 1}, 83)

			checkFprint(t, `: 35
0101: 83
. . 07030C: 16
. 02: 47
080602: 45
. . 0402: 71
. . 05: 53
`, store.Range(From(nil).To(nil)))

			checkFprint(t, `080605: 53
. . 0402: 71
. . 02: 45
0102: 47
. 0107030C: 16
. . : 83
: 35
`, store.Range(From(nil).DownTo(nil)))

			// no keys in range
			checkFprint(t, "", store.Range(From([]byte{1, 3}).To([]byte{8, 6, 1})))
			checkFprint(t, "", store.Range(From([]byte{8, 6, 1}).DownTo([]byte{1, 3})))
		})
	}
}

// Functions to test iterators.

func msgFrom(msgAndArgs ...any) string {
	if len(msgAndArgs) == 0 {
		return ""
	}
	msg, ok := msgAndArgs[0].(string)
	if !ok {
		return fmt.Sprintf("%+v of type %T must be a format string", msgAndArgs[0], msgAndArgs[0])
	}
	return fmt.Sprintf(msg, msgAndArgs[1:]...)
}

func assertItersEqual[K, V any](t *testing.T, expected, actual iter.Seq2[K, V], msgAndArgs ...any) {
	t.Helper()
	msg := msgFrom(msgAndArgs...)
	i := 0
	next, stop := iter.Pull2(actual)
	defer stop()
	for expectedKey, expectedValue := range expected {
		actualKey, actualValue, ok := next()
		require.True(t, ok, "%s: too short, len == %d", msg, i)
		assert.Equal(t, expectedKey, actualKey, "%s: keys at index %d differ", msg, i)
		assert.Equal(t, expectedValue, actualValue, "%s: values at index %d  differ", msg, i)
		i++
	}
	_, _, ok := next()
	assert.False(t, ok, "%s: too long, len > %d", msg, i)
}

// This tests that an early yield does not fail,
// and ensures those code paths get test coverage.
func assertEarlyYield(t *testing.T, itr iter.Seq2[[]byte, byte]) {
	const maxCount = 4
	t.Helper()
	count := 0
	for range itr {
		if count >= maxCount {
			break
		}
		count++
	}
	assert.LessOrEqual(t, count, maxCount)
}

func TestStores(t *testing.T) {
	t.Parallel()
	for store := range createStoresUnderTest(createTestStoreConfigs()) {
		t.Run(store.name, func(t *testing.T) {
			t.Parallel()

			// Build the store, testing along the way.
			s := store.def.factory()
			ref := newReference()
			for k, v := range store.config.ref.All() {
				t.Run("op=set/key="+kv.KeyName(k), func(t *testing.T) {
					assertAbsent(t, k, s)
					assertSame(t, ref, s)

					actual, ok := s.Set(k, v)
					assert.False(t, ok)
					assert.Equal(t, zero, actual)
					ref.Set(k, v)
					assertSame(t, ref, s)
				})
			}

			for k := range absentKeys(store.config.ref) {
				t.Run("op=absent/key="+kv.KeyName(k), func(t *testing.T) {
					assertAbsent(t, k, s)
				})
			}

			t.Run("op=range", func(t *testing.T) {
				for low, high := range rangePairs(boundKeys(store.config.ref)) {
					forward := From(low).To(high)
					reverse := From(high).DownTo(low)
					assertItersEqual(t, store.config.ref.Range(forward), s.Range(forward), "%s", forward)
					assertItersEqual(t, store.config.ref.Range(reverse), s.Range(reverse), "%s", reverse)
				}
				assertEarlyYield(t, s.Range(forwardAll))
				assertEarlyYield(t, s.Range(reverseAll))
			})
		})
	}
}

// Things that failed at one point or another during testing.

func assertIterEmpty[V any](t *testing.T, actual iter.Seq2[[]byte, V]) bool {
	t.Helper()
	for key, value := range actual {
		return assert.Fail(t, fmt.Sprintf("should be empty, contains {%s:%v}", kv.KeyName(key), value))
	}
	return true
}

func assertIterSingleton[V any](t *testing.T, expectedKey []byte, expectedValue V, actual iter.Seq2[[]byte, V]) bool {
	t.Helper()
	next, stop := iter.Pull2(actual)
	defer stop()
	key, value, ok := next()
	assert.True(t, ok, "should not be empty")
	assert.Equal(t, expectedKey, key)
	assert.Equal(t, expectedValue, value)
	_, _, ok = next()
	assert.False(t, ok, "should have exactly one entry")
	return true
}

func TestFail1(t *testing.T) {
	t.Parallel()
	for _, def := range implDefs {
		t.Run(def.name, func(t *testing.T) {
			t.Parallel()
			store := def.factory()
			store.Set([]byte{5}, 0)
			assertIterEmpty(t, store.Range(From([]byte{5, 0}).To([]byte{6})))
			assertIterSingleton(t, []byte{5}, 0, store.Range(From([]byte{4}).To([]byte{5, 0})))
		})
	}
}

func TestFail2(t *testing.T) {
	t.Parallel()
	for _, def := range implDefs {
		t.Run(def.name, func(t *testing.T) {
			t.Parallel()
			store := def.factory()
			store.Set([]byte{0xB3, 0x9C}, 184)

			// forgot to check isTerminal
			actual, actualOk := store.Get([]byte{0xB3})
			assert.False(t, actualOk)
			assert.Equal(t, zero, actual)

			actual, actualOk = store.Get([]byte{0xB3, 0x9C})
			assert.True(t, actualOk)
			assert.Equal(t, byte(184), actual)
		})
	}
}

func TestFail3(t *testing.T) {
	t.Parallel()
	for _, def := range implDefs {
		t.Run(def.name, func(t *testing.T) {
			t.Parallel()
			store := def.factory()
			store.Set([]byte{0xB3, 0x9C}, 184)

			actual, actualOk := store.Delete([]byte{0xB3})
			assert.False(t, actualOk)
			assert.Equal(t, zero, actual)

			// Make sure the subtree wasn't deleted.
			actual, actualOk = store.Get([]byte{0xB3, 0x9C})
			assert.True(t, actualOk)
			assert.Equal(t, byte(184), actual)
		})
	}
}

func TestFail4(t *testing.T) {
	t.Parallel()
	for _, def := range implDefs {
		t.Run(def.name, func(t *testing.T) {
			t.Parallel()
			store := def.factory()
			store.Set([]byte{0x50, 0xEF}, 45)
			assertIterEmpty(t, store.Range(From([]byte{0x50}).DownTo([]byte{0x15})))
		})
	}
}

func TestFail5(t *testing.T) {
	t.Parallel()
	for _, def := range implDefs {
		t.Run(def.name, func(t *testing.T) {
			t.Parallel()
			store := def.factory()
			store.Set([]byte{0x50, 0xEF}, 45)
			assertIterSingleton(t, []byte{0x50, 0xEF}, 45, store.Range(From([]byte{0xFD}).DownTo([]byte{0x3D})))
		})
	}
}

func TestFail6(t *testing.T) {
	t.Parallel()
	for _, def := range implDefs {
		t.Run(def.name, func(t *testing.T) {
			t.Parallel()
			store := def.factory()
			store.Set([]byte{0x50, 0xEF}, 45)
			assertIterSingleton(t, []byte{0x50, 0xEF}, 45, store.Range(From([]byte{0x51}).DownTo([]byte{0x50})))
		})
	}
}

func TestFail7(t *testing.T) {
	// Failure is due to continuing iteration past false yield().
	// Failure requires the second Set.
	t.Parallel()
	for _, def := range implDefs {
		t.Run(def.name, func(t *testing.T) {
			t.Parallel()
			store := def.factory()
			store.Set([]byte{3}, 0)
			store.Set([]byte{4}, 0)
			assertIterEmpty(t, store.Range(From([]byte{2}).DownTo([]byte{1})))
		})
	}
}

func TestFail8(t *testing.T) {
	t.Parallel()
	for _, def := range implDefs {
		t.Run(def.name, func(t *testing.T) {
			t.Parallel()
			store := def.factory()
			store.Set([]byte{}, 1)
			store.Set([]byte{0}, 3)
			store.Set([]byte{0x23}, 4)
			store.Set([]byte{0x23, 0}, 5)
			store.Set([]byte{0x23, 0xA5}, 6)
			assertIterSingleton(t, []byte{0x23, 0}, 5, store.Range(From([]byte{0x23, 0}).To([]byte{0x23, 0, 0})))
		})
	}
}

func TestFail9(t *testing.T) {
	// Test that removing the last value on a path removes the path.
	// Definite hack to detect this one,
	// but there's no good way to test this using the public API.
	// The alternative would be to have implementation-specific tests,
	// which is probably a better approach, but this works for now.
	t.Parallel()
	for _, def := range implDefs {
		t.Run(def.name, func(t *testing.T) {
			t.Parallel()
			store := def.factory()
			sStore, ok := store.(fmt.Stringer)
			if !ok {
				t.Skipf("%T does not implement Stringer", store)
			}
			expected := sStore.String()
			key := []byte{0x23}
			store.Set(key, 6)
			store.Delete(key)
			if d, ok := store.(dirtyable); ok {
				d.refresh()
			}
			assert.Equal(t, expected, sStore.String())
		})
	}
}
