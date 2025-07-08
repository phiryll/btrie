package kv_test

import (
	"fmt"
	"iter"
	"math/bits"
	"strings"
	"testing"

	"github.com/phiryll/kv"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

type (
	TestStore = kv.Cloneable[byte]
	Bounds    = kv.Bounds
	keySet    = [][]byte // instances will generally have unique keys

	implDef struct {
		name    string
		factory func() TestStore
	}

	// A description of a store to be tested or benchmarked.
	// Some fields may not be populated depending on the use case.
	//
	// For benchmarking, there is one storeConfig with a given size used by all benchmarks,
	// shared by all store implementations.
	storeConfig struct {
		name string
		size int
		ref  *reference

		present, absent keySet

		// forward/reverse Bounds instances to test Range.
		forward, reverse []Bounds
	}

	// A store created by a def.factory(), possibly with entries from config.
	// For some tests, an empty store might be created and config might be nil.
	testStore struct {
		name   string
		store  TestStore
		def    *implDef
		config *storeConfig
	}
)

const (
	zero = byte(0)
)

var (
	implDefs = []*implDef{
		{"impl=reference", func() TestStore { return TestStore(newReference()) }},
		{"impl=pointer-trie", asCloneable(kv.NewPointerTrie[byte])},
		{"impl=array-trie", asCloneable(kv.NewArrayTrie[byte])},
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

	// Non-empty keys very near presentTestKeys, but not in presentTestKeys.
	testAbsentKeys = keySet{
		{0, 0},
		{0x22, 0xFF},
		{0x23, 0, 0},
		{0x23, 0xA4, 0xFF},
		{0x23, 0xA5, 0},
		{0x23, 0xA5, 0xFF},
		{0x23, 0xA6, 0},
		{0xC4, 0xFF},
		{0xC5, 0, 0},
		{0xC5, 0x41, 0xFF},
		{0xC5, 0x42, 0},
		{0xC5, 0x42, 0xFF},
		{0xC5, 0x43, 0},
	}

	// presentTestKeys + absentTestKeys + +/-Inf.
	// Except for the nils, these are in lexicographical order.
	testNearKeys = keySet{
		nil, // -Inf
		{},
		{0},
		{0, 0},
		{0x22, 0xFF},
		{0x23},
		{0x23, 0},
		{0x23, 0, 0},
		{0x23, 0xA4, 0xFF},
		{0x23, 0xA5},
		{0x23, 0xA5, 0},
		{0x23, 0xA5, 0xFF},
		{0x23, 0xA6},
		{0x23, 0xA6, 0},
		{0xC4, 0xFF},
		{0xC5},
		{0xC5, 0},
		{0xC5, 0, 0},
		{0xC5, 0x41, 0xFF},
		{0xC5, 0x42},
		{0xC5, 0x42, 0},
		{0xC5, 0x42, 0xFF},
		{0xC5, 0x43},
		{0xC5, 0x43, 0},
		nil, // +Inf
	}

	testStoreConfigs = createTestStoreConfigs()
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

func asCloneable(factory func() kv.Store[byte]) func() TestStore {
	return func() TestStore {
		store := factory()
		cloneable, ok := store.(TestStore)
		if !ok {
			panic(fmt.Sprintf("%T is not Cloneable", store))
		}
		return cloneable
	}
}

// storeConfigs for all possible subsequences of presentKeys.
func createTestStoreConfigs() []*storeConfig {
	result := []*storeConfig{}

	// Every storeConfig here gets the same set of test Bounds.
	var forward, reverse []Bounds
	for i, low := range testNearKeys {
		for _, high := range testNearKeys[i+1:] {
			forward = append(forward, *From(low).To(high))
			reverse = append(reverse, *From(high).DownTo(low))
		}
	}

	// Every bit pattern of i defines which keys are present in that config.
	for keyBits := range 1 << len(testPresentKeys) {
		config := storeConfig{
			fmt.Sprintf("sub-store=%0*b", len(testPresentKeys), keyBits),
			bits.OnesCount(uint(keyBits)),
			newReference(),
			keySet{},
			keySet{},
			forward,
			reverse,
		}
		mask := 0x01
		for i, k := range testPresentKeys {
			if keyBits&mask != 0 {
				config.ref.Set(k, byte(i))
				config.present = append(config.present, k)
			} else {
				config.absent = append(config.absent, k)
			}
			mask <<= 1
		}
		config.absent = append(config.absent, testAbsentKeys...)
		result = append(result, &config)
	}
	return result
}

func createTestStores(storeConfigs []*storeConfig) []*testStore {
	result := []*testStore{}
	for _, config := range storeConfigs {
		for _, def := range implDefs {
			store := def.factory()
			for k, v := range config.ref.All() {
				store.Set(k, v)
			}
			name := config.name + "/" + def.name
			result = append(result, &testStore{name, store, def, config})
		}
	}
	return result
}

/*
func assertPresent(t *testing.T, key []byte, value byte, store TestStore) {
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

func assertAbsent(t *testing.T, key []byte, store TestStore) {
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
func assertSame(t *testing.T, expected *reference, actual TestStore) {
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
func testKey(t *testing.T, key []byte, store TestStore) {
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
func TestStoreString(t *testing.T) {
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
func assertEarlyYield(t *testing.T, size int, itr iter.Seq2[[]byte, byte]) {
	t.Helper()
	expectedCount := size
	if expectedCount > 4 {
		expectedCount = 4
	}
	count := 0
	for range itr {
		if count > 3 {
			break
		}
		count++
	}
	assert.Equal(t, expectedCount, count)
}

func TestStores(t *testing.T) {
	t.Parallel()
	for _, test := range createTestStores(testStoreConfigs) {
		t.Run(test.name, func(t *testing.T) {
			t.Parallel()

			// Build the store, testing along the way.
			store := test.def.factory()
			ref := newReference()
			for k, v := range test.config.ref.All() {
				t.Run("op=set/key="+kv.KeyName(k), func(t *testing.T) {
					assertAbsent(t, k, store)
					assertSame(t, ref, store)

					actual, ok := store.Set(k, v)
					assert.False(t, ok)
					assert.Equal(t, zero, actual)
					ref.Set(k, v)
					assertSame(t, ref, store)
				})
			}

			for _, k := range test.config.absent {
				t.Run("op=absent/key="+kv.KeyName(k), func(t *testing.T) {
					assertAbsent(t, k, store)
				})
			}

			t.Run("op=range", func(t *testing.T) {
				for _, bounds := range test.config.forward {
					assertItersEqual(t, test.config.ref.Range(&bounds), store.Range(&bounds), "%s", &bounds)
				}
				for _, bounds := range test.config.reverse {
					assertItersEqual(t, test.config.ref.Range(&bounds), store.Range(&bounds), "%s", &bounds)
				}
				assertEarlyYield(t, test.config.size, store.Range(forwardAll))
				assertEarlyYield(t, test.config.size, store.Range(reverseAll))
			})
		})
	}
}

func TestClone(t *testing.T) {
	t.Parallel()
	for _, test := range createTestStores(testStoreConfigs) {
		t.Run(test.name, func(t *testing.T) {
			t.Parallel()
			original := test.store
			assertSame(t, test.config.ref, original)

			// test that the clone was correct
			store := original.Clone()
			assertSame(t, test.config.ref, store)

			// mutate the clone and test that original hasn't changed
			for k := range test.config.ref.All() {
				store.Delete(k)
			}
			assertIterEmpty(t, store.Range(forwardAll))
			for i, k := range test.config.absent {
				store.Set(k, byte(i))
			}
			assertSame(t, test.config.ref, original)

			// mutate the original and test that the clone hasn't changed
			store = original.Clone()
			for k := range test.config.ref.All() {
				original.Delete(k)
			}
			assertIterEmpty(t, original.Range(forwardAll))
			for i, k := range test.config.absent {
				original.Set(k, byte(i))
			}
			assertSame(t, test.config.ref, store)
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
			// make sure reference.dirty is false
			for range store.Range(forwardAll) {
				break
			}
			assert.Equal(t, expected, sStore.String())
		})
	}
}
