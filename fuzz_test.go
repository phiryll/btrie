package kv_test

import (
	"bytes"
	"encoding/binary"
	"maps"
	rand "math/rand/v2"
	"slices"
	"testing"

	"github.com/phiryll/kv"
	"github.com/stretchr/testify/assert"
)

// Go's fuzzing can generate []byte inputs, but not with a length constraint.
// Because of that, the fuzzed keys here will the bytes of a uint32,
// with length 0-3 determined by another fuzzed byte.
// See keyForFuzzInputs.

const (
	fuzzSize       = 1 << 20
	fuzzRangeSize  = 1 << 16 // because Range is an expensive operation
	fuzzMeanKeyLen = 4
)

var (
	// There is currently only one config in each of these slices.
	fuzzStoreConfigs      = createFuzzStoreConfigs(fuzzSize)
	fuzzRangeStoreConfigs = createFuzzStoreConfigs(fuzzRangeSize)
)

// Fuzz testing is very parallel, and stores aren't generally thread-safe.
// Ensure that instances are not shared.

// Returns a key for fuzz inputs fuzzKey and fuzzKeyLen.
// The fuzz inputs are used to generate the key, and are not themselves the key and length.
func keyForFuzzInputs(fuzzKey uint32, fuzzKeyLen byte) []byte {
	// fuzzKeyLen => keyLen of result (# out of 256)
	// 0x40-0xFF => 4 (192)
	// 0x10-0x3F => 3 ( 48)
	// 0x04-0x0F => 2 ( 12)
	// 0x01-0x03 => 1 (  3)
	// 0x00      => 0 (  1)
	keyLen := 4
	for ; keyLen > 0; keyLen-- {
		if fuzzKeyLen&(0x03<<(2*(keyLen-1))) != 0 {
			break
		}
	}
	keyBytes := binary.BigEndian.AppendUint32(nil, fuzzKey)
	return keyBytes[(4 - keyLen):] // use low-order bytes
}

func createFuzzStoreConfigs(size int) []*storeConfig {
	random := rand.New(rand.NewPCG(rand.Uint64(), rand.Uint64()))
	ref := newReference()
	for count := 0; count < size; {
		key := randomKey(fuzzMeanKeyLen, random)
		if _, ok := ref.Get(key); !ok {
			ref.Set(key, randomByte(random))
			count++
		}
	}
	return []*storeConfig{{"fuzz", size, ref}}
}

func createReferenceStore(config *storeConfig) ByteStore {
	return &reference{
		entries: maps.Clone(config.ref.entries),
		dirty:   true,
	}
}

func FuzzGet(f *testing.F) {
	fuzzStores := slices.Collect(createStoresUnderTest(slices.Values(fuzzStoreConfigs)))
	f.Fuzz(func(t *testing.T, fuzzKey uint32, fuzzKeyLen byte) {
		key := keyForFuzzInputs(fuzzKey, fuzzKeyLen)
		for _, fuzz := range fuzzStores {
			expected, expectedOk := fuzz.config.ref.Get(key)
			actual, actualOk := fuzz.Get(key)
			assert.Equal(t, expectedOk, actualOk, "%s: %s", fuzz.def.name, kv.KeyName(key))
			assert.Equal(t, expected, actual, "%s: %s", fuzz.def.name, kv.KeyName(key))
		}
	})
}

func FuzzSet(f *testing.F) {
	fuzzStores := slices.Collect(createStoresUnderTest(slices.Values(fuzzStoreConfigs)))
	// This only works because there is only one fuzz store config.
	// This is unfortunately necessary because configs are shared between these test Stores.
	// This needs to be fixed.
	ref := createReferenceStore(fuzzStoreConfigs[0])
	f.Fuzz(func(t *testing.T, fuzzKey uint32, fuzzKeyLen, value byte) {
		key := keyForFuzzInputs(fuzzKey, fuzzKeyLen)
		expected, expectedOk := ref.Set(key, value)
		for _, fuzz := range fuzzStores {
			actual, actualOk := fuzz.Set(key, value)
			assert.Equal(t, expectedOk, actualOk, "%s: %s=%d", fuzz.def.name, kv.KeyName(key), value)
			assert.Equal(t, expected, actual, "%s: %s=%d", fuzz.def.name, kv.KeyName(key), value)
			actual, ok := fuzz.Get(key)
			assert.True(t, ok, "%s: %s=%d", fuzz.def.name, kv.KeyName(key), value)
			assert.Equal(t, value, actual, "%s: %s=%d", fuzz.def.name, kv.KeyName(key), value)
		}
	})
}

func FuzzDelete(f *testing.F) {
	fuzzStores := slices.Collect(createStoresUnderTest(slices.Values(fuzzStoreConfigs)))
	// This only works because there is only one fuzz store config.
	// This is unfortunately necessary because configs are shared between these test Stores.
	// This needs to be fixed.
	ref := createReferenceStore(fuzzStoreConfigs[0])
	f.Fuzz(func(t *testing.T, fuzzKey uint32, fuzzKeyLen byte) {
		key := keyForFuzzInputs(fuzzKey, fuzzKeyLen)
		expected, expectedOk := ref.Delete(key)
		for _, fuzz := range fuzzStores {
			actual, actualOk := fuzz.Delete(key)
			assert.Equal(t, expectedOk, actualOk, "%s: %s", fuzz.def.name, kv.KeyName(key))
			assert.Equal(t, expected, actual, "%s: %s", fuzz.def.name, kv.KeyName(key))
			actual, ok := fuzz.Get(key)
			assert.False(t, ok, "%s: %s", fuzz.def.name, kv.KeyName(key))
			assert.Equal(t, byte(0), actual, "%s: %s", fuzz.def.name, kv.KeyName(key))
		}
	})
}

func FuzzRange(f *testing.F) {
	fuzzStores := slices.Collect(createStoresUnderTest(slices.Values(fuzzRangeStoreConfigs)))
	f.Fuzz(func(t *testing.T, fuzzBeginKey, fuzzEndKey uint32, fuzzBeginKeyLen, fuzzEndKeyLen byte) {
		begin := keyForFuzzInputs(fuzzBeginKey, fuzzBeginKeyLen)
		end := keyForFuzzInputs(fuzzEndKey, fuzzEndKeyLen)
		cmp := bytes.Compare(begin, end)
		if cmp == 0 {
			end = append(end, 0)
		} else if cmp > 0 {
			begin, end = end, begin
		}
		forward := From(begin).To(end)
		reverse := From(end).DownTo(begin)
		for _, fuzz := range fuzzStores {
			assertItersEqual(t, fuzz.config.ref.Range(forward), fuzz.Range(forward),
				"%s: %s", fuzz.def.name, forward)
			assertItersEqual(t, fuzz.config.ref.Range(reverse), fuzz.Range(reverse),
				"%s: %s", fuzz.def.name, reverse)
		}
	})
}

func FuzzMixed(f *testing.F) {
	fuzzStores := slices.Collect(createStoresUnderTest(slices.Values(fuzzStoreConfigs)))
	// This only works because there is only one fuzz store config.
	// This is unfortunately necessary because configs are shared between these test Stores.
	// This needs to be fixed.
	ref := createReferenceStore(fuzzStoreConfigs[0])
	f.Fuzz(func(t *testing.T, fuzzSetKey, fuzzDeleteKey uint32, fuzzSetKeyLen, fuzzDeleteKeyLen, value byte) {
		key := keyForFuzzInputs(fuzzSetKey, fuzzSetKeyLen)
		expected, expectedOk := ref.Set(key, value)
		for _, fuzz := range fuzzStores {
			actual, actualOk := fuzz.Set(key, value)
			assert.Equal(t, expectedOk, actualOk, "%s: %s=%d", fuzz.def.name, kv.KeyName(key), value)
			assert.Equal(t, expected, actual, "%s: %s=%d", fuzz.def.name, kv.KeyName(key), value)
			actual, ok := fuzz.Get(key)
			assert.True(t, ok, "%s: %s=%d", fuzz.def.name, kv.KeyName(key), value)
			assert.Equal(t, value, actual, "%s: %s=%d", fuzz.def.name, kv.KeyName(key), value)
		}

		key = keyForFuzzInputs(fuzzDeleteKey, fuzzDeleteKeyLen)
		expected, expectedOk = ref.Delete(key)
		for _, fuzz := range fuzzStores {
			actual, actualOk := fuzz.Delete(key)
			assert.Equal(t, expectedOk, actualOk, "%s: %s", fuzz.def.name, kv.KeyName(key))
			assert.Equal(t, expected, actual, "%s: %s", fuzz.def.name, kv.KeyName(key))
			actual, ok := fuzz.Get(key)
			assert.False(t, ok, "%s: %s", fuzz.def.name, kv.KeyName(key))
			assert.Equal(t, byte(0), actual, "%s: %s", fuzz.def.name, kv.KeyName(key))
		}
	})
}
