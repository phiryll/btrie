package btrie_test

import (
	"bytes"
	"encoding/binary"
	rand "math/rand/v2"
	"testing"

	"github.com/stretchr/testify/assert"
)

// Go's fuzzing can generate []byte inputs, but not with a length constraint.
// Because of that, the fuzzed keys here will the bytes of a uint32,
// with length 0-3 determined by another fuzzed byte.
// See keyForFuzzInputs.

const (
	fuzzTrieSize      = 1 << 20
	fuzzRangeTrieSize = 1 << 16 // because Range is an expensive operation
	fuzzMeanKeyLen    = 4
)

var (
	// There is currently only one config in each of these slices.
	fuzzTrieConfigs      = createFuzzTrieConfigs(fuzzTrieSize)
	fuzzRangeTrieConfigs = createFuzzTrieConfigs(fuzzRangeTrieSize)
)

// Fuzz testing is very parallel, and tries aren't generally thread-safe.
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

func createFuzzTrieConfigs(trieSize int) []*trieConfig {
	var config trieConfig
	random := rand.New(rand.NewPCG(rand.Uint64(), rand.Uint64()))
	config.name = "fuzz"
	config.trieSize = trieSize
	config.entries = map[string]byte{}
	for count := 0; count < trieSize; {
		key := string(randomKey(fuzzMeanKeyLen, random))
		if _, ok := config.entries[key]; !ok {
			config.entries[key] = randomByte(random)
			count++
		}
	}
	return []*trieConfig{&config}
}

func TestBaseline(t *testing.T) {
	t.Parallel()
	fuzzTries := createTestTries(fuzzTrieConfigs)
	ref := createReferenceTrie(fuzzTrieConfigs[0])
	refForward := collect(ref.Range(forwardAll))
	refReverse := collect(ref.Range(reverseAll))
	for _, fuzz := range fuzzTries {
		t.Run(fuzz.name, func(t *testing.T) {
			t.Parallel()
			assert.Equal(t, refForward, collect(fuzz.trie.Range(forwardAll)), "forward")
			assert.Equal(t, refReverse, collect(fuzz.trie.Range(reverseAll)), "reverse")
		})
	}
}

func FuzzGet(f *testing.F) {
	fuzzTries := createTestTries(fuzzTrieConfigs)
	ref := createReferenceTrie(fuzzTrieConfigs[0])
	f.Fuzz(func(t *testing.T, fuzzKey uint32, fuzzKeyLen byte) {
		key := keyForFuzzInputs(fuzzKey, fuzzKeyLen)
		expected, expectedOk := ref.Get(key)
		for _, fuzz := range fuzzTries {
			actual, actualOk := fuzz.trie.Get(key)
			assert.Equal(t, expectedOk, actualOk, "%s: %s", fuzz.def.name, keyName(key))
			assert.Equal(t, expected, actual, "%s: %s", fuzz.def.name, keyName(key))
		}
	})
}

func FuzzPut(f *testing.F) {
	fuzzTries := createTestTries(fuzzTrieConfigs)
	ref := createReferenceTrie(fuzzTrieConfigs[0])
	f.Fuzz(func(t *testing.T, fuzzKey uint32, fuzzKeyLen, value byte) {
		key := keyForFuzzInputs(fuzzKey, fuzzKeyLen)
		expected, expectedOk := ref.Put(key, value)
		for _, fuzz := range fuzzTries {
			actual, actualOk := fuzz.trie.Put(key, value)
			assert.Equal(t, expectedOk, actualOk, "%s: %s=%d", fuzz.def.name, keyName(key), value)
			assert.Equal(t, expected, actual, "%s: %s=%d", fuzz.def.name, keyName(key), value)
			actual, ok := fuzz.trie.Get(key)
			assert.True(t, ok, "%s: %s=%d", fuzz.def.name, keyName(key), value)
			assert.Equal(t, value, actual, "%s: %s=%d", fuzz.def.name, keyName(key), value)
		}
	})
}

func FuzzDelete(f *testing.F) {
	fuzzTries := createTestTries(fuzzTrieConfigs)
	ref := createReferenceTrie(fuzzTrieConfigs[0])
	f.Fuzz(func(t *testing.T, fuzzKey uint32, fuzzKeyLen byte) {
		key := keyForFuzzInputs(fuzzKey, fuzzKeyLen)
		expected, expectedOk := ref.Delete(key)
		for _, fuzz := range fuzzTries {
			actual, actualOk := fuzz.trie.Delete(key)
			assert.Equal(t, expectedOk, actualOk, "%s: %s", fuzz.def.name, keyName(key))
			assert.Equal(t, expected, actual, "%s: %s", fuzz.def.name, keyName(key))
			actual, ok := fuzz.trie.Get(key)
			assert.False(t, ok, "%s: %s", fuzz.def.name, keyName(key))
			assert.Equal(t, byte(0), actual, "%s: %s", fuzz.def.name, keyName(key))
		}
	})
}

func FuzzRange(f *testing.F) {
	fuzzTries := createTestTries(fuzzRangeTrieConfigs)
	ref := createReferenceTrie(fuzzRangeTrieConfigs[0])
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
		refForward := collect(ref.Range(forward))
		refReverse := collect(ref.Range(reverse))
		for _, fuzz := range fuzzTries {
			assert.Equal(t, refForward, collect(fuzz.trie.Range(forward)), "%s: %s", fuzz.def.name, forward)
			assert.Equal(t, refReverse, collect(fuzz.trie.Range(reverse)), "%s: %s", fuzz.def.name, reverse)
		}
	})
}

func FuzzMixed(f *testing.F) {
	fuzzTries := createTestTries(fuzzTrieConfigs)
	ref := createReferenceTrie(fuzzTrieConfigs[0])
	f.Fuzz(func(t *testing.T, fuzzPutKey, fuzzDeleteKey uint32, fuzzPutKeyLen, fuzzDeleteKeyLen, value byte) {
		key := keyForFuzzInputs(fuzzPutKey, fuzzPutKeyLen)
		expected, expectedOk := ref.Put(key, value)
		for _, fuzz := range fuzzTries {
			actual, actualOk := fuzz.trie.Put(key, value)
			assert.Equal(t, expectedOk, actualOk, "%s: %s=%d", fuzz.def.name, keyName(key), value)
			assert.Equal(t, expected, actual, "%s: %s=%d", fuzz.def.name, keyName(key), value)
			actual, ok := fuzz.trie.Get(key)
			assert.True(t, ok, "%s: %s=%d", fuzz.def.name, keyName(key), value)
			assert.Equal(t, value, actual, "%s: %s=%d", fuzz.def.name, keyName(key), value)
		}

		key = keyForFuzzInputs(fuzzDeleteKey, fuzzDeleteKeyLen)
		expected, expectedOk = ref.Delete(key)
		for _, fuzz := range fuzzTries {
			actual, actualOk := fuzz.trie.Delete(key)
			assert.Equal(t, expectedOk, actualOk, "%s: %s", fuzz.def.name, keyName(key))
			assert.Equal(t, expected, actual, "%s: %s", fuzz.def.name, keyName(key))
			actual, ok := fuzz.trie.Get(key)
			assert.False(t, ok, "%s: %s", fuzz.def.name, keyName(key))
			assert.Equal(t, byte(0), actual, "%s: %s", fuzz.def.name, keyName(key))
		}
	})
}
