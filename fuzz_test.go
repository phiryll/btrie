package btrie_test

import (
	"bytes"
	"encoding/binary"
	"math/bits"
	"math/rand"
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
)

// Go's fuzzing can generate []byte inputs, but not with a length constraint.
// Because of that, the fuzzed keys here will the bytes of a uint32,
// with length 0-3 determined by another fuzzed byte.
// See keyForFuzzInputs.

const (
	fuzzTrieSize      = 1 << 20
	fuzzRangeTrieSize = 1 << 16 // because Range is an expensive operation
	maxFuzzKeyLength  = 4
)

var (
	// There is currently only one config in each of these slices.
	fuzzTrieConfigs      = createFuzzTrieConfigs(fuzzTrieSize)
	fuzzRangeTrieConfigs = createFuzzTrieConfigs(fuzzRangeTrieSize)
)

// Fuzz testing is very parallel, and tries aren't generally thread-safe.
// Ensure that instances are not shared.

func randomBytes(n int, random *rand.Rand) []byte {
	b := make([]byte, n)
	_, _ = random.Read(b)
	return b
}

func randomByte(random *rand.Rand) byte {
	b := []byte{0}
	_, _ = random.Read(b)
	return b[0]
}

// Returns a random key length with distribution:
//
//	50% of maxLength
//	25% of maxLength-1
//	...
//	2 of length 2
//	1 of length 1
//	1 of length 0
func randomKeyLength(maxLength int, random *rand.Rand) int {
	return bits.Len(uint(random.Intn(1 << maxLength)))
}

func randomKey(maxLength int, random *rand.Rand) []byte {
	return randomBytes(randomKeyLength(maxLength, random), random)
}

func keyForFuzzInputs(key uint32, size byte) []byte {
	// size => keySize of result (# out of 256)
	// 0x40-0xFF => 4 (192)
	// 0x10-0x3F => 3 ( 48)
	// 0x04-0x0F => 2 ( 12)
	// 0x01-0x03 => 1 (  3)
	// 0x00      => 0 (  1)
	keySize := 4
	for ; keySize > 0; keySize-- {
		if size&(0x03<<(2*(keySize-1))) != 0 {
			break
		}
	}
	keyBytes := binary.BigEndian.AppendUint32(nil, key)
	return keyBytes[(4 - keySize):] // use low-order bytes
}

func createFuzzTrieConfigs(trieSize int) []*trieConfig {
	var config trieConfig
	random := rand.New(rand.NewSource(time.Now().UnixNano()))
	config.name = "fuzz"
	config.trieSize = trieSize
	config.entries = map[string]byte{}
	for count := 0; count < trieSize; {
		key := string(randomKey(maxFuzzKeyLength, random))
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
	ref := fuzzTries[0].trie
	refForward := collect(ref.Range(forwardAll))
	refReverse := collect(ref.Range(reverseAll))
	for _, fuzz := range fuzzTries[1:] {
		t.Run(fuzz.name, func(t *testing.T) {
			t.Parallel()
			assert.Equal(t, refForward, collect(fuzz.trie.Range(forwardAll)), "forward")
			assert.Equal(t, refReverse, collect(fuzz.trie.Range(reverseAll)), "reverse")
		})
	}
}

func FuzzGet(f *testing.F) {
	fuzzTries := createTestTries(fuzzTrieConfigs)
	ref := fuzzTries[0].trie
	f.Fuzz(func(t *testing.T, uintKey uint32, keySize byte) {
		key := keyForFuzzInputs(uintKey, keySize)
		expected, expectedOk := ref.Get(key)
		for _, fuzz := range fuzzTries[1:] {
			actual, actualOk := fuzz.trie.Get(key)
			assert.Equal(t, expectedOk, actualOk, "%s: %s", fuzz.def.name, keyName(key))
			assert.Equal(t, expected, actual, "%s: %s", fuzz.def.name, keyName(key))
		}
	})
}

func FuzzPut(f *testing.F) {
	fuzzTries := createTestTries(fuzzTrieConfigs)
	ref := fuzzTries[0].trie
	f.Fuzz(func(t *testing.T, uintKey uint32, keySize, value byte) {
		key := keyForFuzzInputs(uintKey, keySize)
		expected, expectedOk := ref.Put(key, value)
		for _, fuzz := range fuzzTries[1:] {
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
	ref := fuzzTries[0].trie
	f.Fuzz(func(t *testing.T, uintKey uint32, keySize byte) {
		key := keyForFuzzInputs(uintKey, keySize)
		expected, expectedOk := ref.Delete(key)
		for _, fuzz := range fuzzTries[1:] {
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
	ref := fuzzTries[0].trie
	f.Fuzz(func(t *testing.T, beginKey, endKey uint32, beginKeySize, endKeySize byte) {
		begin := keyForFuzzInputs(beginKey, beginKeySize)
		end := keyForFuzzInputs(endKey, endKeySize)
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
		for _, fuzz := range fuzzTries[1:] {
			assert.Equal(t, refForward, collect(fuzz.trie.Range(forward)), "%s: %s", fuzz.def.name, forward)
			assert.Equal(t, refReverse, collect(fuzz.trie.Range(reverse)), "%s: %s", fuzz.def.name, reverse)
		}
	})
}

func FuzzMixed(f *testing.F) {
	fuzzTries := createTestTries(fuzzTrieConfigs)
	ref := fuzzTries[0].trie
	f.Fuzz(func(t *testing.T, putKey, deleteKey uint32, putKeySize, deleteKeySize, value byte) {
		key := keyForFuzzInputs(putKey, putKeySize)
		expected, expectedOk := ref.Put(key, value)
		for _, fuzz := range fuzzTries[1:] {
			actual, actualOk := fuzz.trie.Put(key, value)
			assert.Equal(t, expectedOk, actualOk, "%s: %s=%d", fuzz.def.name, keyName(key), value)
			assert.Equal(t, expected, actual, "%s: %s=%d", fuzz.def.name, keyName(key), value)
			actual, ok := fuzz.trie.Get(key)
			assert.True(t, ok, "%s: %s=%d", fuzz.def.name, keyName(key), value)
			assert.Equal(t, value, actual, "%s: %s=%d", fuzz.def.name, keyName(key), value)
		}

		key = keyForFuzzInputs(deleteKey, deleteKeySize)
		expected, expectedOk = ref.Delete(key)
		for _, fuzz := range fuzzTries[1:] {
			actual, actualOk := fuzz.trie.Delete(key)
			assert.Equal(t, expectedOk, actualOk, "%s: %s", fuzz.def.name, keyName(key))
			assert.Equal(t, expected, actual, "%s: %s", fuzz.def.name, keyName(key))
			actual, ok := fuzz.trie.Get(key)
			assert.False(t, ok, "%s: %s", fuzz.def.name, keyName(key))
			assert.Equal(t, byte(0), actual, "%s: %s", fuzz.def.name, keyName(key))
		}
	})
}
