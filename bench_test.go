package btrie_test

import (
	"bytes"
	"fmt"
	"maps"
	"math/rand"
	"reflect"
	"slices"
	"testing"

	"github.com/phiryll/btrie"
	"github.com/stretchr/testify/assert"
)

// No benchmark can have a truly random element, random seeds must be constants!

// Because these are stateful data structures, accurately benchmarking mutating methods is difficult.
// Using the B.Start/StopTimer methods inside the B.N loops can result in erratic stats,
// and sometimes it just hangs forever.
// So it's not possible to create a new non-trivial trie fixture inside the B.N loop
// without the trie's creation also being measured, or at least the B.Start/Stop methods interfering.
// To minimize this, all the tries are constructed only once, and they all implement a non-public-API Clone() method.

const (
	// The maximum size of created absent and bounds slices, 64K.
	maxGenSize = 1 << 16
)

type (
	implDef struct {
		name    string
		factory func() TestBTrie
	}

	// For benchmarking, there is one trieConfig with a given trieSize used by all benchmarks,
	// shared by all trie implementations.
	trieConfig struct {
		// The number of key/value pairs in tries generated by this config.
		// Also the size of entries.
		trieSize int

		// The key/value entries in tries generated by this config.
		entries map[string]byte

		// present/absent[i] = a set of keys of length i that are present/absent.
		// For denser tries, there may be no absent keys of length 0 or 1.
		present, absent []keySet

		// forward/reverse Bounds instances to benchmark Range.
		forward, reverse []Bounds
	}

	// A trie created by an implDef.factory() with entries from config.
	benchConfig struct {
		name   string // impl=X/trieSize=N
		config *trieConfig
		trie   TestBTrie
	}
)

var (
	implDefs = []implDef{
		{"reference", newReference},
		{"pointer-trie", newPointerTrie},
	}

	// How many entries the benchmarked tries will have, with keys of random length up to maxKeySize.
	trieSizes = []int{1 << 12, 1 << 16, 1 << 20}

	// Test keys of these lengths against the bencharked tries.
	keySizes   = []int{3, 4, 5}
	maxKeySize = keySizes[len(keySizes)-1]

	trieConfigs = createTrieConfigs()
)

func BenchmarkTraverser(b *testing.B) {
	benchTraverser(b, "kind=pre-order", btrie.TestingPreOrder)
	benchTraverser(b, "kind=post-order", btrie.TestingPostOrder)
}

func benchTraverser(b *testing.B, name string, traverser btrie.TestingTraverser) {
	b.Run(name, func(b *testing.B) {
		for _, adj := range []btrie.TestingAdjFunction{
			emptyAdjInt,
			adjInt(0),
			adjInt(1 << 4),
			adjInt(1 << 8),
			adjInt(1 << 12),
			adjInt(1 << 16),
			adjInt(1 << 20),
		} {
			var numPaths int
			for range traverser(0, adj) {
				numPaths++
			}
			b.Run(fmt.Sprintf("size=%d", numPaths), func(b *testing.B) {
				b.ResetTimer()
				for range b.N {
					for path := range traverser(0, adj) {
						_ = path
					}
				}
			})
		}
	})
}

//nolint:gocognit
func BenchmarkChildBounds(b *testing.B) {
	// Reuse bounds from the biggest trieConfig, but create new (partial) keys.
	config := trieConfigs[len(trieConfigs)-1]
	forward := config.forward
	reverse := config.reverse

	random := rand.New(rand.NewSource(239057752))
	keys := make(keySet, 1024)
	for i := range keys {
		// make keys shorter on average
		keys[i] = randomKey(maxKeySize-1, random)
	}

	b.Run("dir=forward", func(b *testing.B) {
		count := 0
		b.ResetTimer()
		for {
			for _, bounds := range forward {
				for _, key := range keys {
					btrie.TestingChildBounds(bounds, key)
				}
				count++
				if count == b.N {
					return
				}
			}
		}
	})
	b.Run("dir=reverse", func(b *testing.B) {
		count := 0
		b.ResetTimer()
		for {
			for _, bounds := range reverse {
				for _, key := range keys {
					btrie.TestingChildBounds(bounds, key)
				}
				count++
				if count == b.N {
					return
				}
			}
		}
	})
}

func createEntries(size int, random *rand.Rand) (map[string]byte, []keySet) {
	entries := map[string]byte{}
	present := make([]keySet, maxKeySize+1)
	for count := 0; count < size; {
		key := randomKey(maxKeySize, random)
		keySize := len(key)
		if _, ok := entries[string(key)]; !ok {
			present[keySize] = append(present[keySize], key)
			entries[string(key)] = randomByte(random)
			count++
		}
	}
	return entries, present
}

func createAbsent(entries map[string]byte, random *rand.Rand) []keySet {
	absent := make([]keySet, maxKeySize+1)

	// set absent for key sizes 0-2, everything not present
	if _, ok := entries[""]; !ok {
		absent[0] = append(absent[0], []byte{})
	}
	for hi := range 256 {
		key := []byte{byte(hi)}
		if _, ok := entries[string(key)]; !ok {
			absent[1] = append(absent[1], key)
		}
		for low := range 256 {
			key := []byte{byte(hi), byte(low)}
			if _, ok := entries[string(key)]; !ok {
				absent[2] = append(absent[2], key)
			}
		}
	}

	// set config.absent for key sizes 3+, randomly generated
	// keep adding until each size has maxGenSize keys
	count := 0
	for count < maxGenSize*(maxKeySize-2) {
		key := randomKey(maxKeySize, random)
		keySize := len(key)
		if keySize < 3 || len(absent[keySize]) == maxGenSize {
			continue
		}
		if _, ok := entries[string(key)]; !ok {
			absent[keySize] = append(absent[keySize], key)
			count++
		}
	}

	return absent
}

func createBounds(keys keySet) ([]Bounds, []Bounds) {
	var forward, reverse []Bounds
	for i := range maxGenSize {
		begin := keys[(2*i)%len(keys)]
		end := keys[(2*i+1)%len(keys)]
		switch cmp := bytes.Compare(begin, end); {
		case cmp == 0:
			end = append(end, 0)
		case cmp > 0:
			begin, end = end, begin
		case cmp < 0:
			// no adjustment needed
		}
		forward = append(forward, From(begin).To(end))
		reverse = append(reverse, From(end).DownTo(begin))
	}
	return forward, reverse
}

func createTrieConfigs() []*trieConfig {
	result := []*trieConfig{}
	for _, size := range trieSizes {
		random := rand.New(rand.NewSource(int64(size) + 4839028453))
		//nolint:exhaustruct
		config := trieConfig{trieSize: size}
		config.entries, config.present = createEntries(size, random)
		config.absent = createAbsent(config.entries, random)
		// get bounds from a shuffled slice of present and absent keys
		keys := append(slices.Concat(config.present...), slices.Concat(config.absent...)...)
		random.Shuffle(len(keys), func(i, j int) {
			keys[i], keys[j] = keys[j], keys[i]
		})
		config.forward, config.reverse = createBounds(keys)
		result = append(result, &config)
	}
	return result
}

func benchConfigs() []benchConfig {
	result := []benchConfig{}
	for _, def := range implDefs {
		for _, config := range trieConfigs {
			trie := def.factory()
			for k, v := range config.entries {
				trie.Put([]byte(k), v)
			}
			result = append(result, benchConfig{
				fmt.Sprintf("impl=%s/trieSize=%d", def.name, config.trieSize),
				config,
				trie,
			})
		}
	}
	return result
}

func TestTrieConfigs(t *testing.T) {
	t.Parallel()
	for _, config := range trieConfigs {
		assert.Len(t, config.entries, config.trieSize)
		assert.Equal(t, 1, len(config.present[0])+len(config.absent[0]))
		assert.Equal(t, 1<<8, len(config.present[1])+len(config.absent[1]))
		assert.Equal(t, 1<<16, len(config.present[2])+len(config.absent[2]))
		assert.Len(t, config.forward, 1<<16)
		assert.Len(t, config.reverse, 1<<16)

		present := maps.Clone(config.entries)
		for i := range maxKeySize + 1 {
			if i > 2 {
				assert.Len(t, config.absent[i], 1<<16)
			}
			for _, key := range config.absent[i] {
				assert.Len(t, key, i)
				_, ok := present[string(key)]
				assert.False(t, ok)
			}
			for _, key := range config.present[i] {
				assert.Len(t, key, i)
				_, ok := present[string(key)]
				assert.True(t, ok)
				delete(present, string(key))
			}
		}
		assert.Empty(t, present)
	}
}

func TestTrieConfigRepeatability(t *testing.T) {
	t.Parallel()
	for i, config := range createTrieConfigs() {
		assert.True(t, reflect.DeepEqual(trieConfigs[i], config))
	}
}

// For mutable implementations, Clone() should be efficient, but not absurdly efficient.
// If it is, that's a sign it's sharing storage instead of creating new storage.
func BenchmarkClone(b *testing.B) {
	for _, bench := range benchConfigs() {
		original := bench.trie
		b.Run(bench.name, func(b *testing.B) {
			b.ResetTimer()
			for range b.N {
				_ = original.Clone()
			}
		})
	}
}

//nolint:gocognit
func BenchmarkPut(b *testing.B) {
	for _, bench := range benchConfigs() {
		original := bench.trie
		b.Run(bench.name, func(b *testing.B) {
			for _, keySize := range keySizes {
				present := bench.config.present[keySize]
				absent := bench.config.absent[keySize]
				b.Run(fmt.Sprintf("keyLen=%d/existing=true", keySize), func(b *testing.B) {
					trie := original.Clone()
					b.ResetTimer()
					for i := range b.N {
						trie.Put(present[i%len(present)], 42)
					}
				})
				b.Run(fmt.Sprintf("keyLen=%d/existing=false", keySize), func(b *testing.B) {
					trie := original.Clone()
					b.ResetTimer()
					for i := range b.N {
						if i%len(absent) == 0 && i > 0 {
							b.StopTimer()
							trie = original.Clone()
							b.StartTimer()
						}
						trie.Put(absent[i%len(absent)], 42)
					}
				})
			}
		})
	}
}

func BenchmarkGet(b *testing.B) {
	for _, bench := range benchConfigs() {
		original := bench.trie
		b.Run(bench.name, func(b *testing.B) {
			for _, keySize := range keySizes {
				present := bench.config.present[keySize]
				absent := bench.config.absent[keySize]
				b.Run(fmt.Sprintf("keyLen=%d/existing=true", keySize), func(b *testing.B) {
					trie := original.Clone()
					b.ResetTimer()
					for i := range b.N {
						trie.Get(present[i%len(present)])
					}
				})
				b.Run(fmt.Sprintf("keyLen=%d/existing=false", keySize), func(b *testing.B) {
					trie := original.Clone()
					b.ResetTimer()
					for i := range b.N {
						trie.Get(absent[i%len(absent)])
					}
				})
			}
		})
	}
}

//nolint:gocognit
func BenchmarkDelete(b *testing.B) {
	for _, bench := range benchConfigs() {
		original := bench.trie
		b.Run(bench.name, func(b *testing.B) {
			for _, keySize := range keySizes {
				present := bench.config.present[keySize]
				absent := bench.config.absent[keySize]
				b.Run(fmt.Sprintf("keyLen=%d/existing=true", keySize), func(b *testing.B) {
					trie := original.Clone()
					b.ResetTimer()
					for i := range b.N {
						if i%len(present) == 0 && i > 0 {
							b.StopTimer()
							trie = original.Clone()
							b.StartTimer()
						}
						trie.Delete(present[i%len(present)])
					}
				})
				b.Run(fmt.Sprintf("keyLen=%d/existing=false", keySize), func(b *testing.B) {
					trie := original.Clone()
					b.ResetTimer()
					for i := range b.N {
						trie.Delete(absent[i%len(absent)])
					}
				})
			}
		})
	}
}

//nolint:gocognit
func BenchmarkRange(b *testing.B) {
	for _, bench := range benchConfigs() {
		original := bench.trie
		forward := bench.config.forward
		reverse := bench.config.reverse
		b.Run(bench.name, func(b *testing.B) {
			b.Run("dir=forward", func(b *testing.B) {
				trie := original.Clone()
				count := 0
				b.ResetTimer()
				for i := 0; true; i++ {
					for range trie.Range(forward[i%len(forward)]) {
						count++
						if count == b.N {
							return
						}
					}
				}
			})
			b.Run("dir=reverse", func(b *testing.B) {
				trie := original.Clone()
				count := 0
				b.ResetTimer()
				for i := 0; true; i++ {
					for range trie.Range(reverse[i%len(reverse)]) {
						count++
						if count == b.N {
							return
						}
					}
				}
			})
		})
	}
}
