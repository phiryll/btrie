package btrie_test

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"iter"
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

var (
	// How many entries the benchmarked tries will have, with keys of random length up to maxKeySize.
	benchTrieSizes = []int{1 << 12, 1 << 16, 1 << 20}

	// Test keys of these lengths against the bencharked tries.
	benchKeySizes   = []int{3, 4, 5}
	maxBenchKeySize = benchKeySizes[len(benchKeySizes)-1]

	benchTrieConfigs = createBenchTrieConfigs()
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
			var numNodes int
			for range traverser(0, adj) {
				numNodes++
			}
			b.Run(fmt.Sprintf("size=%d", numNodes), func(b *testing.B) {
				b.ResetTimer()
				for range b.N {
					for node := range traverser(0, adj) {
						_ = node
					}
				}
			})
		}
	})
}

func BenchmarkTraverserPaths(b *testing.B) {
	benchTraverserPaths(b, "kind=pre-order", btrie.TestingPreOrderPaths)
	benchTraverserPaths(b, "kind=post-order", btrie.TestingPostOrderPaths)
}

func benchTraverserPaths(b *testing.B, name string, pathTraverser btrie.TestingPathTraverser) {
	b.Run(name, func(b *testing.B) {
		for _, pathAdj := range []btrie.TestingPathAdjFunction{
			emptyPathAdjInt,
			pathAdjInt(0),
			pathAdjInt(1 << 4),
			pathAdjInt(1 << 8),
			pathAdjInt(1 << 12),
			pathAdjInt(1 << 16),
			pathAdjInt(1 << 20),
		} {
			var numPaths int
			for range pathTraverser(0, pathAdj) {
				numPaths++
			}
			b.Run(fmt.Sprintf("size=%d", numPaths), func(b *testing.B) {
				b.ResetTimer()
				for range b.N {
					for path := range pathTraverser(0, pathAdj) {
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
	config := benchTrieConfigs[len(benchTrieConfigs)-1]
	forward := config.forward
	reverse := config.reverse

	random := rand.New(rand.NewSource(239057752))
	keys := make(keySet, 1024)
	for i := range keys {
		// make keys shorter on average
		keys[i] = randomKey(maxBenchKeySize-1, random)
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

func createSparseKeys(random *rand.Rand) keySet {
	var keys keySet
	for key := range 1 << 8 {
		keyByte := byte(key)
		keys = append(keys, []byte{keyByte, keyByte, keyByte, keyByte})
	}
	shuffle(keys, random)
	return keys
}

func createDenseKeys(random *rand.Rand) (keySet, keySet, keySet) {
	oneKeys := make(keySet, 1<<8)
	twoKeys := make(keySet, 1<<16)
	threeKeys := make(keySet, 1<<24)
	for key := range 1 << 8 {
		oneKeys[key] = []byte{byte(key)}
	}
	for key := range 1 << 16 {
		keyBytes := binary.LittleEndian.AppendUint16(nil, uint16(key))
		twoKeys[key] = []byte{keyBytes[0], keyBytes[1]}
	}
	for key := range 1 << 24 {
		keyBytes := binary.LittleEndian.AppendUint32(nil, uint32(key))
		threeKeys[key] = []byte{keyBytes[0], keyBytes[1], keyBytes[2]}
	}
	shuffle(oneKeys, random)
	shuffle(twoKeys, random)
	shuffle(threeKeys, random)
	return oneKeys, twoKeys, threeKeys
}

func createEntries(size int, random *rand.Rand) (map[string]byte, []keySet) {
	entries := map[string]byte{}
	present := make([]keySet, maxBenchKeySize+1)
	for count := 0; count < size; {
		key := randomKey(maxBenchKeySize, random)
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
	absent := make([]keySet, maxBenchKeySize+1)

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
	for count < maxGenSize*(maxBenchKeySize-2) {
		key := randomKey(maxBenchKeySize, random)
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

func nonEmptyIterators(trie TestBTrie, bounds []Bounds, numIters int) []iter.Seq2[[]byte, byte] {
	iters := []iter.Seq2[[]byte, byte]{}
	for _, bound := range bounds {
		itr := trie.Range(bound)
		itr(func([]byte, byte) bool {
			// This yield function was called, so there's at least one element.
			iters = append(iters, itr)
			return false
		})
		if len(iters) == numIters {
			break
		}
	}
	return iters
}

func createBenchTrieConfigs() []*trieConfig {
	result := []*trieConfig{}
	for _, size := range benchTrieSizes {
		random := rand.New(rand.NewSource(int64(size) + 4839028453))
		var config trieConfig
		config.name = fmt.Sprintf("trieSize=%d", size)
		config.trieSize = size
		config.entries, config.present = createEntries(size, random)
		config.absent = createAbsent(config.entries, random)
		// get bounds from a shuffled slice of present and absent keys
		keys := append(slices.Concat(config.present...), slices.Concat(config.absent...)...)
		shuffle(keys, random)
		config.forward, config.reverse = createBounds(keys)
		result = append(result, &config)
	}
	return result
}

func TestBenchTrieConfigs(t *testing.T) {
	t.Parallel()
	for _, config := range benchTrieConfigs {
		assert.Len(t, config.entries, config.trieSize)
		assert.Equal(t, 1, len(config.present[0])+len(config.absent[0]))
		assert.Equal(t, 1<<8, len(config.present[1])+len(config.absent[1]))
		assert.Equal(t, 1<<16, len(config.present[2])+len(config.absent[2]))
		assert.Len(t, config.forward, 1<<16)
		assert.Len(t, config.reverse, 1<<16)

		present := maps.Clone(config.entries)
		for i := range maxBenchKeySize + 1 {
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

func TestBenchTrieConfigRepeatability(t *testing.T) {
	t.Parallel()
	for i, config := range createBenchTrieConfigs() {
		assert.True(t, reflect.DeepEqual(benchTrieConfigs[i], config))
	}
}

// This benchmark is for memory allocations, not time.
func BenchmarkSparseTries(b *testing.B) {
	random := rand.New(rand.NewSource(12337405))
	keys := createSparseKeys(random)
	for _, def := range implDefs {
		b.Run("impl="+def.name, func(b *testing.B) {
			b.ResetTimer()
			for range b.N {
				trie := def.factory()
				for _, key := range keys {
					trie.Put(key, 0)
				}
			}
		})
	}
}

// This benchmark is for memory allocations, not time.
func BenchmarkDenseTries(b *testing.B) {
	random := rand.New(rand.NewSource(9321075532))
	oneKeys, twoKeys, threeKeys := createDenseKeys(random)
	for _, def := range implDefs {
		for _, tt := range []struct {
			name string
			keys keySet
		}{
			{"/keyLen=1", oneKeys},
			{"/keyLen=2", twoKeys},
			{"/keyLen=3", threeKeys},
		} {
			b.Run("impl="+def.name+tt.name, func(b *testing.B) {
				b.ResetTimer()
				for range b.N {
					trie := def.factory()
					for _, key := range tt.keys {
						trie.Put(key, 0)
					}
				}
			})
		}
	}
}

// For mutable implementations, Clone() should be efficient, but not absurdly efficient.
// If it is, that's a sign it's sharing storage instead of creating new storage.
func BenchmarkClone(b *testing.B) {
	for _, bench := range createTestTries(benchTrieConfigs) {
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
	for _, bench := range createTestTries(benchTrieConfigs) {
		original := bench.trie
		b.Run(bench.name, func(b *testing.B) {
			for _, keySize := range benchKeySizes {
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
	for _, bench := range createTestTries(benchTrieConfigs) {
		original := bench.trie
		b.Run(bench.name, func(b *testing.B) {
			for _, keySize := range benchKeySizes {
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
	for _, bench := range createTestTries(benchTrieConfigs) {
		original := bench.trie
		b.Run(bench.name, func(b *testing.B) {
			for _, keySize := range benchKeySizes {
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
	for _, bench := range createTestTries(benchTrieConfigs) {
		original := bench.trie
		trie := original.Clone()
		forward := bench.config.forward
		reverse := bench.config.reverse
		numIters := maxGenSize
		if _, ok := trie.(*reference); ok {
			numIters = 64
		}
		forwardIters := nonEmptyIterators(trie, forward, numIters)
		reverseIters := nonEmptyIterators(trie, reverse, numIters)
		b.Run(bench.name, func(b *testing.B) {
			b.Run("dir=forward/op=range", func(b *testing.B) {
				if _, ok := trie.(*reference); ok {
					b.Skip("reference.Range() creation is grossly inefficient")
				}
				b.ResetTimer()
				for i := range b.N {
					trie.Range(forward[i%len(forward)])
				}
			})
			b.Run("dir=forward/op=iter", func(b *testing.B) {
				count := 0
				b.ResetTimer()
				for i := 0; true; i++ {
					for range forwardIters[i%len(forwardIters)] {
						count++
						if count == b.N {
							return
						}
					}
				}
			})
			b.Run("dir=reverse/op=range", func(b *testing.B) {
				if _, ok := trie.(*reference); ok {
					b.Skip("reference.Range() creation is grossly inefficient")
				}
				b.ResetTimer()
				for i := range b.N {
					trie.Range(reverse[i%len(reverse)])
				}
			})
			b.Run("dir=reverse/op=iter", func(b *testing.B) {
				count := 0
				b.ResetTimer()
				for i := 0; true; i++ {
					for range reverseIters[i%len(reverseIters)] {
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
