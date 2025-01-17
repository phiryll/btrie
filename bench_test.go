package btrie_test

import (
	"bufio"
	"bytes"
	"encoding/binary"
	"fmt"
	"iter"
	"maps"
	"math/rand"
	"os"
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
	// Don't benchmark keys shorter than this.
	benchMinKeyLen = 3

	// The maximum length of keys for randomly generated tries.
	benchMaxRandomKeyLen = 5

	// The minimum number of keys to benchmark when tries must be cloned after exhausting the keys.
	benchMinNumMutableKeys = 1 << 10

	// The maximum size of created absent and bounds slices, 64K.
	genMaxSize = 1 << 16

	// Only the lowercase characters a-z and newlines appear in this file.
	filenameWords = "testdata/words_alpha.txt"
)

var (
	// How many entries randomly generated benchmarked tries will have,
	// with keys of random length up to benchMaxKeyLen.
	benchRandomTrieSizes = []int{1 << 12, 1 << 16, 1 << 20}

	benchTrieConfigs = createBenchTrieConfigs()
)

// Does not work for single-use iterators.
func repeat2[K, V any](itr iter.Seq2[K, V]) iter.Seq2[K, V] {
	return func(yield func(K, V) bool) {
		for {
			for k, v := range itr {
				if !yield(k, v) {
					return
				}
			}
		}
	}
}

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

func BenchmarkChildBounds(b *testing.B) {
	// Max key length for this benchmark.
	const maxKeyLen = 4

	// Reuse bounds from the biggest trieConfig, but create new (partial) keys.
	config := benchTrieConfigs[len(benchTrieConfigs)-1]
	forward := config.forward
	reverse := config.reverse
	random := rand.New(rand.NewSource(239057752))
	keys := make(keySet, 1024)
	for i := range keys {
		keys[i] = randomKey(maxKeyLen, random)
	}
	b.Run("dir=forward", func(b *testing.B) {
		count := 0
		b.ResetTimer()
		for _, bounds := range repeat2(slices.All(forward)) {
			for _, key := range keys {
				btrie.TestingChildBounds(&bounds, key)
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
		for _, bounds := range repeat2(slices.All(reverse)) {
			for _, key := range keys {
				btrie.TestingChildBounds(&bounds, key)
				count++
				if count == b.N {
					return
				}
			}
		}
	})
}

func randomEntries(numEntries int) map[string]byte {
	random := rand.New(rand.NewSource(int64(numEntries) + 83741074321))
	entries := map[string]byte{}
	for count := 0; count < numEntries; {
		key := randomKey(benchMaxRandomKeyLen, random)
		if _, ok := entries[string(key)]; !ok {
			entries[string(key)] = randomByte(random)
			count++
		}
	}
	return entries
}

func entriesFromFile(filename string) map[string]byte {
	file, err := os.Open(filename)
	if err != nil {
		panic(fmt.Sprintf("text file %s could not be opened: %s", filename, err))
	}
	defer file.Close()
	entries := map[string]byte{}
	scanner := bufio.NewScanner(file)
	for scanner.Scan() {
		entries[scanner.Text()] = 0
	}
	if err := scanner.Err(); err != nil {
		panic(fmt.Sprintf("error reading text file %s: %s", filename, err))
	}
	return entries
}

func createPresent(entries map[string]byte, random *rand.Rand) []keySet {
	present := []keySet{}
	for key := range entries {
		keyLen := len(key)
		for i := len(present); i <= keyLen; i++ {
			present = append(present, keySet{})
		}
		present[keyLen] = append(present[keyLen], []byte(key))
	}
	for _, keys := range present {
		slices.SortFunc(keys, bytes.Compare)
		shuffle(keys, random)
	}
	return present
}

func createAbsent(entries map[string]byte, maxKeyLen int, random *rand.Rand) []keySet {
	absent := make([]keySet, maxKeyLen+1)

	// set absent for key lengths 0-2, everything not present
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

	// set absent for key lengths 3+, randomly generated
	// keep adding until each absent[keyLen] has maxGenSize keys
	for keyLen := range absent {
		if keyLen < benchMinKeyLen {
			continue
		}
		for len(absent[keyLen]) < genMaxSize {
			key := randomFixedLengthKey(keyLen, random)
			if _, ok := entries[string(key)]; !ok {
				absent[keyLen] = append(absent[keyLen], key)
			}
		}
	}

	return absent
}

func createBounds(keys keySet) ([]Bounds, []Bounds) {
	var forward, reverse []Bounds
	for i := range genMaxSize {
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
		forward = append(forward, *From(begin).To(end))
		reverse = append(reverse, *From(end).DownTo(begin))
	}
	return forward, reverse
}

func createFixedBounds(step int, random *rand.Rand) ([]Bounds, []Bounds) {
	var forward, reverse []Bounds
	for low := step / 2; low < 1<<24-step; low += step {
		high := low + step
		keyBytes := binary.BigEndian.AppendUint32(nil, uint32(low))
		lowKey := []byte{keyBytes[1], keyBytes[2], keyBytes[3]}
		keyBytes = binary.BigEndian.AppendUint32(nil, uint32(high))
		highKey := []byte{keyBytes[1], keyBytes[2], keyBytes[3]}
		forward = append(forward, *From(lowKey).To(highKey))
		reverse = append(reverse, *From(highKey).DownTo(lowKey))
	}
	shuffle(forward, random)
	shuffle(reverse, random)
	return forward, reverse
}

func createBenchTrieConfig(corpusName string, entries map[string]byte) *trieConfig {
	random := rand.New(rand.NewSource(int64(len(entries)) + 4839028453))
	present := createPresent(entries, random)
	absent := createAbsent(entries, len(present)-1, random)
	keys := append(slices.Concat(present...), slices.Concat(absent...)...)
	shuffle(keys, random)
	forward, reverse := createBounds(keys)
	return &trieConfig{
		name:     fmt.Sprintf("corpus=%s/trieSize=%d", corpusName, len(entries)),
		trieSize: len(entries),
		entries:  entries,
		present:  present,
		absent:   absent,
		forward:  forward,
		reverse:  reverse,
	}
}

func createBenchRandomTrieConfigs() []*trieConfig {
	result := []*trieConfig{}
	for _, trieSize := range benchRandomTrieSizes {
		result = append(result, createBenchTrieConfig("random", randomEntries(trieSize)))
	}
	return result
}

// Config with lower-case english words, values are all 0.
func createBenchWordTrieConfigs() []*trieConfig {
	return []*trieConfig{createBenchTrieConfig("words", entriesFromFile(filenameWords))}
}

func createBenchTrieConfigs() []*trieConfig {
	return append(createBenchRandomTrieConfigs(), createBenchWordTrieConfigs()...)
}

func TestBenchTrieConfigs(t *testing.T) {
	t.Parallel()
	for _, config := range benchTrieConfigs {
		t.Run(config.name, func(t *testing.T) {
			assert.Len(t, config.entries, config.trieSize)
			assert.Equal(t, len(config.present), len(config.absent))
			assert.Equal(t, 1, len(config.present[0])+len(config.absent[0]))
			assert.Equal(t, 1<<8, len(config.present[1])+len(config.absent[1]))
			assert.Equal(t, 1<<16, len(config.present[2])+len(config.absent[2]))
			assert.Len(t, config.forward, 1<<16)
			assert.Len(t, config.reverse, 1<<16)

			present := maps.Clone(config.entries)
			for i := range len(config.present) {
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
		})
	}
}

func TestBenchTrieConfigRepeatability(t *testing.T) {
	t.Parallel()
	for i, config := range createBenchTrieConfigs() {
		t.Run(config.name, func(t *testing.T) {
			assert.True(t, reflect.DeepEqual(benchTrieConfigs[i], config))
		})
	}
}

// This benchmark is for memory allocations, not time.
func BenchmarkSparseTries(b *testing.B) {
	random := rand.New(rand.NewSource(12337405))
	var keys keySet
	for key := range 1 << 8 {
		keyByte := byte(key)
		keys = append(keys, []byte{keyByte, keyByte, keyByte, keyByte})
	}
	shuffle(keys, random)
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
	keySets := []keySet{oneKeys, twoKeys, threeKeys}
	for _, def := range implDefs {
		for _, tt := range []struct {
			name string
			keys keySet
		}{
			{"/keyLen=1", keySets[0]},
			{"/keyLen=2", keySets[1]},
			{"/keyLen=3", keySets[2]},
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
			for keyLen := range len(bench.config.present) {
				if keyLen < benchMinKeyLen {
					continue
				}
				b.Run(fmt.Sprintf("keyLen=%d/existing=true", keyLen), func(b *testing.B) {
					present := bench.config.present[keyLen]
					if len(present) == 0 {
						b.Skipf("no present keys of length %d", keyLen)
					}
					trie := original.Clone()
					b.ResetTimer()
					for i := range b.N {
						trie.Put(present[i%len(present)], 42)
					}
				})
				b.Run(fmt.Sprintf("keyLen=%d/existing=false", keyLen), func(b *testing.B) {
					absent := bench.config.absent[keyLen]
					if len(absent) == 0 {
						b.Skipf("no absent keys of length %d", keyLen)
					}
					if len(absent) < benchMinNumMutableKeys {
						b.Skipf("insufficient absent keys of length %d: %d", keyLen, len(absent))
					}
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
			for keyLen := range len(bench.config.present) {
				if keyLen < benchMinKeyLen {
					continue
				}
				b.Run(fmt.Sprintf("keyLen=%d/existing=true", keyLen), func(b *testing.B) {
					present := bench.config.present[keyLen]
					if len(present) == 0 {
						b.Skipf("no present keys of length %d", keyLen)
					}
					trie := original.Clone()
					b.ResetTimer()
					for i := range b.N {
						trie.Get(present[i%len(present)])
					}
				})
				b.Run(fmt.Sprintf("keyLen=%d/existing=false", keyLen), func(b *testing.B) {
					absent := bench.config.absent[keyLen]
					if len(absent) == 0 {
						b.Skipf("no absent keys of length %d", keyLen)
					}
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
			for keyLen := range len(bench.config.present) {
				if keyLen < benchMinKeyLen {
					continue
				}
				b.Run(fmt.Sprintf("keyLen=%d/existing=true", keyLen), func(b *testing.B) {
					present := bench.config.present[keyLen]
					if len(present) == 0 {
						b.Skipf("no present keys of length %d", keyLen)
					}
					if len(present) < benchMinNumMutableKeys {
						b.Skipf("insufficient present keys of length %d: %d", keyLen, len(present))
					}
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
				b.Run(fmt.Sprintf("keyLen=%d/existing=false", keyLen), func(b *testing.B) {
					absent := bench.config.absent[keyLen]
					if len(absent) == 0 {
						b.Skipf("no absent keys of length %d", keyLen)
					}
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
func benchRange(b *testing.B, getBounds func(*testTrie) ([]Bounds, []Bounds)) {
	for _, bench := range createTestTries(benchTrieConfigs) {
		if _, ok := bench.trie.(*reference); ok {
			// reference.Range() creation is grossly inefficient
			continue
		}
		forward, reverse := getBounds(bench)
		original := bench.trie
		trie := original.Clone()
		b.Run(bench.name, func(b *testing.B) {
			b.Run("dir=forward/op=range", func(b *testing.B) {
				b.ResetTimer()
				for i := range b.N {
					trie.Range(&forward[i%len(forward)])
				}
			})
			b.Run("dir=forward/op=full", func(b *testing.B) {
				b.ResetTimer()
				for i := range b.N {
					for k, v := range trie.Range(&forward[i%len(forward)]) {
						_, _ = k, v
					}
				}
			})
			b.Run("dir=reverse/op=range", func(b *testing.B) {
				b.ResetTimer()
				for i := range b.N {
					trie.Range(&reverse[i%len(reverse)])
				}
			})
			b.Run("dir=reverse/op=full", func(b *testing.B) {
				b.ResetTimer()
				for i := range b.N {
					for k, v := range trie.Range(&reverse[i%len(reverse)]) {
						_, _ = k, v
					}
				}
			})
		})
	}
}

func BenchmarkShortRange(b *testing.B) {
	random := rand.New(rand.NewSource(74320567))
	forward, reverse := createFixedBounds(0x00_00_00_83, random)
	benchRange(b, func(_ *testTrie) ([]Bounds, []Bounds) {
		return forward, reverse
	})
}

func BenchmarkLongRange(b *testing.B) {
	random := rand.New(rand.NewSource(48239752))
	forward, reverse := createFixedBounds(0x00_02_13_13, random)
	benchRange(b, func(_ *testTrie) ([]Bounds, []Bounds) {
		return forward, reverse
	})
}

func BenchmarkRandomRange(b *testing.B) {
	benchRange(b, func(tt *testTrie) ([]Bounds, []Bounds) {
		return tt.config.forward, tt.config.reverse
	})
}
