package btrie_test

import (
	"bufio"
	"bytes"
	"encoding/binary"
	"fmt"
	"maps"
	rand "math/rand/v2"
	"os"
	"reflect"
	"slices"
	"strings"
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
// That said, extensive benchmarking (not checked in) has shown that neither the frequency nor duration of timer pauses
// to clone a trie have a significant effect on reported timings, at most about 8 ns/op in my current setup.
// However, each clone can take a significant amount of time even though it isn't measured,
// so these benchmarks take steps to reduce that impact.

const (
	benchMeanRandomKeyLen = 8

	// The minimum number of keys to benchmark when tries must be cloned after exhausting the keys.
	// i.e., don't benchmark if we will need to clone the trie every N operations and N < benchMinNumMutableKeys.
	benchMinNumMutableKeys = 1 << 10

	// The maximum size of created absent and bounds slices, 64K.
	genMaxSize = 1 << 16

	// Only the lowercase characters a-z and newlines appear in this file.
	filenameWords = "testdata/words_alpha.txt"
)

var (
	// How many entries randomly generated benchmarked tries will have.
	benchRandomTrieSizes = []int{1 << 16, 1 << 18, 1 << 20}

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

//nolint:funlen
func BenchmarkChildBounds(b *testing.B) {
	for _, tt := range []struct {
		bounds *Bounds
		keys   keySet
	}{
		// no common prefix
		{
			From(nil).To(empty),
			keySet{empty, next(empty), after},
		},
		{
			From(nil).To(next(empty)),
			keySet{empty, next(empty), next(next(empty)), after},
		},
		{
			From(nil).To(high),
			keySet{empty, next(empty), before, high[:1], high[:2], high[:3], prev(high), high, next(high), after},
		},
		{
			From(nil).To(nil),
			keySet{empty, next(empty), within},
		},
		{
			From(empty).To(next(empty)),
			keySet{empty, next(empty), next(next(empty)), after},
		},
		{
			From(empty).To(high),
			keySet{empty, next(empty), before, high[:1], high[:2], high[:3], prev(high), high, next(high), after},
		},
		{
			From(empty).To(nil),
			keySet{empty, next(empty), within},
		},
		{
			From(next(empty)).To(high),
			keySet{
				empty, next(empty), next(next(empty)), before, high[:1], high[:2], high[:3],
				prev(high), high, next(high), after,
			},
		},
		{
			From(next(empty)).To(nil),
			keySet{empty, next(empty), next(next(empty)), within},
		},
		{
			From(low).To(high),
			keySet{
				empty, next(empty), before, low[:1], low[:2], low[:3], prev(low), low, next(low),
				within, high[:1], high[:2], high[:3], prev(high), high, next(high), after,
			},
		},
		{
			From(low).To(nil),
			keySet{empty, next(empty), before, low[:1], low[:2], low[:3], prev(low), low, next(low), after},
		},

		// 2 byte common prefix
		{
			From(low).To(low2),
			keySet{
				empty, next(empty), before, low[:1], low[:2], low[:3], prev(low), low, next(low),
				midLows, low2[:3], prev(low2), low2, next(low2), after,
			},
		},

		// From is a prefix of To
		{
			From(low[:2]).To(low),
			keySet{
				empty, next(empty), before, low[:1], prev(low[:2]), low[:2], next(low[:2]), low[:3],
				prev(low), low, next(low), after,
			},
		},
	} {
		b.Run(fmt.Sprintf("bounds=%s", tt.bounds), func(b *testing.B) {
			forward := tt.bounds
			reverse := From(tt.bounds.End).DownTo(tt.bounds.Begin)
			for _, key := range tt.keys {
				b.Run("key="+btrie.KeyName(key), func(b *testing.B) {
					b.Run("dir=forward", func(b *testing.B) {
						b.ResetTimer()
						for range b.N {
							btrie.TestingChildBounds(forward, key)
						}
					})
					b.Run("dir=reverse", func(b *testing.B) {
						b.ResetTimer()
						for range b.N {
							btrie.TestingChildBounds(reverse, key)
						}
					})
				})
			}
		})
	}
}

func randomEntries(numEntries int) map[string]byte {
	random := rand.New(rand.NewPCG(uint64(numEntries), 83741074321))
	entries := map[string]byte{}
	for count := 0; count < numEntries; {
		key := string(randomKey(benchMeanRandomKeyLen, random))
		if _, ok := entries[key]; !ok {
			entries[key] = randomByte(random)
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
	//nolint:errcheck
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
		if keyLen < 3 {
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

//nolint:nonamedreturns
func createBounds(keys keySet) (forward, reverse []Bounds) {
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

//nolint:nonamedreturns
func createFixedBounds(step int, random *rand.Rand) (forward, reverse []Bounds) {
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
	random := rand.New(rand.NewPCG(uint64(len(entries)), 4839028453))
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
			t.Parallel()
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
			t.Parallel()
			assert.True(t, reflect.DeepEqual(benchTrieConfigs[i], config))
		})
	}
}

// This helps to understand how factory() can impact other benchmarks which use it.
func BenchmarkFactory(b *testing.B) {
	for _, def := range implDefs {
		b.Run("impl="+def.name, func(b *testing.B) {
			b.ResetTimer()
			for range b.N {
				_ = def.factory()
			}
		})
	}
}

func BenchmarkCreate(b *testing.B) {
	for _, bench := range createTestTries(benchTrieConfigs) {
		b.Run(bench.name, func(b *testing.B) {
			b.ResetTimer()
			for range b.N {
				trie := bench.def.factory()
				for k, v := range bench.config.entries {
					trie.Put([]byte(k), v)
				}
			}
		})
	}
}

// For mutable implementations, Clone() should be efficient, but not absurdly efficient.
// If it is, that's a sign it's sharing storage instead of creating new storage.
// This also helps to understand how Clone() can impact other benchmarks which use it.
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

// This benchmark is for memory allocations, not time.
// Creates one trie and puts many keys per benchmark iteration.
func BenchmarkSparseTries(b *testing.B) {
	random := rand.New(rand.NewPCG(12337405, 432843980))
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
// Creates one trie and puts many keys per benchmark iteration.
func BenchmarkDenseTries(b *testing.B) {
	random := rand.New(rand.NewPCG(9321075532, 1293487543289))
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

//nolint:gocognit
func BenchmarkPut(b *testing.B) {
	for _, bench := range createTestTries(benchTrieConfigs) {
		original := bench.trie
		b.Run(bench.name, func(b *testing.B) {
			for keyLen := 8; keyLen < len(bench.config.present); keyLen += 4 {
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

//nolint:gocognit
func BenchmarkGet(b *testing.B) {
	for _, bench := range createTestTries(benchTrieConfigs) {
		original := bench.trie
		b.Run(bench.name, func(b *testing.B) {
			for keyLen := 8; keyLen < len(bench.config.present); keyLen += 4 {
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
			for keyLen := 8; keyLen < len(bench.config.present); keyLen += 4 {
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
		forward, reverse := getBounds(bench)
		original := bench.trie
		trie := original.Clone()
		b.Run(bench.name, func(b *testing.B) {
			// This is a hack, but good enough for now.
			// The words corpus is not uniformly random, unlike the forward/reverse ranges being used.
			if strings.Contains(bench.name, "corpus=words") {
				b.Skip()
			}
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
	random := rand.New(rand.NewPCG(74320567, 6234982127))
	forward, reverse := createFixedBounds(0x00_00_00_83, random)
	benchRange(b, func(_ *testTrie) ([]Bounds, []Bounds) {
		return forward, reverse
	})
}

func BenchmarkLongRange(b *testing.B) {
	random := rand.New(rand.NewPCG(48239752, 80321318701))
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
