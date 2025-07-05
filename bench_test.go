package kv_test

import (
	"bufio"
	"bytes"
	"encoding/binary"
	"fmt"
	"maps"
	"math"
	rand "math/rand/v2"
	"os"
	"reflect"
	"slices"
	"strings"
	"testing"

	"github.com/phiryll/kv"
	"github.com/stretchr/testify/assert"
)

// No benchmark can have a truly random element, random seeds must be constants!

// Because these are stateful data structures, accurately benchmarking mutating methods (Set, Delete) is cumbersome.
// The two possible approaches are to recreate the data structure every time through the loop,
// or when it reaches a state that is no longer useful to benchmark.
// The latter approach was chosen here, since recreation is expensive.
// The benchmark timer is paused when the stores are recreated.
// The measured benchmark timing overhead for pausing the timer is tiny (~8ns on my machine),
// but the wall clock overhead can be significant.

const (
	benchMeanRandomKeyLen = 8

	// The minimum number of keys to benchmark when stores must be cloned after exhausting the keys.
	// i.e., don't benchmark if we will need to clone the store every N operations and N < benchMinNumMutableKeys.
	benchMinNumMutableKeys = 1 << 10

	// The maximum size of created absent and bounds slices, 64K.
	genMaxSize = 1 << 16

	// Only the lowercase characters a-z and newlines appear in this file.
	filenameWords = "testdata/words_alpha.txt"
)

var (
	// How many entries randomly generated benchmarked stores will have.
	benchRandomSizes = []int{1 << 16, 1 << 18, 1 << 20}

	benchStoreConfigs = createBenchStoreConfigs()
)

func randomBytes(n int, random *rand.Rand) []byte {
	if n == 0 {
		return []byte{}
	}
	k := (n-1)/8 + 1
	b := make([]byte, k*8)
	for i := range k {
		binary.BigEndian.PutUint64(b[i*8:], random.Uint64())
	}
	return b[:n]
}

func randomByte(random *rand.Rand) byte {
	return byte(random.UintN(256))
}

// Returns a random key of with length chosen from a roughly normal distribution
// with the given mean. Lengths will range from 0 to 2*mean.
func randomKey(meanLen int, random *rand.Rand) []byte {
	const bound = 4.0 // chosen experimentally
	val := random.NormFloat64()
	for val < -bound || val > +bound {
		val = random.NormFloat64()
	}
	// val is in [-bound, +bound], translate that to [0, 2*mean]
	val = (val + bound) * float64(meanLen) / bound
	return randomBytes(int(math.Round(val)), random)
}

func randomFixedLengthKey(keyLen int, random *rand.Rand) []byte {
	return randomBytes(keyLen, random)
}

func shuffle[S ~[]E, E any](slice S, random *rand.Rand) {
	random.Shuffle(len(slice), func(i, j int) {
		slice[i], slice[j] = slice[j], slice[i]
	})
}

func BenchmarkTraverser(b *testing.B) {
	benchTraverser(b, "kind=pre-order", kv.TestingPreOrder)
	benchTraverser(b, "kind=post-order", kv.TestingPostOrder)
}

func benchTraverser(b *testing.B, name string, traverser kv.TestingTraverser) {
	b.Run(name, func(b *testing.B) {
		for _, adj := range []kv.TestingAdjFunction{
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
				for b.Loop() {
					for node := range traverser(0, adj) {
						_ = node
					}
				}
			})
		}
	})
}

func BenchmarkTraverserPaths(b *testing.B) {
	benchTraverserPaths(b, "kind=pre-order", kv.TestingPreOrderPaths)
	benchTraverserPaths(b, "kind=post-order", kv.TestingPostOrderPaths)
}

func benchTraverserPaths(b *testing.B, name string, pathTraverser kv.TestingPathTraverser) {
	b.Run(name, func(b *testing.B) {
		for _, pathAdj := range []kv.TestingPathAdjFunction{
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
				for b.Loop() {
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
			keySet{empty, nextKey(empty), after},
		},
		{
			From(nil).To(nextKey(empty)),
			keySet{empty, nextKey(empty), nextKey(nextKey(empty)), after},
		},
		{
			From(nil).To(high),
			keySet{
				empty, nextKey(empty), before, high[:1], high[:2], high[:3],
				prevKey(high), high, nextKey(high), after,
			},
		},
		{
			From(nil).To(nil),
			keySet{empty, nextKey(empty), within},
		},
		{
			From(empty).To(nextKey(empty)),
			keySet{empty, nextKey(empty), nextKey(nextKey(empty)), after},
		},
		{
			From(empty).To(high),
			keySet{
				empty, nextKey(empty), before, high[:1], high[:2], high[:3],
				prevKey(high), high, nextKey(high), after,
			},
		},
		{
			From(empty).To(nil),
			keySet{empty, nextKey(empty), within},
		},
		{
			From(nextKey(empty)).To(high),
			keySet{
				empty, nextKey(empty), nextKey(nextKey(empty)), before, high[:1], high[:2], high[:3],
				prevKey(high), high, nextKey(high), after,
			},
		},
		{
			From(nextKey(empty)).To(nil),
			keySet{empty, nextKey(empty), nextKey(nextKey(empty)), within},
		},
		{
			From(low).To(high),
			keySet{
				empty, nextKey(empty), before, low[:1], low[:2], low[:3], prevKey(low), low, nextKey(low),
				within, high[:1], high[:2], high[:3], prevKey(high), high, nextKey(high), after,
			},
		},
		{
			From(low).To(nil),
			keySet{empty, nextKey(empty), before, low[:1], low[:2], low[:3], prevKey(low), low, nextKey(low), after},
		},

		// 2 byte common prefix
		{
			From(low).To(low2),
			keySet{
				empty, nextKey(empty), before, low[:1], low[:2], low[:3], prevKey(low), low, nextKey(low),
				midLows, low2[:3], prevKey(low2), low2, nextKey(low2), after,
			},
		},

		// From is a prefix of To
		{
			From(low[:2]).To(low),
			keySet{
				empty, nextKey(empty), before, low[:1], prevKey(low[:2]), low[:2], nextKey(low[:2]), low[:3],
				prevKey(low), low, nextKey(low), after,
			},
		},
	} {
		b.Run(fmt.Sprintf("bounds=%s", tt.bounds), func(b *testing.B) {
			forward := tt.bounds
			reverse := From(tt.bounds.End).DownTo(tt.bounds.Begin)
			for _, k := range tt.keys {
				b.Run("key="+kv.KeyName(k), func(b *testing.B) {
					b.Run("dir=forward", func(b *testing.B) {
						for b.Loop() {
							kv.TestingChildBounds(forward, k)
						}
					})
					b.Run("dir=reverse", func(b *testing.B) {
						for b.Loop() {
							kv.TestingChildBounds(reverse, k)
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
	for k := range entries {
		keyLen := len(k)
		for i := len(present); i <= keyLen; i++ {
			present = append(present, keySet{})
		}
		present[keyLen] = append(present[keyLen], []byte(k))
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

func createBenchStoreConfig(corpusName string, entries map[string]byte) *storeConfig {
	random := rand.New(rand.NewPCG(uint64(len(entries)), 4839028453))
	present := createPresent(entries, random)
	absent := createAbsent(entries, len(present)-1, random)
	keys := append(slices.Concat(present...), slices.Concat(absent...)...)
	shuffle(keys, random)
	forward, reverse := createBounds(keys)
	return &storeConfig{
		name:    fmt.Sprintf("corpus=%s/size=%d", corpusName, len(entries)),
		size:    len(entries),
		entries: entries,
		present: present,
		absent:  absent,
		forward: forward,
		reverse: reverse,
	}
}

func createBenchRandomStoreConfigs() []*storeConfig {
	result := []*storeConfig{}
	for _, size := range benchRandomSizes {
		result = append(result, createBenchStoreConfig("random", randomEntries(size)))
	}
	return result
}

// Config with lower-case english words, values are all 0.
func createBenchWordStoreConfigs() []*storeConfig {
	return []*storeConfig{createBenchStoreConfig("words", entriesFromFile(filenameWords))}
}

func createBenchStoreConfigs() []*storeConfig {
	return append(createBenchRandomStoreConfigs(), createBenchWordStoreConfigs()...)
}

func TestBenchStoreConfigs(t *testing.T) {
	t.Parallel()
	for _, config := range benchStoreConfigs {
		t.Run(config.name, func(t *testing.T) {
			t.Parallel()
			assert.Len(t, config.entries, config.size)
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
				for _, k := range config.absent[i] {
					assert.Len(t, k, i)
					_, ok := present[string(k)]
					assert.False(t, ok)
				}
				for _, k := range config.present[i] {
					assert.Len(t, k, i)
					_, ok := present[string(k)]
					assert.True(t, ok)
					delete(present, string(k))
				}
			}
			assert.Empty(t, present)
		})
	}
}

func TestBenchStoreConfigRepeatability(t *testing.T) {
	t.Parallel()
	for i, config := range createBenchStoreConfigs() {
		t.Run(config.name, func(t *testing.T) {
			t.Parallel()
			assert.True(t, reflect.DeepEqual(benchStoreConfigs[i], config))
		})
	}
}

// This helps to understand how factory() can impact other benchmarks which use it.
func BenchmarkFactory(b *testing.B) {
	for _, def := range implDefs {
		b.Run(def.name, func(b *testing.B) {
			for b.Loop() {
				_ = def.factory()
			}
		})
	}
}

func BenchmarkCreate(b *testing.B) {
	for _, bench := range createTestStores(benchStoreConfigs) {
		b.Run(bench.name, func(b *testing.B) {
			for b.Loop() {
				store := bench.def.factory()
				for k, v := range bench.config.entries {
					store.Set([]byte(k), v)
				}
			}
		})
	}
}

// For mutable implementations, Clone() should be efficient, but not absurdly efficient.
// If it is, that's a sign it's sharing storage instead of creating new storage.
// This also helps to understand how Clone() can impact other benchmarks which use it.
func BenchmarkClone(b *testing.B) {
	for _, bench := range createTestStores(benchStoreConfigs) {
		original := bench.store
		b.Run(bench.name, func(b *testing.B) {
			for b.Loop() {
				_ = original.Clone()
			}
		})
	}
}

// This benchmark is for memory allocations, not time.
// Creates one store and sets many keys per benchmark iteration.
func BenchmarkSparse(b *testing.B) {
	random := rand.New(rand.NewPCG(12337405, 432843980))
	var keys keySet
	for k := range 1 << 8 {
		keyByte := byte(k)
		keys = append(keys, []byte{keyByte, keyByte, keyByte, keyByte})
	}
	shuffle(keys, random)
	for _, def := range implDefs {
		b.Run(def.name, func(b *testing.B) {
			for b.Loop() {
				store := def.factory()
				for _, k := range keys {
					store.Set(k, 0)
				}
			}
		})
	}
}

// This benchmark is for memory allocations, not time.
// Creates one store and sets many keys per benchmark iteration.
func BenchmarkDense(b *testing.B) {
	random := rand.New(rand.NewPCG(9321075532, 1293487543289))
	oneKeys := make(keySet, 1<<8)
	twoKeys := make(keySet, 1<<16)
	threeKeys := make(keySet, 1<<24)
	for k := range 1 << 8 {
		oneKeys[k] = []byte{byte(k)}
	}
	for k := range 1 << 16 {
		keyBytes := binary.LittleEndian.AppendUint16(nil, uint16(k))
		twoKeys[k] = []byte{keyBytes[0], keyBytes[1]}
	}
	for k := range 1 << 24 {
		keyBytes := binary.LittleEndian.AppendUint32(nil, uint32(k))
		threeKeys[k] = []byte{keyBytes[0], keyBytes[1], keyBytes[2]}
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
			b.Run(def.name+tt.name, func(b *testing.B) {
				for b.Loop() {
					store := def.factory()
					for _, k := range tt.keys {
						store.Set(k, 0)
					}
				}
			})
		}
	}
}

//nolint:gocognit
func BenchmarkSet(b *testing.B) {
	for _, bench := range createTestStores(benchStoreConfigs) {
		original := bench.store
		b.Run(bench.name, func(b *testing.B) {
			for keyLen := 8; keyLen < len(bench.config.present); keyLen += 4 {
				b.Run(fmt.Sprintf("keyLen=%d/existing=true", keyLen), func(b *testing.B) {
					present := bench.config.present[keyLen]
					if len(present) == 0 {
						b.Skipf("no present keys of length %d", keyLen)
					}
					store := original.Clone()
					i := 0
					for b.Loop() {
						store.Set(present[i%len(present)], 42)
						i++
					}
				})
				b.Run(fmt.Sprintf("keyLen=%d/existing=false", keyLen), func(b *testing.B) {
					absent := bench.config.absent[keyLen]
					if len(absent) == 0 {
						b.Skipf("no absent keys of length %d", keyLen)
					}
					store := original.Clone()
					i := 0
					for b.Loop() {
						if i%len(absent) == 0 && i > 0 {
							b.StopTimer()
							store = original.Clone()
							b.StartTimer()
						}
						store.Set(absent[i%len(absent)], 42)
						i++
					}
				})
			}
		})
	}
}

//nolint:gocognit
func BenchmarkGet(b *testing.B) {
	for _, bench := range createTestStores(benchStoreConfigs) {
		original := bench.store
		b.Run(bench.name, func(b *testing.B) {
			for keyLen := 8; keyLen < len(bench.config.present); keyLen += 4 {
				b.Run(fmt.Sprintf("keyLen=%d/existing=true", keyLen), func(b *testing.B) {
					present := bench.config.present[keyLen]
					if len(present) == 0 {
						b.Skipf("no present keys of length %d", keyLen)
					}
					store := original.Clone()
					i := 0
					for b.Loop() {
						store.Get(present[i%len(present)])
						i++
					}
				})
				b.Run(fmt.Sprintf("keyLen=%d/existing=false", keyLen), func(b *testing.B) {
					absent := bench.config.absent[keyLen]
					if len(absent) == 0 {
						b.Skipf("no absent keys of length %d", keyLen)
					}
					store := original.Clone()
					i := 0
					for b.Loop() {
						store.Get(absent[i%len(absent)])
						i++
					}
				})
			}
		})
	}
}

//nolint:gocognit
func BenchmarkDelete(b *testing.B) {
	for _, bench := range createTestStores(benchStoreConfigs) {
		original := bench.store
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
					store := original.Clone()
					i := 0
					for b.Loop() {
						if i%len(present) == 0 && i > 0 {
							b.StopTimer()
							store = original.Clone()
							b.StartTimer()
						}
						store.Delete(present[i%len(present)])
						i++
					}
				})
				b.Run(fmt.Sprintf("keyLen=%d/existing=false", keyLen), func(b *testing.B) {
					absent := bench.config.absent[keyLen]
					if len(absent) == 0 {
						b.Skipf("no absent keys of length %d", keyLen)
					}
					store := original.Clone()
					i := 0
					for b.Loop() {
						store.Delete(absent[i%len(absent)])
						i++
					}
				})
			}
		})
	}
}

//nolint:gocognit
func benchRange(b *testing.B, getBounds func(*testStore) ([]Bounds, []Bounds)) {
	for _, bench := range createTestStores(benchStoreConfigs) {
		forward, reverse := getBounds(bench)
		original := bench.store
		store := original.Clone()
		b.Run(bench.name, func(b *testing.B) {
			// This is a hack, but good enough for now.
			// The words corpus is not uniformly random, unlike the forward/reverse ranges being used.
			if strings.Contains(bench.name, "corpus=words") {
				b.Skip()
			}
			b.Run("dir=forward/op=range", func(b *testing.B) {
				i := 0
				for b.Loop() {
					store.Range(&forward[i%len(forward)])
					i++
				}
			})
			b.Run("dir=forward/op=full", func(b *testing.B) {
				i := 0
				for b.Loop() {
					for k, v := range store.Range(&forward[i%len(forward)]) {
						_, _ = k, v
					}
					i++
				}
			})
			b.Run("dir=reverse/op=range", func(b *testing.B) {
				i := 0
				for b.Loop() {
					store.Range(&reverse[i%len(reverse)])
					i++
				}
			})
			b.Run("dir=reverse/op=full", func(b *testing.B) {
				i := 0
				for b.Loop() {
					for k, v := range store.Range(&reverse[i%len(reverse)]) {
						_, _ = k, v
					}
					i++
				}
			})
		})
	}
}

func BenchmarkShortRange(b *testing.B) {
	random := rand.New(rand.NewPCG(74320567, 6234982127))
	forward, reverse := createFixedBounds(0x00_00_00_83, random)
	benchRange(b, func(_ *testStore) ([]Bounds, []Bounds) {
		return forward, reverse
	})
}

func BenchmarkLongRange(b *testing.B) {
	random := rand.New(rand.NewPCG(48239752, 80321318701))
	forward, reverse := createFixedBounds(0x00_02_13_13, random)
	benchRange(b, func(_ *testStore) ([]Bounds, []Bounds) {
		return forward, reverse
	})
}

func BenchmarkRandomRange(b *testing.B) {
	benchRange(b, func(tt *testStore) ([]Bounds, []Bounds) {
		return tt.config.forward, tt.config.reverse
	})
}
