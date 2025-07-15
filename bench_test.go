package kv_test

import (
	"bufio"
	"encoding/binary"
	"fmt"
	"math"
	rand "math/rand/v2"
	"os"
	"slices"
	"testing"

	"github.com/phiryll/kv"
)

// No benchmark can have a truly random element, random seeds must be constants.

const (
	benchRandomMeanKeyLen = 8

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

// Returns a function which repeatedly loops over the elements of itr,
// invoking reset after every full iteration.
func repeat[V any](s []V, reset func()) func() V {
	if len(s) == 0 {
		panic("cannot repeat empty")
	}
	i := 0
	return func() V {
		value := s[i]
		i++
		if i == len(s) {
			i = 0
			if reset != nil {
				reset()
			}
		}
		return value
	}
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
		// No need to benchmark reverse bounds, the code is the same.

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
			From(nil).To(highKey),
			keySet{
				empty,
				highKey[:1], highKey[:2], highKey[:3], prevKey(highKey), highKey, nextKey(highKey),
				after,
			},
		},
		{
			From(nil).To(nil),
			keySet{empty, nextKey(empty), after},
		},
		{
			From(empty).To(nextKey(empty)),
			keySet{empty, nextKey(empty), nextKey(nextKey(empty)), after},
		},
		{
			From(empty).To(highKey),
			keySet{
				empty, nextKey(empty),
				highKey[:1], highKey[:2], highKey[:3], prevKey(highKey), highKey, nextKey(highKey),
				after,
			},
		},
		{
			From(empty).To(nil),
			keySet{empty, nextKey(empty), after},
		},
		{
			From(nextKey(empty)).To(highKey),
			keySet{
				empty, nextKey(empty), nextKey(nextKey(empty)),
				highKey[:1], highKey[:2], highKey[:3], prevKey(highKey), highKey, nextKey(highKey),
				after,
			},
		},
		{
			From(nextKey(empty)).To(nil),
			keySet{empty, nextKey(empty), nextKey(nextKey(empty)), after},
		},
		{
			From(lowKey).To(highKey),
			keySet{
				empty, nextKey(empty),
				lowKey[:1], lowKey[:2], lowKey[:3], prevKey(lowKey), lowKey, nextKey(lowKey),
				highKey[:1], highKey[:2], highKey[:3], prevKey(highKey), highKey, nextKey(highKey),
				after,
			},
		},
		{
			From(lowKey).To(nil),
			keySet{
				empty, nextKey(empty),
				lowKey[:1], lowKey[:2], lowKey[:3], prevKey(lowKey), lowKey, nextKey(lowKey),
				after,
			},
		},

		// 2 byte common prefix
		{
			From(lowKey).To(low2),
			keySet{
				empty, nextKey(empty), before,
				lowKey[:1], lowKey[:2], lowKey[:3], prevKey(lowKey), lowKey, nextKey(lowKey),
				midLows,
				low2[:3], prevKey(low2), low2, nextKey(low2),
				after,
			},
		},

		// From is a prefix of To
		{
			From(lowKey[:2]).To(lowKey),
			keySet{
				empty, nextKey(empty), before,
				lowKey[:1], prevKey(lowKey[:2]), lowKey[:2], nextKey(lowKey[:2]),
				lowKey[:3], prevKey(lowKey), lowKey, nextKey(lowKey),
				after,
			},
		},
	} {
		b.Run(fmt.Sprintf("bounds=%s", tt.bounds), func(b *testing.B) {
			for _, k := range tt.keys {
				b.Run("key="+kv.KeyName(k), func(b *testing.B) {
					for b.Loop() {
						kv.TestingChildBounds(tt.bounds, k)
					}
				})
			}
		})
	}
}

func randomEntries(numEntries int) map[string]byte {
	random := rand.New(rand.NewPCG(uint64(numEntries), 83741074321))
	entries := map[string]byte{}
	for count := 0; count < numEntries; {
		key := string(randomKey(benchRandomMeanKeyLen, random))
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

func createBenchStoreConfig(corpusName string, entries map[string]byte) *storeConfig {
	ref := newReference()
	for k, v := range entries {
		ref.Set([]byte(k), v)
	}
	ref.refresh()
	return &storeConfig{
		name: fmt.Sprintf("corpus=%s/size=%d", corpusName, len(entries)),
		size: len(entries),
		ref:  ref,
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

// This helps to understand how factory() can impact other benchmarks which use it.
func BenchmarkFactory(b *testing.B) {
	for _, def := range implDefs {
		b.Run(def.name, func(b *testing.B) {
			for b.Loop() {
				def.factory()
			}
		})
	}
}

func BenchmarkCreate(b *testing.B) {
	for _, config := range createBenchStoreConfigs() {
		for _, def := range implDefs {
			b.Run(config.name+"/"+def.name, func(b *testing.B) {
				for b.Loop() {
					store := def.factory()
					for k, v := range config.ref.Asc(nil, nil) {
						store.Set(k, v)
					}
				}
			})
		}
	}
}

func BenchmarkGet(b *testing.B) {
	for store := range createStoresUnderTest(slices.Values(benchStoreConfigs)) {
		b.Run(store.name, func(b *testing.B) {
			b.Run("existing=true", func(b *testing.B) {
				next := repeat(slices.Collect(keyIter(store.config.ref.All())), nil)
				for b.Loop() {
					store.Get(next())
				}
			})
			b.Run("existing=false", func(b *testing.B) {
				next := repeat(slices.Collect(absentKeys(store.config.ref)), nil)
				for b.Loop() {
					store.Get(next())
				}
			})
		})
	}
}

func BenchmarkSet(b *testing.B) {
	for store := range createStoresUnderTest(slices.Values(benchStoreConfigs)) {
		b.Run(store.name, func(b *testing.B) {
			b.Run("existing=true", func(b *testing.B) {
				next := repeat(slices.Collect(keyIter(store.config.ref.All())), nil)
				for b.Loop() {
					store.Set(next(), 42)
				}
			})
			b.Run("existing=false", func(b *testing.B) {
				next := repeat(slices.Collect(absentKeys(store.config.ref)), func() {
					b.StopTimer()
					store.resetFromConfig()
					b.StartTimer()
				})
				for b.Loop() {
					store.Set(next(), 42)
				}
			})
		})
	}
}

func BenchmarkDelete(b *testing.B) {
	for store := range createStoresUnderTest(slices.Values(benchStoreConfigs)) {
		b.Run(store.name, func(b *testing.B) {
			b.Run("existing=true", func(b *testing.B) {
				next := repeat(slices.Collect(keyIter(store.config.ref.All())), func() {
					b.StopTimer()
					store.resetFromConfig()
					b.StartTimer()
				})
				for b.Loop() {
					store.Delete(next())
				}
			})
			b.Run("existing=false", func(b *testing.B) {
				next := repeat(slices.Collect(absentKeys(store.config.ref)), nil)
				for b.Loop() {
					store.Delete(next())
				}
			})
		})
	}
}

// Like rangePairs(s), but repeating, and random.
// Assumes s is sorted.
func randomPairs[V any](s []V) func() (V, V) {
	n := uint(len(s))
	if n <= 1 {
		panic("must have at least 2 elements")
	}
	random := rand.New(rand.NewPCG(3107354753921, 83741074321))
	return func() (V, V) {
		var i, j uint
		for i == j {
			i, j = random.UintN(n), random.UintN(n)
		}
		if i > j {
			i, j = j, i
		}
		return s[i], s[j]
	}
}

func BenchmarkAll(b *testing.B) {
	for store := range createStoresUnderTest(slices.Values(benchStoreConfigs)) {
		b.Run(store.name, func(b *testing.B) {
			b.Run("op=init", func(b *testing.B) {
				for b.Loop() {
					store.Range(forwardAll)
				}
			})
			b.Run("op=full", func(b *testing.B) {
				for b.Loop() {
					for k, v := range store.Range(forwardAll) {
						_, _ = k, v
					}
				}
			})
		})
	}
}

//nolint:gocognit
func benchRange(b *testing.B, bounds func(low, high []byte) *Bounds) {
	for store := range createStoresUnderTest(slices.Values(benchStoreConfigs)) {
		b.Run(store.name, func(b *testing.B) {
			keys := boundKeys(store.config.ref)
			b.Run("op=init", func(b *testing.B) {
				next := randomPairs(keys)
				for b.Loop() {
					store.Range(bounds(next()))
				}
			})
			b.Run("op=full", func(b *testing.B) {
				if d, ok := store.ByteStore.(dirtyable); ok {
					d.refresh()
				}
				next := randomPairs(keys)
				for b.Loop() {
					for k, v := range store.Range(bounds(next())) {
						_, _ = k, v
					}
				}
			})
			b.Run("op=full-dirty", func(b *testing.B) {
				d, ok := store.ByteStore.(dirtyable)
				if !ok {
					b.Skipf("skipping store of type %T", store.ByteStore)
				}
				next := randomPairs(keys)
				for b.Loop() {
					d.makeDirty()
					for k, v := range store.Range(bounds(next())) {
						_, _ = k, v
					}
				}
			})
		})
	}
}

func BenchmarkAsc(b *testing.B) {
	benchRange(b, func(low, high []byte) *Bounds { return From(low).To(high) })
}

func BenchmarkDesc(b *testing.B) {
	benchRange(b, func(low, high []byte) *Bounds { return From(high).DownTo(low) })
}
