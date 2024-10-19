package btrie_test

import (
	"bytes"
	"fmt"
	"iter"
	"math/bits"
	"math/rand"
	"testing"

	"github.com/phiryll/btrie"
	"github.com/stretchr/testify/assert"
)

// No benchmark can have a truly random element, random seeds must be constants!

const (
	maxTrieSize = 1 << 20 // ~1M
	maxKeySize  = 5
)

type (
	trieDef struct {
		name    string
		factory func() TestBTrie
	}

	trieConfig struct {
		keySize  int
		trieSize int
		seed     int64
	}

	benchConfig struct {
		b      *testing.B
		def    trieDef
		config trieConfig
	}
)

var (
	trieDefs = []trieDef{
		{"reference", newReference},
		{"pointer-trie", btrie.NewPointerTrie[byte]},
	}

	trieConfigs = createTrieConfigs()

	randomKeysByLength = [][][]byte{
		randomKeysOfLength(0, rand.New(rand.NewSource(93012741))),
		randomKeysOfLength(1, rand.New(rand.NewSource(37468914))),
		randomKeysOfLength(2, rand.New(rand.NewSource(129457732))),
		randomKeysOfLength(3, rand.New(rand.NewSource(652340902))),
		randomKeysOfLength(4, rand.New(rand.NewSource(8912361))),
		randomKeysOfLength(5, rand.New(rand.NewSource(319090564))),
		randomKeysOfLength(6, rand.New(rand.NewSource(5109310239))),
	}
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
				for i := 0; i < b.N; i++ {
					for path := range traverser(0, adj) {
						_ = path
					}
				}
			})
		}
	})
}

// Because these are stateful data structures, accurately benchmarking mutating methods is difficult.
// Using the B.Start/StopTimer methods inside the B.N loops results in erratic stats,
// and sometimes they just hang forever.
// So it's not possible to create a new non-trivial trie fixture inside the B.N loop
// without the trie's creation also being measured, or at least the B.Start/Stop methods interfering.

func createTrieConfigs() []trieConfig {
	configs := []trieConfig{}
	random := rand.New(rand.NewSource(3472109))
	for keySize := 1; keySize <= maxKeySize; keySize++ {
		maxKey := 1 << (keySize*8 - 1)
		if maxKey > maxTrieSize {
			maxKey = maxTrieSize
		}
		for trieSize := maxKey >> 4; trieSize <= maxKey; trieSize <<= 1 {
			configs = append(configs, trieConfig{keySize, trieSize, random.Int63()})
		}
	}
	return configs
}

func TestTrieConfigs(t *testing.T) {
	t.Parallel()
	for _, config := range trieConfigs {
		assert.LessOrEqual(t, config.keySize, maxKeySize, "%v", config)
		assert.LessOrEqual(t, config.trieSize, maxTrieSize, "%v", config)
		// Ensure that trieSize < half the number of possible keys.
		assert.LessOrEqual(t, bits.Len(uint(config.trieSize)), config.keySize*8, "%v", config)
	}
}

// Returns (trie, presentKeys)
// Any key longer than c.keySize is absent.
func (c trieConfig) createTrie(factory func() TestBTrie) (TestBTrie, [][]byte) {
	count := 0
	trie := factory()
	var present [][]byte
	random := rand.New(rand.NewSource(c.seed))
	for count < c.trieSize {
		key := randomKey(c.keySize, random)
		if _, ok := trie.Put(key, randomByte(random)); !ok {
			present = append(present, key)
			count++
		}
	}
	return trie, present
}

// Just testing that the tries can be created without failing or timing out.
func TestCreateTrie(t *testing.T) {
	t.Parallel()
	for _, def := range trieDefs {
		t.Run("impl="+def.name, func(t *testing.T) {
			t.Parallel()
			for _, config := range trieConfigs {
				t.Run(fmt.Sprintf("keyLen=%d/trieSize=%d", config.keySize, config.trieSize), func(t *testing.T) {
					t.Parallel()
					config.createTrie(def.factory)
				})
			}
		})
	}
}

func (b benchConfig) Run(name string, f func(*testing.B)) {
	b.b.Run(name, f)
}

func benchmarkConfigs(b *testing.B) iter.Seq[benchConfig] {
	return func(yield func(benchConfig) bool) {
		for _, def := range trieDefs {
			b.Run("impl="+def.name, func(b *testing.B) {
				for _, config := range trieConfigs {
					b.Run(fmt.Sprintf("keyLen=%d/trieSize=%d", config.keySize, config.trieSize), func(b *testing.B) {
						if !yield(benchConfig{b, def, config}) {
							return
						}
					})
				}
			})
		}
	}
}

func randomKeysOfLength(keySize int, random *rand.Rand) [][]byte {
	keys := [][]byte{}
	var numKeys int
	switch keySize {
	case 0:
		return [][]byte{{}}
	case 1:
		for low := range 256 {
			keys = append(keys, []byte{byte(low)})
		}
		return keys
	case 2:
		for hi := range 256 {
			for low := range 256 {
				keys = append(keys, []byte{byte(hi), byte(low)})
			}
		}
		return keys
	case 3, 4, 5, 6:
		numKeys = 1 << 16
	default:
		numKeys = 1 << 16
	}
	b := make([]byte, keySize)
	for range numKeys {
		_, _ = random.Read(b)
		keys = append(keys, bytes.Clone(b))
	}
	return keys
}

func BenchmarkPut(b *testing.B) {
	for bench := range benchmarkConfigs(b) {
		bench.Run("key=present", func(b *testing.B) {
			trie, present := bench.config.createTrie(bench.def.factory)
			b.ResetTimer()
			for i := range b.N {
				trie.Put(present[i%len(present)], 42)
			}
		})
		bench.Run("key=absent", func(b *testing.B) {
			trie, _ := bench.config.createTrie(bench.def.factory)
			absent := randomKeysByLength[bench.config.keySize+1]
			b.ResetTimer()
			for i := range b.N {
				if i%len(absent) == 0 && i > 0 {
					b.StopTimer()
					// TODO: a clone() method could help minimize the impact here
					trie, _ = bench.config.createTrie(bench.def.factory)
					b.StartTimer()
				}
				trie.Put(absent[i%len(absent)], 42)
			}
		})
	}
}

func BenchmarkGet(b *testing.B) {
	for bench := range benchmarkConfigs(b) {
		trie, present := bench.config.createTrie(bench.def.factory)
		bench.Run("key=present", func(b *testing.B) {
			b.ResetTimer()
			for i := range b.N {
				trie.Get(present[i%len(present)])
			}
		})
		bench.Run("key=absent", func(b *testing.B) {
			absent := randomKeysByLength[bench.config.keySize+1]
			b.ResetTimer()
			for i := range b.N {
				trie.Get(absent[i%len(absent)])
			}
		})
	}
}

func BenchmarkDelete(b *testing.B) {
	for bench := range benchmarkConfigs(b) {
		bench.Run("key=present", func(b *testing.B) {
			trie, present := bench.config.createTrie(bench.def.factory)
			b.ResetTimer()
			for i := range b.N {
				if i%len(present) == 0 && i > 0 {
					b.StopTimer()
					// TODO: a clone() method could help minimize the impact here
					trie, _ = bench.config.createTrie(bench.def.factory)
					b.StartTimer()
				}
				trie.Delete(present[i%len(present)])
			}
		})
		bench.Run("key=absent", func(b *testing.B) {
			trie, _ := bench.config.createTrie(bench.def.factory)
			absent := randomKeysByLength[bench.config.keySize+1]
			b.ResetTimer()
			for i := range b.N {
				trie.Delete(absent[i%len(absent)])
			}
		})
	}
}

func BenchmarkRange(b *testing.B) {
	for bench := range benchmarkConfigs(b) {
		trie, keys := bench.config.createTrie(bench.def.factory)
		keys = append(keys, randomKeysByLength[bench.config.keySize+1]...)
		random := rand.New(rand.NewSource(247850342))
		random.Shuffle(len(keys), func(i, j int) {
			keys[i], keys[j] = keys[j], keys[i]
		})
		forward := []Bounds{}
		reverse := []Bounds{}
		for i := range keys {
			begin := keys[(2*i)%len(keys)]
			end := keys[(2*i+1)%len(keys)]
			switch cmp := bytes.Compare(begin, end); {
			case cmp == 0:
				forward = append(forward, From(begin).To(append(end, 0)))
				reverse = append(reverse, From(append(begin, 0)).DownTo(end))
			case cmp < 0:
				forward = append(forward, From(begin).To(end))
				reverse = append(reverse, From(end).DownTo(begin))
			case cmp > 0:
				forward = append(forward, From(end).To(begin))
				reverse = append(reverse, From(begin).DownTo(end))
			}
		}
		bench.Run("dir=forward", func(b *testing.B) {
			b.ResetTimer()
			for i := range b.N {
				bounds := forward[i%len(forward)]
				for k, v := range trie.Range(bounds) {
					_, _ = k, v
				}
			}
		})
		bench.Run("dir=reverse", func(b *testing.B) {
			b.ResetTimer()
			for i := range b.N {
				bounds := reverse[i%len(reverse)]
				for k, v := range trie.Range(bounds) {
					_, _ = k, v
				}
			}
		})
	}
}
