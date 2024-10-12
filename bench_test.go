package btrie_test

import (
	"testing"

	"github.com/phiryll/btrie"
)

func BenchmarkPreOrder(b *testing.B) {
	for _, tt := range []struct {
		name string
		adj  btrie.TestingAdjFunction
	}{
		// The number in parentheses is the number of paths in the traversal.
		{"empty (1)", emptyAdjInt},
		{"limit 0 (4)", adjInt(0)},
		{"limit 2^4 (40)", adjInt(1 << 4)},
		{"limit 2^8 (364)", adjInt(1 << 8)},
		{"limit 2^12 (3280)", adjInt(1 << 12)},
		{"limit 2^16 (29524)", adjInt(1 << 16)},
		{"limit 2^20 (265720)", adjInt(1 << 20)},
	} {
		b.Run(tt.name, func(b *testing.B) {
			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				for path := range btrie.TestingPreOrder(0, tt.adj) {
					_ = path
				}
			}
		})
	}
}
