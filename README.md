# btrie

Binary Tries

Btrie is a library with a few binary trie implementations. These are
roughly equivalent to an ordered `map[[]byte]<value type>`, but
without a `map`'s syntax or its full semantics. These are the
implementations, all in-memory only:

* neither persistent nor thread-safe
* fully persistent and thread-safe, in [this
  sense](https://en.wikipedia.org/wiki/Persistent_data_structure).
* partially persistent and thread-safe
* thread-safe  
  This uses the same mechanism as the partially persistent variant to
  allow concurrent reads with no synchronization (path copying), but
  does not allow explicitly querying previous versions. All reads use
  the data structure at the time a reference to the trie is taken,
  which can be garbage collected when it can no longer be referenced.
  This allows the trie to be compacted without affecting in-progress
  readers.

Binary tries make an excellent in-memory index for [lexicographically
byte-ordered data](https://github.com/phiryll/lexy). The principles
could also possibly be applied to a [data history tracking
database](https://phiryll.github.io/projects/data-history.html).
