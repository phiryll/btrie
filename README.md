# kv

Key-Value Store Experimentation

## Preface

**DO NOT USE! (yet)**

This is still under development, it **will** change. Drastically.
This is the first attempt, intended be thrown away.

## Goals (eventually)

kv is a library with a few key-value store implementations.
These are roughly equivalent to a `map[[]byte]<value type>`,
but without a `map`'s syntax or its full semantics.
Most implementations are ordered tries.
Some implementations are persistent,
in [this sense](https://en.wikipedia.org/wiki/Persistent_data_structure).
These are the tentative implementations, all in-memory only:

* neither persistent nor thread-safe
* partially persistent and thread-safe  
  All versions can be accessed, but only the newest can be modified.
* thread-safe  
  This uses the same mechanism as the partially persistent variant to
  allow concurrent reads with no synchronization (path copying), but
  does not allow explicitly querying previous versions. All reads use
  the data structure at the time a reference to the trie is taken,
  which can be garbage collected when it can no longer be referenced.
  This allows the trie to be compacted without affecting in-progress
  readers.
* confluently persistent and thread-safe  
  Every version can be accessed and modified, with a merge operation.

Binary tries make an excellent in-memory index for [lexicographically
byte-ordered data](https://github.com/phiryll/lexy).
