#!/bin/bash

# This script is for running benchmark tests during development.
# A typical use would look like:
#
#   $ ./bench.sh Foo 3
#
# Or if "Foo" unintentionally matches multiple benchmarks:
#
#   $ ./bench.sh "BenchmarkFoo$" 3

set -e

cd "$(dirname "$0")"
rm -f btrie.test
go test -c

tests=${1:-.}
count=${2:-10}
go test -bench "${tests}" -benchmem -timeout 0 -count=${count}
