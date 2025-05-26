#!/bin/bash

# This script is for running fuzz tests during development.
# A typical use would look like:
#
#   $ ./fuzz.sh Foo 3
#
# Or if "Foo" unintentionally matches multiple benchmarks:
#
#   $ ./fuzz.sh "^FuzzFoo$" 3

set -e

cd "$(dirname "$0")"
rm -f kv.test
go test -c

tests=${1:-Get}
fuzzTime=${2:-30}

go test -fuzz=$tests -fuzztime=${fuzzTime}s
