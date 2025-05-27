#!/bin/bash

# This script runs all benchmarks, writing each benchark's results
# to a separate file in the benchmarks/ directory.
# For the BenchmarkFoo function, the file would be named Foo.tmp
# This is intentional to avoid overwriting the baseline Foo.txt file,
# which is checked into version control.
#
# This script is normally only used when establishing a baseline or
# bencharking on a new system/os/....
# The same effect can be achieved for a single benchmark like this:
#
#   $ ./bench.sh "BenchmarkFoo$" 20 > benchmarks/Foo.tmp

# This is what works on my mac, using homebrew versions of grep and sed.
# The BSD versions supplied with mac os don't work the same way.

set -e

cd "$(dirname "$0")"
rm -f kv.test
go test -c

files=$(ggrep -r --include='**_test.go' -l 'func Bench' .)
count=${1:-20}

for file in ${files}
do
    funcs=$(gsed -nr 's/func (Bench\w+).*/\1/p' $file)
    for func in ${funcs}
    do
        echo "$func in $file"
        test_name=${func#Benchmark}
        go test -bench "${func}$" -benchmem -timeout 0 -count=${count} > benchmarks/${test_name}.tmp
    done
done
