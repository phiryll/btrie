#!/bin/bash

# This is what works on my mac, using homebrew versions of grep and sed.
# The BSD versions supplied with mac os don't work the same way.

set -e

cd "$(dirname "$0")"
rm -f kv.test
go test -c

fuzzTime=${1:-30}

files=$(ggrep -r --include='**_test.go' --files-with-matches 'func Fuzz' .)

for file in ${files}
do
    funcs=$(gsed -nr 's/func (Fuzz\w+).*/\1/p' $file)
    for func in ${funcs}
    do
        echo "Fuzzing $func in $file"
        parentDir=$(dirname $file)
        go test $parentDir -fuzz=$func -fuzztime=${fuzzTime}s
    done
done
