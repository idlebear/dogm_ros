#!/bin/bash

# find include \( -iname '*.h' -o -iname '*.cpp' -o -iname '*.cu' \) -print0 | xargs -0  clang-format -i
srcs=("include" "src")

for src in ${srcs[*]}; do
    for f in `find $src \( -iname '*.h' -o -iname '*.cpp' -o -iname '*.cu' \)`; do
        clang-format -i $f
    done
done
# find src \( -iname '*.h' -o -iname '*.cpp' -o -iname '*.cu' \) -print0 | xargs -0  clang-format -i

