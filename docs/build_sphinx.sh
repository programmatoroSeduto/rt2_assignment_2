#! /bin/bash

source ../../../devel/setup.bash

cp ../README.md ./readme.md
make clean
make html
rm readme.md

firefox ./_build/html/index.html &
