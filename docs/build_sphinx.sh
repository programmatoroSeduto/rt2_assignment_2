#! /bin/bash

cp ../README.md ./readme.md
make clean
make html
rm readme.md
