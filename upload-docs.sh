#!/bin/bash          

# Note: You will need to install both doxygen as well as graphviz packages!

rm -rf ./docs/html

doxygen doxygen.conf

cd docs/html/

git init

git remote add origin git@github.com:UWARG/PICpilot.git

git add .

git commit -m "Updated documentation"

git push origin master:gh-pages --force
