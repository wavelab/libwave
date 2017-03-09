#!/bin/sh

# checkout gh-pages branch
git checkout gh-pages

# get docs folder from master branch and move contents out
git checkout master -- docs
mv docs/* .
rm -rf docs

# get latest README file
git checkout master -- README.md

# commit updated docs
git add *
git commit -m "Update gh-pages"
git push

# go back to master branch
git checkout master
git clean -fd
