#!/bin/sh

git checkout gh-pages
git checkout master -- docs
git checkout master -- index.html
git checkout master -- README.md
git add docs index.html README.md
git commit -m "Update gh-pages"
git push
