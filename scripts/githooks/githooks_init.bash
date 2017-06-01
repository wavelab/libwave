#!/bin/bash

rm .git/hooks/pre-commit
ln -s $PWD/scripts/githooks/pre-commit .git/hooks/pre-commit

rm .git/hooks/pre-push
ln -s $PWD/scripts/githooks/pre-push .git/hooks/pre-push
