#!/bin/bash

rm .git/hooks/pre-commit
ln -s $PWD/scripts/githooks/pre-commit .git/hooks/pre-commit
