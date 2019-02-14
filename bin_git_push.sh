#!/usr/bin/zsh

echo 'Pushing changes to Master'
echo 'Type the commit message'

read commit_message

git add -A
git commit -m $commit_message
git push origin master
