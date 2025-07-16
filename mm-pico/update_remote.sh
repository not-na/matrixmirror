#!/usr/bin/env bash

# First arg: hostname of device that Picos are connected to
# Second arg: UF2 file to upload

rsync -avzhP $2 $1:~/mm_pico.uf2

ssh $1 "
  picotool load -f ~/mm_pico.uf2
  "