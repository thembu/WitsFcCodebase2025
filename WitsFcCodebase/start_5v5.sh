#!/bin/bash
export OMP_NUM_THREADS=1

host=${1:-localhost}
port=${2:-3100}

# Your team (5 players)
for i in {1..5}; do
  python3 ./Run_Player.py -i $host -p $port -u $i -t YourTeam -P 0 -D 0 &
done

# Opponent team (5 players)
for i in {1..5}; do
  python3 ./Run_Player.py -i $host -p $port -u $i -t Opponent -P 0 -D 0 &
done