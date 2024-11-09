#!/bin/bash

freq=$1

ffplay -f lavfi -i "sine=frequency=$freq" -nodisp
