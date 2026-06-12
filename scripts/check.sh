#!/usr/bin/env bash
set -e
sudo ethercat slaves || true
sudo ethercat pdos -p 0 || true
sudo ethercat upload -p 0 -t uint16 0x6041 0x00 || true
sudo ethercat upload -p 0 -t int32 0x6064 0x00 || true
