#!/usr/bin/env bash

./mav_uart | stdbuf -oL \grep ICAO | while read LINE; do printf "time=$(($(date +%s%N)/1000000)) "; printf "$LINE\n"; done
