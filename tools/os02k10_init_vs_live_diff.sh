#!/usr/bin/env bash
set -euo pipefail

# os02k10_init_vs_live_diff.sh — compare the recovered OS02K10 init table
# against a live register dump and report what changed.
#
# Usage:
#   tools/os02k10_init_vs_live_diff.sh <init.csv> <live.txt>
#
#   <init.csv>  documentation/os02k10/os02k10_registers.csv format
#               ("register,value,group,meaning", header row required)
#   <live.txt>  tools/os02k10_i2c_dump output format ("0xADDR 0xVAL" or
#               "0xADDR ERR" per line)
#
# Output (stdout, tab-separated so it stays diffable/greppable):
#   register  init_value  live_value  changed
#
# A register present in <live.txt> but not in <init.csv> (or vice versa) is
# reported with the missing side as "-". "changed" is yes/no/unknown (the
# live read failed, e.g. "ERR").
#
# Typical use, after capturing a live dump on the target:
#   ssh root@<ascent-ip> /tmp/os02k10_i2c_dump > /tmp/os02k10_live.txt
#   tools/os02k10_init_vs_live_diff.sh \
#       documentation/os02k10/os02k10_registers.csv /tmp/os02k10_live.txt

if [ "$#" -ne 2 ]; then
	echo "usage: $0 <init.csv> <live.txt>" >&2
	exit 2
fi

INIT_CSV="$1"
LIVE_TXT="$2"

awk -v init_csv="$INIT_CSV" -v live_txt="$LIVE_TXT" '
	BEGIN {
		FS = ","
		while ((getline line < init_csv) > 0) {
			n = split(line, f, ",")
			if (f[1] == "register") continue
			reg = tolower(f[1])
			init_val[reg] = tolower(f[2])
			order[++norder] = reg
		}
		close(init_csv)

		while ((getline line < live_txt) > 0) {
			if (line == "" || substr(line, 1, 1) == "#") continue
			split(line, f, " ")
			reg = tolower(f[1])
			val = tolower(f[2])
			if (!(reg in init_val)) {
				order[++norder] = reg
			}
			live_val[reg] = val
			have_live[reg] = 1
		}
		close(live_txt)

		print "register\tinit_value\tlive_value\tchanged"
		for (i = 1; i <= norder; i++) {
			reg = order[i]
			if (seen[reg]) continue
			seen[reg] = 1

			iv = (reg in init_val) ? init_val[reg] : "-"
			lv = (reg in have_live) ? live_val[reg] : "-"

			if (iv == "-" || lv == "-") {
				changed = "unknown"
			} else if (lv == "err") {
				changed = "unknown"
			} else if (iv == lv) {
				changed = "no"
			} else {
				changed = "yes"
			}
			print reg "\t" iv "\t" lv "\t" changed
		}
	}
'
