#!/bin/bash
# Same-bearing (false-distance) phantom vs attacker fraction, at full and partial
# sensing -> characterises the structural blind spot (limitation figure).
# Produces: data/sweep_aligned_full.csv , data/sweep_aligned_partial.csv
cd "$(dirname "$0")/.." || exit 1
ARGOS=experiments/secureflock_single.argos
sed -i 's/ground="[^"]*"/ground="true"/; s/defense="[^"]*"/defense="true"/' $ARGOS
sed -i 's/method="\(trust\|wmsr\)"/method="trust"/; s/attacker="[^"]*"/attacker="follow"/' $ARGOS
sed -i 's/agree="[^"]*"/agree="0.0"/; s/tolerance="[^"]*"/tolerance="3.0"/' $ARGOS
sed -i 's/false_x="[^"]*"/false_x="1.0"/; s/false_y="[^"]*"/false_y="0.0"/' $ARGOS   # same bearing, wrong distance
run() {  # $1 = ground_fraction, $2 = outfile
  sed -i 's/ ground_fraction="[^"]*"//g' $ARGOS
  sed -i "s/<secureflock /<secureflock ground_fraction=\"$1\" /" $ARGOS
  echo "frac,seed,naive_est_err,n_naive" > "$2"
  for SEED in 101 102 103 104 105; do
    for F in 0.10 0.20 0.30 0.40 0.50 0.60; do
      sed -i "s/random_seed=\"[^\"]*\"/random_seed=\"$SEED\"/" $ARGOS
      sed -i "s/malicious_fraction=\"[^\"]*\"/malicious_fraction=\"$F\"/" $ARGOS
      argos3 -c $ARGOS >/dev/null 2>&1
      echo "$F,$SEED,$(tail -1 secureflock_log.csv | awk -F, '{print $8","$5}')" >> "$2"
    done
  done
}
run 1.00 data/sweep_aligned_full.csv       # full sensing (blind spot masked)
run 0.15 data/sweep_aligned_partial.csv    # partial sensing (blind spot exposed)
echo "DONE"
