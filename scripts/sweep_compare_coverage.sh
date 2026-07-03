#!/bin/bash
# Belief error vs sensing coverage at fixed attacker fraction (headline figure).
# Produces: data/sweep_compare_coverage.csv  -> plotting/plot_coverage.py
cd "$(dirname "$0")/.." || exit 1
ARGOS=experiments/secureflock_single.argos
sed -i 's/attacker="[^"]*"/attacker="follow"/' $ARGOS
sed -i 's/malicious_fraction="[^"]*"/malicious_fraction="0.30"/' $ARGOS
sed -i 's/ ground_fraction="[^"]*"//g' $ARGOS
sed -i 's/<secureflock /<secureflock ground_fraction="-1" /' $ARGOS
OUT=data/sweep_compare_coverage.csv
echo "method,ground_fraction,seed,naive_est_err,n_naive" > $OUT
for SEED in 101 102 103 104 105; do
  for COND in trust wmsr grounded; do
    case $COND in
      trust)    DEFv=true; METHv=trust; GRNDv=false;;
      wmsr)     DEFv=true; METHv=wmsr;  GRNDv=false;;
      grounded) DEFv=true; METHv=trust; GRNDv=true;;
    esac
    for GF in 0.0 0.2 0.4 0.6 0.8 1.0; do
      if [ "$COND" = "grounded" ]; then GFv=$GF; else GFv=-1; fi
      sed -i "s/random_seed=\"[^\"]*\"/random_seed=\"$SEED\"/" $ARGOS
      sed -i "s/defense=\"[^\"]*\"/defense=\"$DEFv\"/" $ARGOS
      sed -i "s/method=\"\(trust\|wmsr\)\"/method=\"$METHv\"/" $ARGOS
      sed -i "s/ground=\"[^\"]*\"/ground=\"$GRNDv\"/" $ARGOS
      sed -i "s/ground_fraction=\"[^\"]*\"/ground_fraction=\"$GFv\"/" $ARGOS
      argos3 -c $ARGOS >/dev/null 2>&1
      V=$(tail -1 secureflock_log.csv | awk -F, '{print $8","$5}')
      echo "$COND,$GF,$SEED,$V" >> $OUT
    done
  done
done
echo "DONE -> $OUT"
