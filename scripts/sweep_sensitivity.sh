#!/bin/bash
# Parameter sensitivity (sigma, cone width c, phantom bearing phi) at a working
# operating point. Set GROUND_FRACTION to the coverage where grounding operates.
# Produces: data/sweep_sens_sigma.csv , data/sweep_sens_agree.csv
cd "$(dirname "$0")/.." || exit 1
ARGOS=experiments/secureflock_single.argos
PI=3.14159265
GROUND_FRACTION="${1:-0.80}"   # pass as arg; 0.80 = working point used in paper
sed -i 's/ground="[^"]*"/ground="true"/' $ARGOS
sed -i 's/defense="[^"]*"/defense="true"/' $ARGOS
sed -i 's/method="\(trust\|wmsr\)"/method="trust"/' $ARGOS
sed -i 's/attacker="[^"]*"/attacker="follow"/' $ARGOS
sed -i 's/malicious_fraction="[^"]*"/malicious_fraction="0.30"/' $ARGOS
sed -i 's/ ground_fraction="[^"]*"//g' $ARGOS
sed -i "s/<secureflock /<secureflock ground_fraction=\"$GROUND_FRACTION\" /" $ARGOS
# Part 1: tolerance sigma
OUT1=data/sweep_sens_sigma.csv
echo "sigma,seed,naive_est_err,n_naive" > $OUT1
sed -i 's/agree="[^"]*"/agree="0.0"/' $ARGOS
sed -i 's/false_x="[^"]*"/false_x="-4.0"/; s/false_y="[^"]*"/false_y="0.0"/' $ARGOS
for SEED in 101 102 103 104 105; do
  for SG in 0.5 1.0 2.0 3.0 5.0 8.0 12.0; do
    sed -i "s/random_seed=\"[^\"]*\"/random_seed=\"$SEED\"/" $ARGOS
    sed -i "s/tolerance=\"[^\"]*\"/tolerance=\"$SG\"/" $ARGOS
    argos3 -c $ARGOS >/dev/null 2>&1
    echo "$SG,$SEED,$(tail -1 secureflock_log.csv | awk -F, '{print $8","$5}')" >> $OUT1
  done
done
# Part 2: cone width x phantom bearing
OUT2=data/sweep_sens_agree.csv
echo "agree,phi_deg,seed,naive_est_err,n_naive" > $OUT2
sed -i 's/tolerance="[^"]*"/tolerance="3.0"/' $ARGOS
for SEED in 101 102 103 104 105; do
  for AG in 0.0 0.5; do
    for PHI in 20 40 60 80 100 140 180; do
      FX=$(awk "BEGIN{printf \"%.3f\", 4*cos($PHI*$PI/180)}")
      FY=$(awk "BEGIN{printf \"%.3f\", 4*sin($PHI*$PI/180)}")
      sed -i "s/random_seed=\"[^\"]*\"/random_seed=\"$SEED\"/" $ARGOS
      sed -i "s/agree=\"[^\"]*\"/agree=\"$AG\"/" $ARGOS
      sed -i "s/false_x=\"[^\"]*\"/false_x=\"$FX\"/; s/false_y=\"[^\"]*\"/false_y=\"$FY\"/" $ARGOS
      argos3 -c $ARGOS >/dev/null 2>&1
      echo "$AG,$PHI,$SEED,$(tail -1 secureflock_log.csv | awk -F, '{print $8","$5}')" >> $OUT2
    done
  done
done
echo "DONE -> $OUT1 , $OUT2"
