#!/usr/bin/env python3
"""Same-bearing blind spot: full vs partial sensing across attacker fraction."""
import pandas as pd, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
def agg(fn): 
    d=pd.read_csv(fn); return d.groupby("frac").agg(e=("naive_est_err","mean"),s=("naive_est_err","std")).reset_index()
gf, gp = agg("data/sweep_aligned_full.csv"), agg("data/sweep_aligned_partial.csv")
fig,ax=plt.subplots(figsize=(6.6,4.6))
ax.errorbar(gf.frac,gf.e,yerr=gf.s,color="#1b7837",marker="o",lw=2,ms=6,capsize=3,label="Full sensing")
ax.errorbar(gp.frac,gp.e,yerr=gp.s,color="#b2182b",marker="s",lw=2,ms=6,capsize=3,label="Partial sensing (15%)")
ax.set_xlabel("Embedded attacker fraction"); ax.set_ylabel("Naive belief error (m)")
ax.set_ylim(-0.15,3.1); ax.grid(alpha=.25); ax.legend()
plt.tight_layout(); plt.savefig("limitation_bearing.png", dpi=150)
print("wrote limitation_bearing.png")
