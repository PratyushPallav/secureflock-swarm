#!/usr/bin/env python3
"""Headline figure: belief error vs sensing coverage (three methods)."""
import pandas as pd, numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
d = pd.read_csv("data/sweep_compare_coverage.csv")
g = d.groupby(["method","ground_fraction"]).agg(
        e=("naive_est_err","mean"), s=("naive_est_err","std")).reset_index()
st = {"grounded":("#1b7837","o","-","Beacon-grounded (ours)"),
      "wmsr":("#2166ac","^","--","W-MSR (resilient consensus)"),
      "trust":("#b2182b","s",":","Consensus-trust")}
fig, ax = plt.subplots(figsize=(7.4,5.2))
for m in ["grounded","wmsr","trust"]:
    s = g[g.method==m].sort_values("ground_fraction"); c,mk,ls,lab = st[m]
    ax.errorbar(s.ground_fraction*100, s.e, yerr=s.s, color=c, marker=mk,
                ls=ls, lw=2.3, ms=7, capsize=3, label=lab)
ax.axhspan(0,1,color="#1b7837",alpha=0.08)
ax.set_xlabel("Naive robots that can sense the beacon (%)")
ax.set_ylabel("Naive belief error vs true goal (m)")
ax.set_xlim(0,100); ax.set_ylim(-0.4,9); ax.grid(alpha=0.25); ax.legend()
plt.tight_layout(); plt.savefig("coverage_comparison.png", dpi=150)
print("wrote coverage_comparison.png")
