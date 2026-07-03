#!/usr/bin/env python3
"""Two-panel parameter sensitivity (sigma; cone width x phantom bearing)."""
import pandas as pd, numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
s = pd.read_csv("data/sweep_sens_sigma.csv")
a = pd.read_csv("data/sweep_sens_agree.csv")
gs = s.groupby("sigma").agg(e=("naive_est_err","mean"),sd=("naive_est_err","std")).reset_index()
ga = a.groupby(["agree","phi_deg"]).agg(e=("naive_est_err","mean"),sd=("naive_est_err","std")).reset_index()
fig,(a1,a2) = plt.subplots(1,2,figsize=(11.5,4.6))
a1.errorbar(gs.sigma,gs.e,yerr=gs.sd,color="#1b7837",marker="o",lw=2,ms=6,capsize=3)
a1.set_xscale("log"); a1.set_xticks(gs.sigma); a1.set_xticklabels([f"{v:g}" for v in gs.sigma])
a1.set_xlabel(r"Trust tolerance $\sigma$"); a1.set_ylabel("Naive belief error (m)")
a1.set_ylim(-0.4,9); a1.grid(alpha=.25); a1.set_title("(A) Tolerance")
for ag,(c,mk) in {0.0:("#2166ac","o"),0.5:("#e08214","s")}.items():
    d=ga[ga.agree==ag].sort_values("phi_deg")
    a2.errorbar(d.phi_deg,d.e,yerr=d.sd,color=c,marker=mk,lw=2,ms=6,capsize=3,
                label=("90 deg cone" if ag==0.0 else "60 deg cone"))
a2.set_xlabel(r"Phantom bearing offset $\varphi$ (deg)"); a2.set_ylabel("Naive belief error (m)")
a2.set_ylim(-0.4,9); a2.grid(alpha=.25); a2.legend(); a2.set_title("(B) Cone width")
plt.tight_layout(); plt.savefig("sensitivity.png", dpi=150)
print("wrote sensitivity.png")
