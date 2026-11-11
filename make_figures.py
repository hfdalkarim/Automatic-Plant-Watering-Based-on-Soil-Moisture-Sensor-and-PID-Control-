"""Figure-generation script: redraws Figures 4-7 from the CSV files written by the Scilab scripts.

Run the three Scilab scripts first (they write results/*.csv), then:
    python make_figures.py
Requires Python 3 with numpy and matplotlib. Figures are saved to results/figures/.
"""
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
RES = HERE / "results"
OUT = RES / "figures"
OUT.mkdir(parents=True, exist_ok=True)

KE, KD, KI, KM = 0.015, 0.005, 0.080, 1.20
SP = 60.0


def load(name):
    return np.genfromtxt(RES / name, delimiter=",", names=True)


def save(fig, name):
    fig.tight_layout()
    fig.savefig(OUT / name, dpi=300)
    plt.close(fig)
    print("Wrote", (OUT / name).relative_to(HERE))


# Figure 4: soil moisture, S1 baseline
s1 = load("S1_baseline_timeseries.csv")
t, th, u = s1["t_s"], s1["theta_pct"], s1["u_sat"]
ipk = int(np.argmax(th))
tr = t[np.argmax(th >= 30 + 0.9 * (SP - 30))]
out = np.abs(th - SP) > 0.02 * SP
ts = t[np.where(out)[0][-1] + 1]
fig, ax = plt.subplots(figsize=(6.0, 3.6))
ax.axhspan(0.98 * SP, 1.02 * SP, color="0.85", label="±2% settling band")
ax.axhline(SP, ls="--", color="0.3", lw=1, label="Setpoint (60%)")
ax.plot(t, th, lw=1.8, label="Soil moisture, PID")
ax.annotate(f"Peak {th[ipk]:.2f}% at {t[ipk]:.2f} s\n(overshoot {(th[ipk] - SP) / SP * 100:.2f}% of setpoint)",
            xy=(t[ipk], th[ipk]), xytext=(t[ipk] + 6, th[ipk] + 1), arrowprops=dict(arrowstyle="->"))
ax.axvline(tr, ls=":", color="0.4"); ax.text(tr + 0.5, 32, f"$t_r$ = {tr:.2f} s")
ax.axvline(ts, ls=":", color="0.4"); ax.text(ts + 0.5, 72, f"$t_s$ = {ts:.2f} s")
ax.set(xlabel="Time (s)", ylabel="Soil moisture (%)", xlim=(0, 60), ylim=(25, 85))
ax.legend(loc="lower right", fontsize=8)
save(fig, "Figure4_S1_moisture.png")

# Figure 5: continuous PID command, S1 baseline
u_ss = (KE + KD) * SP / (KI * KM)
fig, ax = plt.subplots(figsize=(6.0, 3.4))
ax.plot(t, u, lw=1.8)
ax.axhline(u_ss, ls="--", color="0.4", lw=1)
ax.annotate(f"Initial command {u[1]:.1f}", xy=(t[1], u[1]), xytext=(8, 175), arrowprops=dict(arrowstyle="->"))
ax.annotate(f"Analytical steady-state command\n≈ {u_ss:.1f} ({u_ss / 255 * 100:.1f}% duty)", xy=(52, u_ss),
            xytext=(33, 60), arrowprops=dict(arrowstyle="->"))
ax.annotate("Clipped at 0\n(negative PID output)", xy=(20, 0), xytext=(14, 35), arrowprops=dict(arrowstyle="->"))
ax.set(xlabel="Time (s)", ylabel="PID duty-cycle command (0–255)", xlim=(0, 60), ylim=(-5, 200))
save(fig, "Figure5_S1_command.png")

# Figure 6: multi-scenario validation
sc = load("Fig6_scenarios_timeseries.csv")
fig, (a, b) = plt.subplots(2, 1, figsize=(6.0, 6.2), sharex=True)
for col, lab, sp, ls in [("theta_S1", "S1: 30% → 60%", 60, "-"), ("theta_S2", "S2: 20% → 50%", 50, "--"),
                         ("theta_S3", "S3: 40% → 70%", 70, "-.")]:
    line, = a.plot(sc["t_s"], sc[col], ls=ls, lw=1.8, label=lab)
    a.axhline(sp, ls=":", lw=1, color=line.get_color())
a.set(ylabel="Soil moisture (%)", ylim=(15, 95)); a.legend(fontsize=8); a.text(1, 88, "(a)", weight="bold")
b.plot(sc["t_s"], sc["u_S4"], lw=1.0, label="S4: with ±1% sensor disturbance")
b.plot(sc["t_s"], sc["u_S1"], lw=1.8, label="S1: noise-free")
b.set(xlabel="Time (s)", ylabel="PID duty-cycle command (0–255)", xlim=(0, 60), ylim=(-5, 200))
b.legend(fontsize=8); b.text(1, 185, "(b)", weight="bold")
save(fig, "Figure6_scenarios.png")

# Figure 7: PID versus hysteresis ON/OFF
cmp_ = load("Fig7_PID_vs_ONOFF_timeseries.csv")
fig, (a, b) = plt.subplots(2, 1, figsize=(6.0, 6.2), sharex=True)
a.axhspan(SP - 1, SP + 1, color="0.88", label="ON/OFF hysteresis band (±1 pp)")
a.axhline(SP, ls="--", color="0.3", lw=1, label="Setpoint (60%)")
a.plot(cmp_["t_s"], cmp_["theta_ONOFF"], ls="--", lw=1.6, label="ON/OFF")
a.plot(cmp_["t_s"], cmp_["theta_PID"], lw=1.8, label="PID")
a.set(ylabel="Soil moisture (%)", ylim=(25, 90)); a.legend(fontsize=8, loc="lower right")
a.text(1, 85, "(a)", weight="bold")
b.plot(cmp_["t_s"], cmp_["u_ONOFF"], ls="--", lw=1.4, label="ON/OFF (fixed 180/255)")
b.plot(cmp_["t_s"], cmp_["u_PID"], lw=1.8, label="PID")
b.set(xlabel="Time (s)", ylabel="Pump command (0–255)", xlim=(0, 60), ylim=(-5, 200))
b.legend(fontsize=8, loc="upper center"); b.text(1, 185, "(b)", weight="bold")
save(fig, "Figure7_PID_vs_ONOFF.png")
