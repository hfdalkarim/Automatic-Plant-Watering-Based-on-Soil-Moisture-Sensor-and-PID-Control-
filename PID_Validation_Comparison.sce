// PID_Validation_Comparison.sce
// Multi-scenario validation of the baseline PID (Table 5, Figure 6) and comparison with
// the hysteresis ON/OFF controller under identical conditions (Table 6, Figure 7).
// Outputs: results/Table5_scenarios.csv, results/Table6_PID_vs_ONOFF.csv,
//          results/Fig6_scenarios_timeseries.csv, results/Fig7_PID_vs_ONOFF_timeseries.csv

here = get_absolute_file_path("PID_Validation_Comparison.sce");
exec(here + "pid_model.sci", -1);
out_dir = apw_results_dir(here);

Kp = 5.0; Ki = 0.8; Kd = 0.1;
names  = ["S1: 30% -> 60%"; "S2: 20% -> 50%"; "S3: 40% -> 70%"; "S4: S1 + 1% sensor disturbance"];
theta0 = [30; 20; 40; 30];
sp     = [60; 50; 70; 60];
noise  = [0; 0; 0; 1.0];

mprintf("=== Table 5: baseline PID in four scenarios ===\n");
mprintf("%-32s %7s %7s %7s %7s %8s %7s\n", "Scenario", "tr(s)", "OS(%)", "ts(s)", "e60", "IAE", "Switch");
rows = ["scenario,theta0_pct,setpoint_pct,noise_pp,tr_s,OS_pct,ts_s,e60_pp,IAE_pct_s,switch"];
TH = []; U = [];
for i = 1:4
    [t, theta, u, sw] = apw_simulate_pid(Kp, Ki, Kd, theta0(i), sp(i), noise(i));
    r = apw_metrics(t, theta, theta0(i), sp(i));
    mprintf("%-32s %7.2f %7.2f %7.2f %7.2f %8.1f %7d\n", names(i), r.tr, r.os, r.ts, r.e60, r.iae, sw);
    rows = [rows; msprintf("%s,%g,%g,%g,%.2f,%.2f,%.2f,%.2f,%.1f,%d", names(i), theta0(i), sp(i), ..
            noise(i), r.tr, r.os, r.ts, r.e60, r.iae, sw)];
    TH = [TH theta']; U = [U u'];
end
mputl(rows, out_dir + "Table5_scenarios.csv");
apw_write_series(out_dir + "Fig6_scenarios_timeseries.csv", ..
    "t_s,theta_S1,theta_S2,theta_S3,theta_S4,u_S1,u_S2,u_S3,u_S4", [t' TH U], ..
    "%.2f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f");

// ---- Table 6: PID versus hysteresis ON/OFF (scenario S1) ----
[t, th_p, u_p, sw_p] = apw_simulate_pid(Kp, Ki, Kd, 30, 60, 0);
rp = apw_metrics(t, th_p, 30, 60);
[t, th_o, u_o, sw_o] = apw_simulate_onoff(30, 60);
ro = apw_metrics(t, th_o, 30, 60);
cyc = th_o(t >= 30);

mprintf("\n=== Table 6: PID versus hysteresis ON/OFF (S1) ===\n");
mprintf("%-30s %10s %10s\n", "Metric", "PID", "ON/OFF");
mprintf("%-30s %10.2f %10.2f\n", "Rise time t_r (s)", rp.tr, ro.tr);
mprintf("%-30s %10.2f %10.2f\n", "Peak soil moisture (%)", rp.peak, ro.peak);
mprintf("%-30s %10.2f %10.2f\n", "Overshoot (% of setpoint)", rp.os, ro.os);
if isnan(ro.ts) then
    mprintf("%-30s %10.2f %s\n", "Settling time t_s, 2% (s)", rp.ts, ..
            msprintf("Not reached (limit cycle %.2f-%.2f%%)", min(cyc), max(cyc)));
else
    mprintf("%-30s %10.2f %10.2f\n", "Settling time t_s, 2% (s)", rp.ts, ro.ts);
end
mprintf("%-30s %10.2f %10.2f\n", "Final moisture at 60 s (%)", rp.final, ro.final);
mprintf("%-30s %10.2f %10.2f\n", "Final error at 60 s (pp)", rp.e60, ro.e60);
mprintf("%-30s %10.1f %10.1f\n", "IAE (%.s)", rp.iae, ro.iae);
mprintf("%-30s %10d %10d\n", "Pump switching events", sw_p, sw_o);
mprintf("ON/OFF limit cycle for t >= 30 s: %.2f-%.2f %% (mean %.2f %%)\n", min(cyc), max(cyc), mean(cyc));
mprintf("PID vs ON/OFF: overshoot -%.1f %%, IAE -%.1f %%, rise time +%.1f %%\n", ..
        (ro.os - rp.os) / ro.os * 100, (ro.iae - rp.iae) / ro.iae * 100, (rp.tr - ro.tr) / ro.tr * 100);

mputl(["metric,PID,ONOFF";
       msprintf("rise_time_s,%.2f,%.2f", rp.tr, ro.tr);
       msprintf("peak_pct,%.2f,%.2f", rp.peak, ro.peak);
       msprintf("overshoot_pct,%.2f,%.2f", rp.os, ro.os);
       msprintf("settling_time_s,%.2f,%s", rp.ts, string(ro.ts));
       msprintf("final_moisture_60s_pct,%.2f,%.2f", rp.final, ro.final);
       msprintf("final_error_60s_pp,%.2f,%.2f", rp.e60, ro.e60);
       msprintf("IAE_pct_s,%.1f,%.1f", rp.iae, ro.iae);
       msprintf("pump_switching_events,%d,%d", sw_p, sw_o);
       msprintf("onoff_limit_cycle_min_pct,,%.2f", min(cyc));
       msprintf("onoff_limit_cycle_max_pct,,%.2f", max(cyc));
       msprintf("onoff_limit_cycle_mean_pct,,%.2f", mean(cyc))], out_dir + "Table6_PID_vs_ONOFF.csv");
apw_write_series(out_dir + "Fig7_PID_vs_ONOFF_timeseries.csv", "t_s,theta_PID,theta_ONOFF,u_PID,u_ONOFF", ..
    [t' th_p' th_o' u_p' u_o'], "%.2f,%.6f,%.6f,%.6f,%.6f");
mprintf("\nWrote results/Table5_scenarios.csv, results/Table6_PID_vs_ONOFF.csv, results/Fig6_scenarios_timeseries.csv, results/Fig7_PID_vs_ONOFF_timeseries.csv\n");
