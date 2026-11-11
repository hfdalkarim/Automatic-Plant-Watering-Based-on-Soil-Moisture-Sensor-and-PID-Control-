// PID_Main_Simulation.sce
// Baseline PID (Kp = 5.0, Ki = 0.8, Kd = 0.1), scenario S1 (30% -> 60%).
// Reproduces Table 4 and the values quoted in the text for Figures 4 and 5.
// Outputs: results/S1_baseline_timeseries.csv, results/Table4_S1_metrics.csv

here = get_absolute_file_path("PID_Main_Simulation.sce");
exec(here + "pid_model.sci", -1);
out_dir = apw_results_dir(here);

Kp = 5.0; Ki = 0.8; Kd = 0.1;
theta0 = 30; sp = 60;

[t, theta, u, n_sw] = apw_simulate_pid(Kp, Ki, Kd, theta0, sp, 0);
r = apw_metrics(t, theta, theta0, sp);

// Command statistics (Figure 5)
[u_max, i_max] = max(u);
i_zero = find(u == 0 & t > 1);
after_peak = find(t > r.tpeak);
[th_min, i_min] = min(theta(after_peak));
u_ss = (APW_KE + APW_KD) * sp / (APW_KI * APW_Km);   // Eq. (5)

mprintf("=== Scenario S1: 30%% -> 60%%, Kp = %.1f, Ki = %.1f, Kd = %.1f ===\n", Kp, Ki, Kd);
mprintf("Rise time t_r            : %.2f s\n", r.tr);
mprintf("Peak value               : %.2f %% at %.2f s\n", r.peak, r.tpeak);
mprintf("Overshoot                : %.2f %%\n", r.os);
mprintf("Settling time t_s (2%%)   : %.2f s\n", r.ts);
mprintf("Final moisture at 60 s   : %.2f %%\n", r.final);
mprintf("Final error at 60 s e60  : %.2f pp\n", r.e60);
mprintf("IAE                      : %.1f %%.s\n", r.iae);
mprintf("Pump switching events    : %d\n", n_sw);
mprintf("Undershoot after peak    : %.2f %% at %.2f s\n", th_min, t(after_peak(i_min)));
mprintf("--- Continuous command u_sat(t) (Figure 5) ---\n");
mprintf("Initial command          : %.1f/255 (%.1f %% duty)\n", u(2), u(2) / 255 * 100);
mprintf("Maximum command          : %.1f/255 at %.2f s\n", u_max, t(i_max));
mprintf("Clipped to zero          : %.2f s to %.2f s\n", t(i_zero(1)), t(i_zero($)));
mprintf("Mean command, last 10 s  : %.1f/255\n", mean(u(t > 50)));
mprintf("Analytical steady state  : %.1f/255 (%.1f %% duty), Eq. (5)\n", u_ss, u_ss / 255 * 100);

apw_write_series(out_dir + "S1_baseline_timeseries.csv", "t_s,theta_pct,u_sat", ..
    [t' theta' u'], "%.2f,%.6f,%.6f");
mputl(["metric,value,unit";
       msprintf("rise_time,%.2f,s", r.tr);
       msprintf("peak,%.2f,%%", r.peak);
       msprintf("peak_time,%.2f,s", r.tpeak);
       msprintf("overshoot,%.2f,%% of setpoint", r.os);
       msprintf("settling_time_2pct,%.2f,s", r.ts);
       msprintf("final_moisture_60s,%.2f,%%", r.final);
       msprintf("final_error_60s,%.2f,pp", r.e60);
       msprintf("IAE,%.1f,%%.s", r.iae);
       msprintf("pump_switching_events,%d,-", n_sw)], out_dir + "Table4_S1_metrics.csv");
mprintf("Wrote results/S1_baseline_timeseries.csv\n");
mprintf("Wrote results/Table4_S1_metrics.csv\n");
