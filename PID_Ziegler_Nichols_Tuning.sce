// PID_Ziegler_Nichols_Tuning.sce
// Ziegler-Nichols (ZN) starting point and trial-and-error refinement, scenario S1.
// Reproduces Table 3. Switching events and sigma_u (t >= 45 s) use the +-1% sensor
// disturbance case; all other metrics use the undisturbed case.
// Output: results/Table3_gain_candidates.csv

here = get_absolute_file_path("PID_Ziegler_Nichols_Tuning.sce");
exec(here + "pid_model.sci", -1);
out_dir = apw_results_dir(here);

// Classical ZN closed-loop rules with an assumed ultimate gain and period
Ku = 8.30; Pu = 12.0;
Kp_zn = 0.6 * Ku;
Ti = Pu / 2; Td = Pu / 8;
Ki_zn = Kp_zn / Ti;
Kd_zn = Kp_zn * Td;
mprintf("ZN: Ku = %.2f, Pu = %.1f s -> Kp = %.2f, Ki = %.2f 1/s, Kd = %.2f s\n\n", Ku, Pu, Kp_zn, Ki_zn, Kd_zn);

names = ["ZN start"; "Trial 1"; "Trial 2"; "Trial 3"; "Trial 4 (baseline)"; "Trial 5"; "Trial 6 (low Ki)"];
gains = [round(Kp_zn * 100) / 100, round(Ki_zn * 100) / 100, round(Kd_zn * 100) / 100;
         5.00, 0.83, 1.00;
         5.00, 0.80, 0.50;
         5.00, 0.80, 0.20;
         5.00, 0.80, 0.10;
         4.80, 0.75, 0.10;
         2.00, 0.10, 0.10];
theta0 = 30; sp = 60; noise = 1.0;

rows = ["candidate,Kp,Ki,Kd,tr_s,OS_pct,ts_s,e60_pp,IAE_pct_s,switch_noise,sigma_u_noise"];
mprintf("%-20s %6s %6s %6s %7s %7s %7s %7s %8s %7s %8s\n", "Candidate", "Kp", "Ki", "Kd", ..
        "tr(s)", "OS(%)", "ts(s)", "e60", "IAE", "Switch", "sigma_u");
for i = 1:size(gains, 1)
    [t, theta, u, sw0] = apw_simulate_pid(gains(i,1), gains(i,2), gains(i,3), theta0, sp, 0);
    r = apw_metrics(t, theta, theta0, sp);
    [tn, thn, un, swn] = apw_simulate_pid(gains(i,1), gains(i,2), gains(i,3), theta0, sp, noise);
    su = apw_sigma_u(tn, un, 45);
    mprintf("%-20s %6.2f %6.2f %6.2f %7.2f %7.2f %7.2f %7.2f %8.1f %7d %8.2f\n", names(i), ..
            gains(i,1), gains(i,2), gains(i,3), r.tr, r.os, r.ts, r.e60, r.iae, swn, su);
    rows = [rows; msprintf("%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%d,%.2f", names(i), ..
            gains(i,1), gains(i,2), gains(i,3), r.tr, r.os, r.ts, r.e60, r.iae, swn, su)];
end
mputl(rows, out_dir + "Table3_gain_candidates.csv");
mprintf("\nWrote results/Table3_gain_candidates.csv\n");
