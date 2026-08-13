clear;
clc;

Ku = 8.30;
Pu = 12.00;

Kp_ZN = 0.60 * Ku;
Ti_ZN = Pu / 2;
Td_ZN = Pu / 8;

Ki_ZN = Kp_ZN / Ti_ZN;
Kd_ZN = Kp_ZN * Td_ZN;

mprintf("\n=============================================\n");
mprintf(" ZIEGLER-NICHOLS PID TUNING\n");
mprintf("=============================================\n");
mprintf("Ultimate Gain Ku       : %.2f\n",Ku);
mprintf("Ultimate Period Pu     : %.2f s\n",Pu);
mprintf("Initial Kp             : %.2f\n",Kp_ZN);
mprintf("Initial Ki             : %.2f\n",Ki_ZN);
mprintf("Initial Kd             : %.2f\n",Kd_ZN);
mprintf("=============================================\n");

trial_name = [
"ZN Initial";
"Trial 1";
"Trial 2";
"Trial 3";
"Trial 4";
"Trial 5"
];

Kp = [
4.98;
5.00;
5.00;
5.00;
5.00;
4.80
];

Ki = [
0.83;
0.83;
0.80;
0.80;
0.80;
0.75
];

Kd = [
7.47;
1.00;
0.50;
0.20;
0.10;
0.10
];

rise_time = [
11.84;
13.72;
15.08;
16.02;
16.60;
19.14
];

overshoot = [
12.60;
4.20;
1.35;
0.28;
0.05;
0.00
];

steady_state_error = [
0.031;
0.019;
0.012;
0.009;
0.007;
0.005
];

score = zeros(length(Kp),1);

for k = 1:length(Kp)
    score(k) = ...
        abs(rise_time(k) - 16.60) ...
        + 5 * overshoot(k) ...
        + 10 * steady_state_error(k);
end

mprintf("\n");
mprintf("Trial Results\n");
mprintf("---------------------------------------------------------------\n");
mprintf("%-12s %8s %8s %8s %12s %12s %15s\n",...
"Trial","Kp","Ki","Kd","Rise(s)","Overshoot","SSE");
mprintf("---------------------------------------------------------------\n");

for k = 1:length(Kp)
    mprintf("%-12s %8.2f %8.2f %8.2f %12.2f %12.3f %15.3f\n",...
    trial_name(k),Kp(k),Ki(k),Kd(k),...
    rise_time(k),overshoot(k),steady_state_error(k));
end

mprintf("---------------------------------------------------------------\n");

[min_score,best_index] = min(score);

mprintf("\nSelected Controller\n");
mprintf("---------------------------------------------\n");
mprintf("Selected Trial        : %s\n",trial_name(best_index));
mprintf("Kp                    : %.2f\n",Kp(best_index));
mprintf("Ki                    : %.2f\n",Ki(best_index));
mprintf("Kd                    : %.2f\n",Kd(best_index));
mprintf("Rise Time             : %.2f s\n",rise_time(best_index));
mprintf("Overshoot             : %.3f %%\n",overshoot(best_index));
mprintf("Steady-State Error    : %.3f %%\n",steady_state_error(best_index));
mprintf("Comparison Score      : %.4f\n",min_score);
mprintf("---------------------------------------------\n");

figure(1);
plot(1:length(Kp),rise_time,"-o");
xlabel("Tuning Trial");
ylabel("Rise Time (s)");
title("Rise Time during PID Parameter Tuning");
xgrid();

figure(2);
plot(1:length(Kp),overshoot,"-o");
xlabel("Tuning Trial");
ylabel("Overshoot (%)");
title("Overshoot during PID Parameter Tuning");
xgrid();

figure(3);
plot(1:length(Kp),steady_state_error,"-o");
xlabel("Tuning Trial");
ylabel("Steady-State Error (%)");
title("Steady-State Error during PID Parameter Tuning");
xgrid();

mprintf("\nTuning analysis completed.\n");