clear;
clc;
clf;

Kp = 5.0;
Ki = 0.8;
Kd = 0.1;

Km = 1.20;
tau_m = 2.50;

KI = 0.080;
KE = 0.015;
KD = 0.005;

dt = 0.01;
simulation_time = 60.0;

function [t,moisture,control_signal,switch_count] = simulate_pid(initial_moisture,setpoint,noise_level)

    t = 0:dt:simulation_time;
    n = length(t);

    moisture = zeros(1,n);
    control_signal = zeros(1,n);

    moisture(1) = initial_moisture;

    motor_output = 0.0;
    integral_error = 0.0;
    previous_error = setpoint - moisture(1);

    switch_count = 0;
    previous_state = 0;

    for k = 2:n

        measured_moisture = moisture(k-1);

        if noise_level > 0 then
            measured_moisture = measured_moisture ...
                + noise_level * sin(2 * %pi * 0.8 * t(k));
        end

        error_signal = setpoint - measured_moisture;

        derivative_error = (error_signal - previous_error) / dt;

        integral_error = integral_error + error_signal * dt;

        integral_error = min(max(integral_error,-100),100);

        pid_output = Kp * error_signal ...
                   + Ki * integral_error ...
                   + Kd * derivative_error;

        control_signal(k) = min(max(pid_output,0),255);

        motor_output = motor_output ...
            + dt * ((Km * control_signal(k)) - motor_output) / tau_m;

        irrigation_input = KI * motor_output;

        evaporation_loss = KE * moisture(k-1);

        drainage_loss = KD * moisture(k-1);

        moisture_rate = irrigation_input ...
                      - evaporation_loss ...
                      - drainage_loss;

        moisture(k) = moisture(k-1) + moisture_rate * dt;

        moisture(k) = min(max(moisture(k),0),100);

        current_state = control_signal(k) > 10;

        if current_state <> previous_state then
            switch_count = switch_count + 1;
        end

        previous_state = current_state;
        previous_error = error_signal;

    end
endfunction

function [t,moisture,switch_count] = simulate_onoff(initial_moisture,setpoint)

    t = 0:dt:simulation_time;
    n = length(t);

    moisture = zeros(1,n);

    moisture(1) = initial_moisture;

    pump_state = 0;
    switch_count = 0;

    upper_threshold = setpoint + 1.0;
    lower_threshold = setpoint - 1.0;

    motor_output = 0.0;

    for k = 2:n

        if moisture(k-1) <= lower_threshold then
            new_state = 1;
        elseif moisture(k-1) >= upper_threshold then
            new_state = 0;
        else
            new_state = pump_state;
        end

        if new_state <> pump_state then
            switch_count = switch_count + 1;
        end

        pump_state = new_state;

        if pump_state == 1 then
            motor_command = 180;
        else
            motor_command = 0;
        end

        motor_output = motor_output ...
            + dt * ((Km * motor_command) - motor_output) / tau_m;

        irrigation_input = KI * motor_output;

        evaporation_loss = KE * moisture(k-1);

        drainage_loss = KD * moisture(k-1);

        moisture_rate = irrigation_input ...
                      - evaporation_loss ...
                      - drainage_loss;

        moisture(k) = moisture(k-1) + moisture_rate * dt;

        moisture(k) = min(max(moisture(k),0),100);

    end
endfunction

function [rise_time,overshoot,sse,final_value] = calculate_metrics(t,moisture,initial_moisture,setpoint)

    final_value = moisture($);

    rise_target = initial_moisture ...
        + 0.90 * (setpoint - initial_moisture);

    rise_index = find(moisture >= rise_target);

    if size(rise_index,"*") > 0 then
        rise_time = t(rise_index(1));
    else
        rise_time = %nan;
    end

    maximum_value = max(moisture);

    overshoot = max(0,((maximum_value - setpoint) / setpoint) * 100);

    sse = abs(setpoint - final_value);

endfunction

scenario_initial = [
30;
20;
40;
30
];

scenario_setpoint = [
60;
50;
70;
60
];

scenario_noise = [
0;
0;
0;
1.0
];

scenario_name = [
"30 to 60";
"20 to 50";
"40 to 70";
"30 to 60 + Noise"
];

pid_rise = zeros(4,1);
pid_overshoot = zeros(4,1);
pid_sse = zeros(4,1);
pid_final = zeros(4,1);
pid_switch = zeros(4,1);

for s = 1:4

    [t,pid_moisture,pid_control,pid_switch_count] = ...
        simulate_pid(...
        scenario_initial(s),...
        scenario_setpoint(s),...
        scenario_noise(s));

    [pid_rise(s),...
     pid_overshoot(s),...
     pid_sse(s),...
     pid_final(s)] = ...
        calculate_metrics(...
        t,...
        pid_moisture,...
        scenario_initial(s),...
        scenario_setpoint(s));

    pid_switch(s) = pid_switch_count;

end

[onoff_t,onoff_moisture,onoff_switch_count] = ...
    simulate_onoff(30,60);

[onoff_rise,...
 onoff_overshoot,...
 onoff_sse,...
 onoff_final] = ...
    calculate_metrics(...
    onoff_t,...
    onoff_moisture,...
    30,...
    60);

mprintf("\n==============================================================\n");
mprintf(" PID MULTI-SCENARIO VALIDATION\n");
mprintf("==============================================================\n");
mprintf("%-20s %10s %12s %12s %12s %12s\n",...
"Scenario","Rise(s)","Overshoot","SSE","Final(%)","Switch");
mprintf("--------------------------------------------------------------\n");

for s = 1:4

    mprintf("%-20s %10.2f %12.3f %12.4f %12.3f %12.0f\n",...
    scenario_name(s),...
    pid_rise(s),...
    pid_overshoot(s),...
    pid_sse(s),...
    pid_final(s),...
    pid_switch(s));

end

mprintf("==============================================================\n");

mprintf("\nPID versus ON/OFF - Identical Condition\n");
mprintf("--------------------------------------------------------------\n");
mprintf("%-25s %12s %12s\n","Performance","PID","ON/OFF");
mprintf("--------------------------------------------------------------\n");
mprintf("%-25s %12.2f %12.2f\n","Rise Time (s)",pid_rise(1),onoff_rise);
mprintf("%-25s %12.3f %12.3f\n","Overshoot (%)",pid_overshoot(1),onoff_overshoot);
mprintf("%-25s %12.4f %12.4f\n","Steady-State Error (%)",pid_sse(1),onoff_sse);
mprintf("%-25s %12.3f %12.3f\n","Final Moisture (%)",pid_final(1),onoff_final);
mprintf("%-25s %12.0f %12.0f\n","Switching Cycles",pid_switch(1),onoff_switch_count);
mprintf("--------------------------------------------------------------\n");

overshoot_reduction = ...
    ((onoff_overshoot - pid_overshoot(1)) ...
    / onoff_overshoot) * 100;

rise_time_reduction = ...
    ((onoff_rise - pid_rise(1)) ...
    / onoff_rise) * 100;

mprintf("Overshoot Reduction : %.2f %%\n",overshoot_reduction);
mprintf("Rise-Time Reduction : %.2f %%\n",rise_time_reduction);

figure(1);
plot(t,0);
clf();

[t1,m1,~,~] = simulate_pid(30,60,0);
[t2,m2,~,~] = simulate_pid(20,50,0);
[t3,m3,~,~] = simulate_pid(40,70,0);
[t4,m4,~,~] = simulate_pid(30,60,1.0);

plot(t1,m1);
plot(t2,m2);
plot(t3,m3);
plot(t4,m4);

xlabel("Time (s)");
ylabel("Soil Moisture (%)");
title("PID Response under Multiple Operating Conditions");
legend([
"30% to 60%";
"20% to 50%";
"40% to 70%";
"30% to 60% with Noise"
]);
xgrid();

figure(2);
plot(onoff_t,onoff_moisture);
plot(t1,m1);

xlabel("Time (s)");
ylabel("Soil Moisture (%)");
title("PID versus ON/OFF Control");
legend([
"ON/OFF";
"PID"
]);
xgrid();

figure(3);

comparison_values = [
pid_rise(1) onoff_rise;
pid_overshoot(1) onoff_overshoot;
pid_sse(1) onoff_sse;
pid_switch(1) onoff_switch_count
];

bar(comparison_values);

xlabel("Performance Metric");
ylabel("Value");
title("Quantitative PID versus ON/OFF Comparison");
xgrid();

mprintf("\nValidation and comparison completed.\n");