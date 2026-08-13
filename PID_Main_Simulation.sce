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

initial_moisture = 30.0;
setpoint = 60.0;

dt = 0.01;
simulation_time = 60.0;

t = 0:dt:simulation_time;
n = length(t);

moisture = zeros(1,n);
motor_output = zeros(1,n);
control_signal = zeros(1,n);
error_signal = zeros(1,n);

moisture(1) = initial_moisture;

integral_error = 0.0;
previous_error = setpoint - moisture(1);

for k = 2:n

    error_signal(k) = setpoint - moisture(k-1);

    derivative_error = (error_signal(k) - previous_error) / dt;

    integral_error = integral_error + error_signal(k) * dt;

    integral_error = min(max(integral_error,-100),100);

    pid_output = Kp * error_signal(k) ...
               + Ki * integral_error ...
               + Kd * derivative_error;

    control_signal(k) = min(max(pid_output,0),255);

    motor_output(k) = motor_output(k-1) ...
        + dt * ((Km * control_signal(k)) - motor_output(k-1)) / tau_m;

    irrigation_input = KI * motor_output(k);

    evaporation_loss = KE * moisture(k-1);

    drainage_loss = KD * moisture(k-1);

    moisture_rate = irrigation_input ...
                  - evaporation_loss ...
                  - drainage_loss;

    moisture(k) = moisture(k-1) + moisture_rate * dt;

    moisture(k) = min(max(moisture(k),0),100);

    previous_error = error_signal(k);

end

error_signal(1) = setpoint - moisture(1);

final_moisture = moisture($);

steady_state_error = abs(setpoint - final_moisture);

maximum_moisture = max(moisture);

overshoot = max(0,((maximum_moisture - setpoint) / setpoint) * 100);

rise_target = initial_moisture + 0.90 * (setpoint - initial_moisture);

rise_index = find(moisture >= rise_target);

if size(rise_index,"*") > 0 then
    rise_time = t(rise_index(1));
else
    rise_time = %nan;
end

steady_start = max(1,n-100);

steady_state_mean = mean(moisture(steady_start:n));

steady_state_error_mean = abs(setpoint - steady_state_mean);

mprintf("\n=============================================\n");
mprintf(" AUTOMATIC PLANT WATERING - PID SIMULATION\n");
mprintf("=============================================\n");
mprintf("Initial Moisture       : %.2f %%\n",initial_moisture);
mprintf("Setpoint               : %.2f %%\n",setpoint);
mprintf("Kp                     : %.2f\n",Kp);
mprintf("Ki                     : %.2f\n",Ki);
mprintf("Kd                     : %.2f\n",Kd);
mprintf("Final Moisture         : %.4f %%\n",final_moisture);
mprintf("Steady-State Error     : %.6f %%\n",steady_state_error);
mprintf("Rise Time (90%%)        : %.4f s\n",rise_time);
mprintf("Overshoot              : %.4f %%\n",overshoot);
mprintf("Steady-State Mean      : %.4f %%\n",steady_state_mean);
mprintf("Mean Steady-State Err. : %.6f %%\n",steady_state_error_mean);
mprintf("=============================================\n");

figure(1);
plot(t,moisture);
plot(t,setpoint * ones(t));
xlabel("Time (s)");
ylabel("Soil Moisture (%)");
title("Soil Moisture Response with PID Control");
legend(["Soil Moisture","Setpoint"]);
xgrid();

figure(2);
plot(t,control_signal);
xlabel("Time (s)");
ylabel("PID Control Signal (PWM)");
title("PID Control Signal to Pump");
xgrid();

figure(3);
plot(t,error_signal);
xlabel("Time (s)");
ylabel("Error (%)");
title("Control Error");
xgrid();

results = [initial_moisture;
           setpoint;
           final_moisture;
           steady_state_error;
           rise_time;
           overshoot;
           steady_state_mean;
           steady_state_error_mean];

mprintf("\nSimulation completed.\n");