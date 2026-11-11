// pid_model.sci
// Shared model, controllers and metrics for
// "Automatic Plant Watering Based on Soil Moisture Sensor and PID Control"
// (Science, Engineering and Technology, 2026).
//
// Plant (Section 4.4): first-order pump/motor driving a first-order soil water balance
//   tau_m * dm/dt = Km * u(t) - m(t)                         (Eq. 3)
//   dtheta/dt     = KI * m(t) - KE * theta(t) - KD * theta(t) (Eq. 4)
// integrated with forward Euler (dt = 0.01 s, horizon 60 s).
//
// The simulation applies the saturated PID command u_sat(t) (0-255) directly as a
// continuous average drive. The firmware relay windowing (Eq. 6) and interlock are
// NOT modelled here; see arduino.ino for the firmware implementation.
//
// Load with: exec("pid_model.sci", -1);

// ---------------- Nominal parameters (Table 2) ----------------
APW_Km    = 1.20;    // pump/motor static gain
APW_tau_m = 2.50;    // pump/motor time constant (s)
APW_KI    = 0.080;   // irrigation (infiltration) coefficient (1/s)
APW_KE    = 0.015;   // evaporation coefficient (1/s)
APW_KD    = 0.005;   // drainage coefficient (1/s)
APW_dt    = 0.01;    // integration step (s)
APW_T     = 60.0;    // simulation horizon (s)
APW_ILIM  = 100;     // anti-windup limit on the integral state
APW_UMAX  = 255;     // 8-bit command saturation
APW_PUMP_ACTIVE = 10;     // command (/255) above which the simulated pump counts as active
APW_ONOFF_CMD   = 180;    // ON/OFF baseline command (/255)
APW_ONOFF_BAND  = 1.0;    // ON/OFF hysteresis half-band (pp)
APW_NOISE_FREQ  = 0.8;    // sensor disturbance frequency (Hz)

function t = apw_time()
    t = 0:APW_dt:APW_T;
endfunction

function [t, theta, u, n_switch] = apw_simulate_pid(Kp, Ki, Kd, theta0, sp, noise)
    // PID with anti-windup and 0-255 saturation; noise = amplitude (pp) of the
    // sinusoidal sensor disturbance added to the measurement (0 = undisturbed).
    t = apw_time();
    n = length(t);
    theta = zeros(1, n);
    u = zeros(1, n);
    theta(1) = theta0;
    m = 0;
    I = 0;
    e_prev = sp - theta(1);
    n_switch = 0;
    prev_active = %f;
    for k = 2:n
        meas = theta(k-1);
        if noise > 0 then
            meas = meas + noise * sin(2 * %pi * APW_NOISE_FREQ * t(k));
        end
        e = sp - meas;
        D = (e - e_prev) / APW_dt;
        I = min(max(I + e * APW_dt, -APW_ILIM), APW_ILIM);
        u(k) = min(max(Kp * e + Ki * I + Kd * D, 0), APW_UMAX);
        m = m + APW_dt * (APW_Km * u(k) - m) / APW_tau_m;
        dtheta = APW_KI * m - APW_KE * theta(k-1) - APW_KD * theta(k-1);
        theta(k) = min(max(theta(k-1) + dtheta * APW_dt, 0), 100);
        active = u(k) > APW_PUMP_ACTIVE;
        if active <> prev_active then
            n_switch = n_switch + 1;
        end
        prev_active = active;
        e_prev = e;
    end
endfunction

function [t, theta, u, n_switch] = apw_simulate_onoff(theta0, sp)
    // Hysteresis ON/OFF baseline: fully ON (180/255) at theta <= sp - 1, OFF at theta >= sp + 1.
    t = apw_time();
    n = length(t);
    theta = zeros(1, n);
    u = zeros(1, n);
    theta(1) = theta0;
    m = 0;
    state = 0;
    n_switch = 0;
    for k = 2:n
        if theta(k-1) <= sp - APW_ONOFF_BAND then
            new_state = 1;
        elseif theta(k-1) >= sp + APW_ONOFF_BAND then
            new_state = 0;
        else
            new_state = state;
        end
        if new_state <> state then
            n_switch = n_switch + 1;
        end
        state = new_state;
        u(k) = APW_ONOFF_CMD * state;
        m = m + APW_dt * (APW_Km * u(k) - m) / APW_tau_m;
        dtheta = APW_KI * m - APW_KE * theta(k-1) - APW_KD * theta(k-1);
        theta(k) = min(max(theta(k-1) + dtheta * APW_dt, 0), 100);
    end
endfunction

function r = apw_metrics(t, theta, theta0, sp)
    // Returns a struct with the metrics defined in Section 4.5:
    //   tr    rise time (s): first time theta reaches 90% of the step
    //   peak  peak value (%), tpeak its time (s)
    //   os    overshoot (% of setpoint)
    //   ts    settling time (s), +-2% of setpoint; %nan if not settled at 60 s
    //   final value at 60 s (%), e60 final error at 60 s (pp)
    //   iae   integral absolute error over 0-60 s (%.s), rectangle rule
    r = struct();
    idx = find(theta >= theta0 + 0.9 * (sp - theta0));
    if idx == [] then
        r.tr = %nan;
    else
        r.tr = t(idx(1));
    end
    [r.peak, ipk] = max(theta);
    r.tpeak = t(ipk);
    r.os = max(0, (r.peak - sp) / sp * 100);
    out = abs(theta - sp) > 0.02 * sp;
    if out($) then
        r.ts = %nan;
    else
        last = find(out);
        if last == [] then
            r.ts = 0;
        else
            r.ts = t(last($) + 1);
        end
    end
    r.final = theta($);
    r.e60 = abs(sp - r.final);
    r.iae = sum(abs(sp - theta)) * APW_dt;
endfunction

function s = apw_sigma_u(t, u, t_from)
    // Sample standard deviation of the command for t >= t_from.
    s = stdev(u(t >= t_from));
endfunction

function apw_write_series(path, header, cols, fmt)
    // Write a CSV with a header line; cols is an n-by-k matrix, fmt a printf row format.
    lines = msprintf(fmt + "\n", cols);
    mputl([header; lines], path);
endfunction

function d = apw_results_dir(script_dir)
    d = script_dir + "results" + filesep();
    if ~isdir(d) then
        mkdir(d);
    end
endfunction
