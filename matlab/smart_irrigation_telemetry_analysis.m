%% SMART IRRIGATION CONTROL SYSTEM TELEMETRY & EFFICIENCY ANALYSIS
% Author: M. Deniz Sepit — Electrical & Electronics Engineering (deniz.sepit@std.ieu.tr)
clear; clc; close all;

T_hours = 72; dt = 0.25; t_hr = (0:dt:T_hours)'; N = length(t_hr);
rng(42);
temp_C = 30.0 + 7.5 * sin(2*pi*(t_hr - 8)/24) + 0.4*randn(N, 1);

moist_init = 68.0; thresh_low = 35.0; thresh_high = 75.0;
temp_lockout = 35.5; liters_per_pct = 0.18;

moist_closed = zeros(N,1); moist_timer = zeros(N,1);
moist_closed(1) = moist_init; moist_timer(1) = moist_init;
water_closed_L = zeros(N,1); water_timer_L = zeros(N,1);

for k = 1:(N-1)
    evap_rate = 0.48 + 0.038 * max(0, temp_C(k) - 18);
    loss_step = evap_rate * dt;

    % ArduKit Closed-Loop Hysteresis Controller
    m_c = moist_closed(k) - loss_step;
    if (m_c <= thresh_low) && (temp_C(k) <= temp_lockout)
        water_closed_L(k+1) = water_closed_L(k) + (thresh_high - m_c)*liters_per_pct;
        m_c = thresh_high;
    else
        water_closed_L(k+1) = water_closed_L(k);
    end
    moist_closed(k+1) = max(10, min(100, m_c));

    % Fixed 12-Hour Timer Schedule
    m_t = moist_timer(k) - loss_step;
    if mod(t_hr(k), 12) == 0 && t_hr(k) > 0
        water_timer_L(k+1) = water_timer_L(k) + 30.0 * liters_per_pct;
        m_t = min(98, m_t + 30.0);
    else
        water_timer_L(k+1) = water_timer_L(k);
    end
    moist_timer(k+1) = max(10, min(100, m_t));
end

water_saved_pct = ((water_timer_L(end) - water_closed_L(end)) / water_timer_L(end)) * 100;
fprintf('Net Water Consumption Reduction: %.1f %%
', water_saved_pct);

figure('Color','w');
subplot(2,1,1); plot(t_hr, moist_timer, '--r', t_hr, moist_closed, '-k', 'LineWidth', 1.8);
grid on; ylabel('Soil Moisture [%]'); title('ArduKit Closed-Loop vs. Fixed 12h Timer');
subplot(2,1,2); plot(t_hr, water_timer_L, '--r', t_hr, water_closed_L, '-k', 'LineWidth', 1.8);
grid on; xlabel('Time [Hours]'); ylabel('Cumulative Water [L]');
