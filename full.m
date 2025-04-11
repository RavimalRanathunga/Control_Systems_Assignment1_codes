%% MATLAB code for temperature distribution when θset = 20oC

% Parameters
theta0 = 31;         % Initial temperature (°C)
theta_outd = 32;     % Outdoor temperature (°C)
Q_out = 24000/(60.*0.94787);         % Heat loss (W)
P = (2314.3.*60)/1000;             % Power input (W)
U = 0.13;              % Overall heat transfer coefficient (W/m^2K)
A = 242;               % Surface area (m^2)
m = 10.*8.*4.5.*1.18;              % Mass (kg)
c = 1;            % Specific heat capacity (kJ/kg·K), converted to W·s/kg·K

% Calculated constants
k = U * A;                               % Heat transfer rate
k1 = theta_outd - Q_out/k + P/k;        % Steady state term

% Time vector
t = linspace(0, 100, 100);             % Time in minutes

% Temperature as a function of time
theta = (theta0 - k1) .* exp(-((k .* t) ./ (m * c))) + k1;

% Plot
plot(t, theta, 'LineWidth', 2)
xlabel('Time (min)')
ylabel('\theta(t) (°C)')
title('Temperature vs Time')
grid on



%% Temperature Variation for Different Kp Values

% Parameters
theta_outd = 32;         
theta_o = 31;            
theta_set = 26;          
U = 0.13;                
A = 242;                 

% Updated P and mc values
P = 2.3143 * 60;       
mc_value = 10 * 8 * 4.5 * 1.18; 

k = U * A;              

% Range of Kp values
Kp_values = 200:200:1000;

% Time span for the simulation
tspan = [0, 300]; 

% Initialize figure
figure;

% Loop through each Kp value
for Kp_value = Kp_values
    alpha = k / mc_value;          

    % Define the differential equation function
    dtheta_dt = @(t, theta) -alpha * (theta - (theta_outd - (Kp_value * (theta - theta_set)) / k + P / k));

    % Initial condition
    initial_theta = theta_o;

    % Solve the differential equation using ode45
    [t, theta] = ode45(dtheta_dt, tspan, initial_theta);

    % Plot the results for each Kp
    plot(t, theta, 'DisplayName', ['Kp = ', num2str(Kp_value)]);
    hold on;
end

% Add labels and title to the plot
xlabel('Time (minutes)');
ylabel('Temperature (°C)');
title(['Temperature Variation for Different Kp Values (mc = ', num2str(mc_value), ')']);
grid on;
legend('show');
hold off;




%% Steady-State Error vs. Proportional Gain for Different Kp Values

% Parameters
theta_outd = 32;         
theta_o = 31;            
theta_set = 26;          
U = 0.13;                
A = 242;                 

% Updated P and mc values
P = 2.3143 * 60;        
mc_value = 10 * 8 * 4.5 * 1.18; 

k = U * A;               

% Range of Kp values
Kp_values = 100:100:1000;

% Initialize arrays to store Kp and steady-state error
steady_state_errors = zeros(size(Kp_values));

% Loop through each Kp value
for i = 1:length(Kp_values)
    Kp_value = Kp_values(i);

    % Calculate the approximate steady-state temperature
    steady_state_temp = (theta_outd * k + P + Kp_value * theta_set) / (k + Kp_value); % More direct steady-state formula

    % Calculate the steady-state error
    steady_state_error = abs(theta_set - steady_state_temp);
    steady_state_errors(i) = steady_state_error;
end

% Plot the steady-state error vs. Kp
figure;
plot(Kp_values, steady_state_errors, 'b-');
xlabel('Proportional Gain (Kp)');
ylabel('Steady-State Error (°C)');
title('Steady-State Error vs. Proportional Gain');
grid on;



%% 4. Response time vs Proportional Gain for Different Kp Values

% Parameters
theta_outd = 32;         
theta_o = 31;            
theta_set = 26;          
U = 0.13;                
A = 242;                 
P = 2.3143 * 60;        
mc_value = 10 * 8 * 4.5 * 1.18; 
k = U * A;               

% Range of Kp values
Kp_values = 100:100:1000;

% Time span for simulation 
tspan = [0, 1000]; 

% Tolerance for reaching steady state 
tolerance_percent = 1;

% Initialize array to store response times
response_times = zeros(size(Kp_values));

% Initialize figure
figure;

% Loop through each Kp value
for i = 1:length(Kp_values)
    Kp_value = Kp_values(i);
    alpha = k / mc_value;

    % Define the differential equation function
    dtheta_dt = @(t, theta) -alpha * (theta - (theta_outd - (Kp_value * (theta - theta_set)) / k + P / k));

    % Initial condition
    initial_theta = theta_o;

    % Solve the differential equation using ode45
    [t, theta] = ode45(dtheta_dt, tspan, initial_theta);

    % Calculate the steady-state temperature
    steady_state_temp = (theta_outd * k + P + Kp_value * theta_set) / (k + Kp_value);

    % Determine the response time
    difference = abs(steady_state_temp - initial_theta);
    tolerance = tolerance_percent / 100 * difference;
    settling_time_index = find(abs(theta - steady_state_temp) <= tolerance, 1, 'first');

    if ~isempty(settling_time_index)
        response_times(i) = t(settling_time_index);
    else
        response_times(i) = NaN; % Did not reach steady state within the simulation time
    end
end

% Plot the response time vs. Kp
figure;
plot(Kp_values, response_times);
xlabel('Proportional Gain (Kp)');
ylabel(['Response Time (minutes) to within ', num2str(tolerance_percent), '%']);
title('Response Time vs. Proportional Gain');
grid on;



%% 5.	Temperature Response with PI Control for different ki values (kp = 42)

% Parameters
mc = 10 * 8 * 4.5 * 1.18;          
UA = 0.13*242;         
Kp = 42;          
ki_values = 200:200:1000;           
theta_set = 26;     
theta0 = 31;        
dtheta0 = 0;        
tspan = [0 100];   

% Define the ODE system for PI control

% Plot the response
figure;
for ki_value=ki_values
    
    ode_fun = @(t, y) [
    y(2);  % dθ/dt = y(2)
    (ki_value * theta_set - (UA + Kp) * y(2) - ki_value * y(1)) / mc  % d²θ/dt²
     ];
 
    % Solve the ODE
    [t, y] = ode45(ode_fun, tspan, [theta0; dtheta0]);
    theta = y(:, 1);  % Extract temperature

    plot(t, theta,'DisplayName', ['Ki = ', num2str(ki_value)]);
    hold on;
end

xlabel('Time (minutes)');
ylabel('Indoor Temperature (°C)');
title('Temperature Response with PI Control');
grid on;
ylim([20 32]);
legend('show');
hold off;



%% 6. Steady state error vs PI controller for different ki values (kp = 42)

% Parameters
mc = 10 * 8 * 4.5 * 1.18;          
UA = 0.13 * 242;         
Kp = 42;          
ki_values = 100:100:1000;           
theta_set = 26;     
theta0 = 31;        
dtheta0 = 0;        
tspan = [0 300];    % Increase time to let system settle

% Initialize error storage
steady_state_errors = zeros(size(ki_values));

% Loop through Ki values to compute steady-state errors
for i = 1:length(ki_values)
    ki_value = ki_values(i);

    % Define ODE system
    ode_fun = @(t, y) [
        y(2);  % dθ/dt
        (ki_value * theta_set - (UA + Kp) * y(2) - ki_value * y(1)) / mc  % d²θ/dt²
    ];

    % Solve the ODE
    [t, y] = ode45(ode_fun, tspan, [theta0; dtheta0]);

    % Final temperature value
    theta_final = y(end, 1);

    % Compute steady-state error
    steady_state_errors(i) = abs(theta_set - theta_final);
end

% Plot steady-state error vs Ki
figure;
plot(ki_values, steady_state_errors, '-o', 'LineWidth', 2);
xlabel('Ki Value');
ylabel('Steady-State Error (°C)');
title('Steady-State Error vs. Ki for PI Control');
grid on;



%% 7. Response time vs PI controller for different ki values (kp = 42)

% Parameters
mc = 10 * 8 * 4.5 * 1.18;          
UA = 0.13 * 242;         
Kp = 42;          
ki_values = 200:200:1000;           
theta_set = 26;     
theta0 = 31;        
dtheta0 = 0;        
tspan = [0 300];    

% Settling tolerance (e.g., ±0.5°C)
tolerance = 1;

% Initialize storage
response_times = zeros(size(ki_values));

% Loop through Ki values
for i = 1:length(ki_values)
    ki_value = ki_values(i);

    % Define ODE system
    ode_fun = @(t, y) [
        y(2);  % dθ/dt
        (ki_value * theta_set - (UA + Kp) * y(2) - ki_value * y(1)) / mc  % d²θ/dt²
    ];

    % Solve the ODE
    [t, y] = ode45(ode_fun, tspan, [theta0; dtheta0]);
    theta = y(:,1);

    % Find time when theta stays within the tolerance band
    within_band = abs(theta - theta_set) <= tolerance;

    % Determine response time: last time before it stays within band
    response_time = NaN;
    for j = 1:length(t)
        if all(within_band(j:end))
            response_time = t(j);
            break;
        end
    end

    % Store response time
    response_times(i) = response_time;
end

% Plot response time vs Ki
figure;
plot(ki_values, response_times, '-o', 'LineWidth', 2);
xlabel('Ki Value');
ylabel('Response Time (minutes)');
title('Response Time vs. Ki for PI Control');
grid on;




%% 8. Temperature Variation for Different Kp Values for two different set temperature values.

% Parameters
theta_outd = 32;          
theta_o = 31;             
theta_set_initial = 25;   
theta_set_changed = 26;   % °C (setpoint after 10 minutes)
U = 0.13;                 % kJ m^-2 °C^-1 min^-1
A = 242;                  % m^2

% Updated P and mc values
P = 2.3143 * 60;          % kJ min^-1
mc_value = 10 * 8 * 4.5 * 1.18; % kJ °C^-1

k = U * A;                % kJ °C^-1 min^-1

% Range of Kp values
Kp_values = 500:500:5000;

% Time span for the first part of the simulation
tspan1 = [0, 10]; % Simulate for the first 10 minutes

% Initialize figure
figure;

% Loop through each Kp value
for Kp_value = Kp_values
    alpha = k / mc_value;         % min^-1

    % Define the differential equation function for the first part
    dtheta_dt1 = @(t, theta) -alpha * (theta - (theta_outd - (Kp_value * (theta - theta_set_initial)) / k + P / k));

    % Initial condition
    initial_theta = theta_o;

    % Solve the differential equation for the first part using ode45
    [t1, theta1] = ode45(dtheta_dt1, tspan1, initial_theta);

    % The final temperature of the first part becomes the initial temperature
    initial_theta2 = theta1(end);

    % Time span for the second part of the simulation (shifted by 10 minutes)
    tspan2 = [10, 300];

    % Define the differential equation function for the second part with the
    % changed theta_set
    dtheta_dt2 = @(t, theta) -alpha * (theta - (theta_outd - (Kp_value * (theta - theta_set_changed)) / k + P / k));

    % Solve the differential equation for the second part using ode45
    [t2, theta2] = ode45(dtheta_dt2, tspan2, initial_theta2);

    % Combine the time and temperature vectors for plotting
    t = [t1; t2];
    theta = [theta1; theta2];

    % Plot the results for each Kp
    plot(t, theta, 'DisplayName', ['Kp = ', num2str(Kp_value)]);
    hold on;
end

% Add labels and title to the plot
xlabel('Time (minutes)');
ylabel('Temperature (°C)');
title(['Temperature Variation for Different Kp Values (mc = ', num2str(mc_value), ')']);
grid on;
legend('show');
hold off;


%% 9.	Steady state errors for two different set temperature values with proportional controller

% Parameters
theta_outd = 32;          
theta_o = 31;            
theta_set_initial = 25;  
theta_set_changed = 26;   
U = 0.13;                 
A = 242;                  

% Updated P and mc values
P = 2.3143 * 60;          
mc_value = 10 * 8 * 4.5 * 1.18; 

k = U * A;                

% Range of Kp values
Kp_values = 500:500:5000;

% Time span for the first part of the simulation
tspan1 = [0, 10]; 

% Time span for the second part of the simulation
tspan2 = [10, 300]; 

% Initialize arrays to store steady-state errors
steady_state_error_initial = zeros(size(Kp_values));
steady_state_error_changed = zeros(size(Kp_values));


% Loop through each Kp value
for i = 1:length(Kp_values)
    Kp_value = Kp_values(i);
    alpha = k / mc_value;         

    % --- Simulation for the first part (theta_set = 25) ---
    dtheta_dt1 = @(t, theta) -alpha * (theta - (theta_outd - (Kp_value * (theta - theta_set_initial)) / k + P / k));
    initial_theta = theta_o;
    [t1, theta1] = ode45(dtheta_dt1, tspan1, initial_theta);

    % --- Simulation for the second part (theta_set = 26) ---
    initial_theta2 = theta1(end);
    dtheta_dt2 = @(t, theta) -alpha * (theta - (theta_outd - (Kp_value * (theta - theta_set_changed)) / k + P / k));
    [t2, theta2] = ode45(dtheta_dt2, tspan2, initial_theta2);

    % Combine time and temperature
    t = [t1; t2];
    theta = [theta1; theta2];


    % Approximate steady-state error for the initial setpoint (before 10 % min)
    steady_state_temp_initial_simulated = theta1(end);
    steady_state_error_initial(i) = abs(theta_set_initial - steady_state_temp_initial_simulated);

    % Approximate steady-state error for the changed setpoint (after 10 min)
    steady_state_temp_changed_simulated = theta2(end);
    steady_state_error_changed(i) = abs(theta_set_changed - steady_state_temp_changed_simulated);
end

figure;

plot(Kp_values, steady_state_error_initial, '-o');
xlabel('Kp');
ylabel('Steady-State Error (°C) (theta\_set = 25 °C)');
title('Steady-State Error vs. Kp (Initial Setpoint)');
grid on;

figure;
plot(Kp_values, steady_state_error_changed, '-o');
xlabel('Kp');
ylabel('Steady-State Error (°C) (theta\_set = 26 °C)');
title('Steady-State Error vs. Kp (Changed Setpoint)');
grid on;



%% 10. Temperature variation with PI controller for Different Ki Values for two different set temperature values. (kp = 42)

% Parameters
mc = 10 * 8 * 4.5 * 1.18;          
UA = 0.13 * 242;         
Kp = 42;          
ki_values = 200:200:1000;           
theta_set1 = 25;     % Initial setpoint
theta_set2 = 26;     % New setpoint after switch
theta0 = 31;        
dtheta0 = 0;        
tspan = [0 100];   
t_switch = 10;     % Time at which setpoint changes

% Initialize plot
figure;

for ki_value = ki_values

    % Define time-varying ODE with dynamic theta_set
    ode_fun = @(t, y) [
        y(2);  % dθ/dt
        (ki_value * (t < t_switch)*theta_set1 + ...
         ki_value * (t >= t_switch)*theta_set2 - ...
         (UA + Kp) * y(2) - ...
         ki_value * y(1)) / mc  % d²θ/dt²
    ];

    % Solve ODE
    [t, y] = ode45(ode_fun, tspan, [theta0; dtheta0]);
    theta = y(:, 1);

    % Plot
    plot(t, theta, 'DisplayName', ['Ki = ', num2str(ki_value)]);
    hold on;
end

% Plot formatting
xlabel('Time (minutes)');
ylabel('Indoor Temperature (°C)');
title(['Temperature Variation for Different Ki Values (mc = ', num2str(mc_value), ')']);
grid on;
ylim([20 32]);
legend('show');
hold off;



%% 11. Steady state errors for two different set temperature values with PI controller for different ki values. (kp = 42)

% Parameters
theta_outd = 32;
theta_o = 31;
theta_set_initial = 25;
theta_set_changed = 26;
U = 0.13;
A = 242;

% Updated P and mc values
P = 2.3143 * 60;
mc_value = 10 * 8 * 4.5 * 1.18;

k = U * A;

% Fixed Kp value
Kp_value = 42;

% Range of Ki values
Ki_values = 200:200:1000;     % Adjusted range for Ki

% Time span for the first part of the simulation
tspan1 = [0, 10];

% Time span for the second part of the simulation
tspan2 = [10, 300];

% Initialize arrays to store steady-state errors
steady_state_error_initial = zeros(size(Ki_values));
steady_state_error_changed = zeros(size(Ki_values));

% Loop through each Ki value
for j = 1:length(Ki_values)
    Ki_value = Ki_values(j);
    alpha = k / mc_value;

    % Initialize integral error
    integral_error = 0;

    % --- Simulation for the first part (theta_set = 25) ---
    dtheta_dt1 = @(t, vars) HeatTransferPI(t, vars, theta_outd, Kp_value, Ki_value, k, P, alpha, theta_set_initial);
    initial_vars = [theta_o; 0]; % [theta; integral_error]
    [t1, vars1] = ode45(dtheta_dt1, tspan1, initial_vars);
    theta1 = vars1(:, 1);
    integral_error_end1 = vars1(end, 2);

    % --- Simulation for the second part (theta_set = 26) ---
    initial_vars2 = [theta1(end); integral_error_end1];
    dtheta_dt2 = @(t, vars) HeatTransferPI(t, vars, theta_outd, Kp_value, Ki_value, k, P, alpha, theta_set_changed);
    [t2, vars2] = ode45(dtheta_dt2, tspan2, initial_vars2);
    theta2 = vars2(:, 1);

    % Combine time and temperature
    t = [t1; t2];
    theta = [theta1; theta2];

    % Approximate steady-state error for the initial setpoint (before 10 min)
    steady_state_temp_initial_simulated = theta1(end);
    steady_state_error_initial(j) = abs(theta_set_initial - steady_state_temp_initial_simulated);

    % Approximate steady-state error for the changed setpoint (after 10 min)
    steady_state_temp_changed_simulated = theta2(end);
    steady_state_error_changed(j) = abs(theta_set_changed - steady_state_temp_changed_simulated);
end


% --- Plotting the results ---
figure;
plot(Ki_values, steady_state_error_initial, '-o');
xlabel('Ki');
ylabel('Steady-State Error (°C) (theta\_set = 25 °C)');
title(['Steady-State Error vs. Ki (Kp = ', num2str(Kp_value), ', Initial Setpoint)']);
grid on;

figure;
plot(Ki_values, steady_state_error_changed, '-o');
xlabel('Ki');
ylabel('Steady-State Error (°C) (theta\_set = 26 °C)');
title(['Steady-State Error vs. Ki (Kp = ', num2str(Kp_value), ' , Changed Setpoint)']);
grid on;

% --- Function for the PI controller dynamics ---
function dvars_dt = HeatTransferPI(t, vars, theta_outd, Kp, Ki, k, P, alpha, theta_set)
    theta = vars(1);
    integral_error = vars(2);
    error = theta_set - theta;
    control_signal = Kp * error + Ki * integral_error;
    dtheta_dt = -alpha * (theta - (theta_outd - control_signal / k + P / k));
    dintegral_error_dt = error;
    dvars_dt = [dtheta_dt; dintegral_error_dt];
end



%% 12.Temperature variation with proportional controller when outdoor temperature changes steadily

% Parameters
theta_outd_initial = 32;    
theta_outd_final = 34;      
change_duration = 60;     
theta_o = 31;               
theta_set = 26;             
U = 0.13;                   
A = 242;                    

% Updated P and mc values
P = 2.3143 * 60;            
mc_value = 10 * 8 * 4.5 * 1.18; 

k = U * A;                  

% Range of Kp values
Kp_values = 500:500:5000;

% Time span for the simulation
tspan = [0, 300]; 

% Initialize figure
figure;

% Loop through each Kp value
for Kp_value = Kp_values
    alpha = k / mc_value;         

    % Define the differential equation function with time-varying theta_outd (using local function)
    dtheta_dt = @(t, theta) -alpha * (theta - (theta_outd_t(t) - (Kp_value * (theta - theta_set)) / k + P / k));

    % Initial condition
    initial_theta = theta_o;

    % Solve the differential equation using ode45
    [t, theta] = ode45(dtheta_dt, tspan, initial_theta);

    % Plot the results for each Kp
    plot(t, theta, 'DisplayName', ['Kp = ', num2str(Kp_value)]);
    hold on;
end

% Add labels and title to the room temperature subplot
xlabel('Time (minutes)');
ylabel('Room Temperature (°C)');
title(['Room Temperature Variation for Different Kp Values (mc = ', num2str(mc_value), ')']);
grid on;
legend('show');
hold off;

figure;

% Create time vector for plotting outdoor temperature
t_outd = 0:1:tspan(end);
theta_outd_plot = arrayfun(@(t) theta_outd_t(t), t_outd);

plot(t_outd, theta_outd_plot, 'b-', 'LineWidth', 2); % Plot outdoor temperature in blue
xlabel('Time (minutes)');
ylabel('Outdoor Temperature (°C)');
title('Outdoor Temperature Variation');
grid on;

% Local function definition (must be at the end of the script)
function theta_out = theta_outd_t(t)
    if t <= 0
        theta_out = 32; % theta_outd_initial;
    elseif t <= 60
        theta_out = 32 + (34 - 32) * (t / 60); % Linear increase
    else
        theta_out = 34; % theta_outd_final;
    end
end



%% 13. Steady state error with different kp values when outdoor temperature changes steadily

% Parameters
theta_outd_initial = 32;    
theta_outd_final = 34;      
change_duration = 60;     
theta_o = 31;               
theta_set = 26;             
U = 0.13;                   
A = 242;                    

% Updated P and mc values
P = 2.3143 * 60;            
mc_value = 10 * 8 * 4.5 * 1.18; 

k = U * A;                  

% Range of Kp values
Kp_values = 500:500:5000;

% Time span for the simulation
tspan = [0, 300]; 

% Initialize arrays to store results
steady_state_errors = zeros(size(Kp_values));
response_times = zeros(size(Kp_values));

% Initialize figure
figure;


% Loop through each Kp value
for i = 1:length(Kp_values)
    Kp_value = Kp_values(i);
    alpha = k / mc_value;         

    % Define the differential equation function with time-varying theta_outd (using local function)
    dtheta_dt = @(t, theta) -alpha * (theta - (theta_outd_t(t) - (Kp_value * (theta - theta_set)) / k + P / k));

    % Initial condition
    initial_theta = theta_o;

    % Solve the differential equation using ode45
    [t, theta] = ode45(dtheta_dt, tspan, initial_theta);

    % Plot the results for each Kp
    plot(t, theta, 'DisplayName', ['Kp = ', num2str(Kp_value)]);
    hold on;

    % Approximate steady-state error (using the final value)
    steady_state_errors(i) = abs(theta_set - theta(end));

    % Approximate response time (time to reach within 1% of steady state)
    final_value = theta(end);
    tolerance = 0.01 * abs(theta(end) - theta_o); % 2% of the total change
    settling_threshold_upper = final_value + tolerance;
    settling_threshold_lower = final_value - tolerance;

    settling_index = find(t >= 60 & theta >= settling_threshold_lower & theta <= settling_threshold_upper, 1, 'first'); % Start checking after outdoor temp stabilizes
    if ~isempty(settling_index)
        response_times(i) = t(settling_index) - 60; % Time after the outdoor temp finished changing
    else
        response_times(i) = NaN; % Did not settle within the simulation time
    end
end

% plot for Steady-State Error vs Kp
plot(Kp_values, steady_state_errors, 'r-o');
xlabel('Kp Value');
ylabel('Steady-State Error (°C)');
title('Steady-State Error vs Kp');
ylim([0 2]);
grid on;

% Local function definition (must be at the end of the script)
function theta_out = theta_outd_t(t)
    if t <= 0
        theta_out = 32; % theta_outd_initial;
    elseif t <= 60
        theta_out = 32 + (34 - 32) * (t / 60); % Linear increase
    else
        theta_out = 34; % theta_outd_final;
    end
end
