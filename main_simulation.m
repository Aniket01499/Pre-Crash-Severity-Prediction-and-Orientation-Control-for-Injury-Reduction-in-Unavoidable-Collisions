addpath(genpath('C:\Users\Aniket Anil Kakde\Documents\MATLAB\Examples\R2024a\New System\Thesis'));
close all; clearvars -except ans; clc;

% --- 1. Load Map ---
if isfile('unavoidable_map.mat')
    MapData = load('unavoidable_map.mat');
else
    error('Run Scenario_validation.m first!');
end

% --- 2. Test Scope ---
test_angles = -15; % -30:5:10
nTests = numel(test_angles);

all_results = repmat(struct('angle',[],'dist',[],'res_un',[],'res_aeb',[],'res_ctrl',[]), nTests, 1);

fprintf('=== AUTOMATED SIMULATION SWEEP (%d Angles) ===\n', nTests);

% --- 3. Main Angle Loop ---
for i = 1:nTests
    ang = test_angles(i);
    
    % [A] Initialize Scenario
    scenario = setup_scenario();
    scenario.tgt.heading0_rad = deg2rad(ang);
    
% [B] Lookup Distance
    y0_dist = get_start_dist_from_map(MapData, scenario.ego.v0, scenario.tgt.v0, ang);
    
    if isnan(y0_dist)
        fprintf('\nSkipping Angle %+d: Classified as Miss/Glancing/Avoidable.\n', ang);
        
        % Fill placeholders to prevent errors in final table generation
        all_results(i).angle    = ang;
        all_results(i).dist     = NaN;
        
        % Dummy result struct to satisfy table helpers
        dummy_res = struct('collision_detected', true, ... % Set true to force print 'region'
                           'final_region', 'SKIPPED (Miss/Glance)', ...
                           'final_severity_S', NaN, ...
                           'final_dV2_lat_target', NaN);
                           
        all_results(i).res_un   = dummy_res;
        all_results(i).res_aeb  = dummy_res;
        all_results(i).res_ctrl = dummy_res;
        
        continue; % Skip simulation for this angle
    end
    scenario.ego.y0 = -y0_dist; 

    % [C] SETTINGS (Corrected Steering Rates)
    scenario.ego.max_steer = deg2rad(35); 
    
    v_kph = scenario.ego.v0 * 3.6;
    if v_kph <= 60
        scenario.mpc.steer_rate_max = deg2rad(60); % Agile (Low speed)
    elseif v_kph <= 100
        scenario.mpc.steer_rate_max = deg2rad(45); % Evasion (Mid speed)
    else
        scenario.mpc.steer_rate_max = deg2rad(15); % Stable (High speed)
    end
    
    scenario.apf.debug = false;
    
    % [D] AEB Config (Must be inside loop)
    scenario.aeb.trigger_distance_m = 25;
    scenario.aeb.brake_cmd          = scenario.ego.max_brake;
    scenario.aeb.coast_cmd          = 0.0;

    fprintf('\n------------------------------------------------------------\n');
    fprintf('TEST [%d/%d]: Angle = %+d deg | Start Dist = %.2f m\n', i, nTests, ang, y0_dist);
    fprintf('------------------------------------------------------------\n');

    % ---------- Run Cases ----------
    % 1. Uncontrolled
    res_un = run_uncontrolled_simulation(scenario);

    % 2. AEB Only
    res_aeb = run_aeb_only_simulation(scenario);
    
    % 3. Controlled (APF+MPC)
    clear compute_apf_reference; % Reset persistent memory
    res_ctrl = run_controlled_simulation(scenario);

    % ---------- Store Results ----------
    all_results(i).angle    = ang;
    all_results(i).dist     = y0_dist;
    all_results(i).res_un   = res_un;
    all_results(i).res_aeb  = res_aeb;
    all_results(i).res_ctrl = res_ctrl;

    % ---------- VISUALIZATION (RESTORED) ----------
    % This will open a new figure for every angle showing Trajectories & Inputs
    compare_three_case_results(res_un, res_aeb, res_ctrl, scenario, ang);
    drawnow; 
end

% --- 4. Final Master Table ---
fprintf('\n================================================================\n');
fprintf('                   FINAL COMPARISON SUMMARY                     \n');
fprintf('================================================================\n');

Angles      = [all_results.angle]';
StartDist   = [all_results.dist]';

% Columns
Un_Coll = cell(nTests,1); Un_S = zeros(nTests,1); Un_dV = zeros(nTests,1);
AEB_Coll= cell(nTests,1); AEB_S= zeros(nTests,1); AEB_dV= zeros(nTests,1);
MPC_Coll= cell(nTests,1); MPC_S= zeros(nTests,1); MPC_dV= zeros(nTests,1);

for i = 1:nTests
    % Uncontrolled
    Un_Coll{i} = get_coll_str(all_results(i).res_un);
    Un_S(i)    = get_severity(all_results(i).res_un);
    Un_dV(i)   = get_deltaV(all_results(i).res_un);

    % AEB
    AEB_Coll{i}= get_coll_str(all_results(i).res_aeb);
    AEB_S(i)   = get_severity(all_results(i).res_aeb);
    AEB_dV(i)  = get_deltaV(all_results(i).res_aeb);

    % MPC
    MPC_Coll{i}= get_coll_str(all_results(i).res_ctrl);
    MPC_S(i)   = get_severity(all_results(i).res_ctrl);
    MPC_dV(i)  = get_deltaV(all_results(i).res_ctrl);
end

MasterTable = table(Angles, StartDist, ...
    Un_Coll, Un_dV, Un_S, ...
    AEB_Coll, AEB_dV, AEB_S, ...
    MPC_Coll, MPC_dV, MPC_S, ...
    'VariableNames', {'Angle', 'Start', ...
                      'Un_Res', 'Un_dV', 'Un_S', ...
                      'AEB_Res', 'AEB_dV', 'AEB_S', ...
                      'MPC_Res', 'MPC_dV', 'MPC_S'});

disp(MasterTable);
assignin('base', 'MasterTable', MasterTable);
assignin('base', 'all_results', all_results);

%% ===================== SCENARIO SETUP =====================
function scenario = setup_scenario()
    % --- Simulation Parameters ---
    scenario.dt = 0.02;                % [s] Time step
    scenario.T_sim = 2.5;              % [s] Simulation time

    % --- Horizons (MPC) ---
    scenario.prediction_horizon = 25;  % Np: how many steps we look ahead
    scenario.control_horizon    = 10;   % Nu: how many distinct future moves we optimise

    % --- Ego Vehicle Initial Conditions ---
    scenario.ego.x0           = -2.3;
    scenario.ego.y0           = -14.5;
    scenario.ego.v0           = 20.84;  % [m/s] 13.89, 22.22, 30.56 - [K/h] 50, 80, 110
    scenario.ego.heading0_rad = deg2rad(90);

    scenario.ego.length = 4.6;
    scenario.ego.width  = 1.9;

    % --- Target Vehicle Initial Conditions ---
    scenario.tgt.x0           = -10.0;
    scenario.tgt.y0           = 2.2;
    scenario.tgt.v0           = 16.67;
    scenario.tgt.heading0_rad = deg2rad(0);

    scenario.tgt.length = 4.6;
    scenario.tgt.width  = 1.9;

    % --- Control Constraints (USED in mpc and plots) ---
    scenario.ego.max_throttle =  1.0;   % dimensionless command
    scenario.ego.max_brake    = -1.0;   % dimensionless command

    % Physical longitudinal acceleration limits [m/s^2]
    scenario.ego.a_long_max   =  6.5;   % max accel (throttle = +1)
    scenario.ego.a_long_min   = -7.0;   % max brake (throttle = -1)

    scenario.mpc.throttle_rate_max = 25.0;  % [1/s] change in throttle command
    scenario.ego.max_steer = deg2rad(35);
    v_kph = scenario.ego.v0 * 3.6;
    if v_kph <= 60
        scenario.mpc.steer_rate_max = deg2rad(60);
    elseif v_kph <= 100
        scenario.mpc.steer_rate_max = deg2rad(45);
    else
        scenario.mpc.steer_rate_max = deg2rad(15);
    end

    % --- Side Impact Region Definitions (used by APF + impact logic) ---
    scenario.regions = struct();
    scenario.regions.miss_front   = struct('range', [ 2.30,    2.50], 'weight', 0.10);
    scenario.regions.front_region = struct('range', [ 1.40,  2.30], 'weight', 0.20);
    scenario.regions.front_door   = struct('range', [ 0.20,  1.40], 'weight', 0.45);
    scenario.regions.b_pillar     = struct('range', [-0.20,  0.20], 'weight', 0.60);
    scenario.regions.back_door    = struct('range', [-1.40, -0.20], 'weight', 0.45);
    scenario.regions.rear_region  = struct('range', [-2.30, -1.40], 'weight', 0.20);
    scenario.regions.miss_rear    = struct('range', [  -2.50, -2.30], 'weight', 0.10);

    % Occupant assumption: young adult, 29 years, near-side driver
    scenario.occupant.age_years = 29;

    % ================= SEVERITY MODEL (ΔV + AGE + DIRECTION + REGION) ================
    % Logistic regression for thorax MAIS3+ in near-side impacts
    % From: "Characteristics of Crashes that Increase the Risk of Serious Injuries"
    % Table 4: near-side, thorax AIS3+, ΔV (mph), age (years)
    %
    %   w = β0 + β1 * ΔV_mph + β2 * age_years
    %   p = 1 / (1 + exp(-w))
    %
    % NOTE: Freeze age at 29 years (young adult), as per euro report.
    scenario.severity.age_ref = 29;   % [years]

    scenario.severity.thorax_MAIS3.beta0    = -5.989;   % intercept
    scenario.severity.thorax_MAIS3.beta_dv  = 0.167;    % per mph of ΔV
    scenario.severity.thorax_MAIS3.beta_age = 0.0401;   % per year of age

    % ============ CLOCK SYSTEM (IMPACT DIRECTION) ===============
   
    % Pelvis AIS3+ logistic model (near-side), uses ln(lateral Delta-V)
    scenario.severity.pelvis_logit.b0 = -11.911;
    scenario.severity.pelvis_logit.b_lnV = 2.826;
    scenario.severity.pelvis_logit.b_angle10 = 1.286;  % Angle10 = 1 for 10 o'clock, else 0
    scenario.severity.pelvis_logit.minV_kph = 0.5;      % avoid ln(0)

    % IMPORTANT:
    %  - To extend to 360°, add more angles and probability rows:
    %      scenario.severity.pelvis.angles_deg = [..., new_angle_deg];
    %      scenario.severity.pelvis.prob_MAIS3 = [existing_rows; new_row];
    %  - For "less severe" directions, choose lower curves (e.g. fraction of 9h curve).

        % ============ ANGLE-BAND WEIGHTS FOR NEAR-SIDE RISK ============
    % We want:
    %   - 265°–280°  : medium risk
    %   - 280°–305°  : highest risk
    %   - otherwise  : low/normal risk (assumption)
    %
    % These are *design* ORs (odds ratios) used to make APF avoid these bands.
    % Tune OR_med and OR_high to adjust how aggressively APF avoids them.
    scenario.severity.angle_band.deg_low  = 265;  % lower bound of interest
    scenario.severity.angle_band.deg_med1 = 265;  % [265, 280): medium
    scenario.severity.angle_band.deg_med2 = 280;
    scenario.severity.angle_band.deg_high1 = 280; % [280, 305]: high
    scenario.severity.angle_band.deg_high2 = 305;

    scenario.severity.angle_band.OR_low  = 1.0;  % base risk for other angles
    scenario.severity.angle_band.OR_med  = 2.0;  % medium risk
    scenario.severity.angle_band.OR_high = 5.0;  % very high risk


    % --- Vehicle Model Parameters (Ego & Target use same structure) ---
    % These ARE used in state_function / veh_continuous_dynamics
    scenario.ego.mass = 1600;          % [kg]
    scenario.ego.Iz   = 2500;          % [kg m^2]
    scenario.ego.lf   = 1.35;          % [m]
    scenario.ego.lr   = 1.35;          % [m]
    scenario.ego.csf  = 8e4;           % [N/rad]
    scenario.ego.csr  = 1e5;           % [N/rad]

    scenario.tgt.mass = 1600;
    scenario.tgt.Iz   = 2500;
    scenario.tgt.lf   = 1.35;
    scenario.tgt.lr   = 1.35;
    scenario.tgt.csf  = 8e4;
    scenario.tgt.csr  = 1e5;

    % APF path-planning parameters
    scenario.apf.v_ref_min     = 1.0;  % minimum reference speed along path [m/s]
    scenario.apf.lock_distance = 5.0;   % [m] – tune if needed

    % --- Road Parameters (USED in plotting) ---
    scenario.road.width  = 14;
    scenario.road.length = 40;
end


%% ===================== UNCONTROLLED SIMULATION =====================
function results = run_uncontrolled_simulation(scenario)
% Run simulation WITHOUT any controller

    % Initialize vehicle states
    ego_state = initialize_veh_state(scenario.ego);
    tgt_state = initialize_veh_state(scenario.tgt);
    collision_region  = '';
    collision_deltaV  = NaN;
    collision_step    = NaN;

    
    % Pre-compute target trajectory
    time = 0:scenario.dt:scenario.T_sim;
    N_steps = length(time);
    tgt_trajectory = precompute_target_trajectory(tgt_state, time, scenario);
    
    % Storage for states
    ego_traj = zeros(8, N_steps);
    collision_detected = false;
    impact_region_history = cell(1, N_steps);
    
    % Initialize trajectory
    ego_traj(:,1) = [ego_state.x; ego_state.y; ego_state.psi; ego_state.v; ...
                    ego_state.psi_dot; ego_state.beta; 0; 0];
    
    % Create figure
    fig = figure('Position', [100, 100, 1200, 800]);
    
    for k = 1:N_steps-1
        current_time = time(k);
        current_tgt_state = tgt_trajectory(k);
        
        % UNCONTROLLED - maintain constant inputs
        ego_state.throttle = 0;  % NO braking
        ego_state.steering = 0;  % NO steering
        impact_region_history{k} = 'uncontrolled';
        
        % Update dynamics
        ego_state = update_veh_dynamics(ego_state, scenario);
        
        % Check collision
        [collision, impact_point, vrel_mag, vrel_lat, vrel_long, dV2_mag, dV2_lat, alpha_rad] = ...
            check_collision_0(ego_state, current_tgt_state, scenario);
        
        if collision && ~collision_detected
            collision_detected = true;
            collision_time = current_time;
            collision_step = k;

            [impact_region, impact_side] = determine_impact_region(impact_point, scenario);
            collision_region = impact_region;            
            collision_alpha_rad = alpha_rad;

            % Approach speed (NOT delta-V)
            collision_vrel_lat  = vrel_lat;
            collision_vrel_mag  = vrel_mag;
            collision_vrel_long = vrel_long;
            
            % Target delta-V proxy (momentum transfer)
            collision_dV2_lat = dV2_lat;
            collision_dV2_mag = dV2_mag;
            
            % Severity: pelvis model should use lateral delta-V proxy
            S   = compute_severity_score(collision_dV2_lat, impact_region, collision_alpha_rad, scenario);
            
            % Baseline thorax model (if you still want it) can use magnitude proxy
            pDV = compute_baseline_severity_deltaV(collision_dV2_mag, scenario);
                      
            fprintf('*** COLLISION DETECTED at t=%.2fs ***\n', current_time);
            fprintf('Impact region: %s (%s side)\n', impact_region, impact_side);
            fprintf('Approach (rel) lateral speed: %.2f m/s\n', vrel_lat);
            fprintf('Target ΔV proxy lateral:      %.2f m/s\n', dV2_lat);

            impact_region_history{k} = impact_region;
        end
        
        % Store trajectory
        ego_traj(:,k+1) = [ego_state.x; ego_state.y; ego_state.psi; ego_state.v; ...
                          ego_state.psi_dot; ego_state.beta; ego_state.throttle; ego_state.steering];
        
        % Visualization
        plot_apf_scenario(ego_state, current_tgt_state, tgt_trajectory, scenario, ...
                         current_time, false, impact_region_history{k}, ...
                         vrel_lat, []);
        drawnow;
        
        % Stop after collision
        if collision_detected && (k >= collision_step + 2)
            fprintf('Stopping simulation after collision\n');
            break;
        end
    end
    
    % Store results
    k_end = min(k + 1, N_steps);
    results.ego_traj = ego_traj(:,1:k_end);
    results.tgt_trajectory = tgt_trajectory(1:k_end);
    results.time = time(1:k_end);
    results.impact_region_history = impact_region_history(1:k_end);
    results.collision_detected = collision_detected;

    if collision_detected
        results.collision_time  = collision_time;
        results.collision_step  = collision_step;
        results.final_region    = collision_region;
        % Keep old name if other code depends on it, but redefine meaning clearly:
        results.final_vrel_lat = collision_vrel_lat;     % approach speed lateral
        results.final_vrel_mag = collision_vrel_mag;
        results.final_vrel_long = collision_vrel_long;
        
        results.final_dV2_lat_target = collision_dV2_lat; % target delta-V proxy lateral
        results.final_dV2_mag_target = collision_dV2_mag;
        
        results.final_alpha_rad = collision_alpha_rad;
        results.final_alpha_deg = rad2deg(collision_alpha_rad);
        
        results.use_apf_mpc = false;
        results.final_severity_S = S;
        results.final_pDV = pDV;
        
        results.final_lambdaR = scenario.regions.(impact_region).weight;
        results.final_lambdaA = compute_angle_weight_dv(results.final_dV2_lat_target, collision_alpha_rad, scenario);
    end
end

function results = run_aeb_only_simulation(scenario)
% AEB-only = longitudinal control only (brake/coast), steering locked to 0

    ego_state = initialize_veh_state(scenario.ego);
    tgt_state = initialize_veh_state(scenario.tgt);

    time = 0:scenario.dt:scenario.T_sim;
    N_steps = numel(time);

    tgt_trajectory = precompute_target_trajectory(tgt_state, time, scenario);

    ego_traj = zeros(8, N_steps);
    impact_region_history = cell(1, N_steps);

    collision_detected = false;
    collision_region   = '';
    collision_deltaV_lat = NaN;
    collision_step     = NaN;

    ego_traj(:,1) = [ego_state.x; ego_state.y; ego_state.psi; ego_state.v; ...
                     ego_state.psi_dot; ego_state.beta; 0; 0];

    fig = figure('Position', [100, 100, 1200, 800]);

    drawnow;

    for k = 1:N_steps-1
        current_time = time(k);
        current_tgt_state = tgt_trajectory(k);

        % --- AEB logic (simple and deterministic) ---
        d_now = calculate_min_distance(ego_state, current_tgt_state);

        if d_now < scenario.aeb.trigger_distance_m
            ego_state.throttle = scenario.aeb.brake_cmd;   % full brake typically
        else
            ego_state.throttle = scenario.aeb.coast_cmd;   % 0
        end
        ego_state.steering = 0;                            % no lateral control
        impact_region_history{k} = 'aeb_only';

        % Update dynamics
        ego_state = update_veh_dynamics(ego_state, scenario);

        % Collision check
        [collision, impact_point, vrel_mag, vrel_lat, vrel_long, dV2_mag, dV2_lat, alpha_rad] = ...
            check_collision_0(ego_state, current_tgt_state, scenario);

        if collision && ~collision_detected
            collision_detected = true;
            collision_time = current_time;
            collision_step = k;

            [impact_region, impact_side] = determine_impact_region(impact_point, scenario);
            collision_region = impact_region;
            collision_vrel_lat  = vrel_lat;
            collision_vrel_mag  = vrel_mag;
            collision_vrel_long = vrel_long;
            
            collision_dV2_lat = dV2_lat;
            collision_dV2_mag = dV2_mag;
            
            collision_alpha_rad = alpha_rad;
            
            S   = compute_severity_score(collision_dV2_lat, impact_region, collision_alpha_rad, scenario);
            pDV = compute_baseline_severity_deltaV(collision_dV2_mag, scenario);

            fprintf('*** (AEB) COLLISION DETECTED at t=%.2fs ***\n', current_time);
            fprintf('Impact region: %s (%s side)\n', impact_region, impact_side);
            fprintf('v_rel_lat=%.2f m/s | ΔV2_lat=%.2f m/s | α=%.1f deg | S=%.4f\n', ...
                vrel_lat, dV2_lat, rad2deg(collision_alpha_rad), S);

            impact_region_history{k} = impact_region;
        end

        ego_traj(:,k+1) = [ego_state.x; ego_state.y; ego_state.psi; ego_state.v; ...
                           ego_state.psi_dot; ego_state.beta; ego_state.throttle; ego_state.steering];

        % Live visualization (AEB)
        plot_apf_scenario(ego_state, current_tgt_state, tgt_trajectory, scenario, ...
                          current_time, 'AEB only', impact_region_history{k}, ...
                          vrel_lat, []);   % no APF ref
        drawnow;

        % Stop shortly after collision
        if collision_detected && (k >= collision_step + 2)
            break;
        end
    end

    k_end = min(k+1, N_steps);

    results.ego_traj = ego_traj(:,1:k_end);
    results.tgt_trajectory = tgt_trajectory(1:k_end);
    results.time = time(1:k_end);
    results.impact_region_history = impact_region_history(1:k_end);
    results.collision_detected = collision_detected;
    results.use_apf_mpc = false;
    results.case_name = 'aeb_only';

    if collision_detected
        results.collision_time  = collision_time;
        results.collision_step  = collision_step;
        results.final_region    = collision_region;
        results.final_vrel_lat  = collision_vrel_lat;
        results.final_vrel_mag  = collision_vrel_mag;
        results.final_vrel_long = collision_vrel_long;
        
        results.final_dV2_lat_target = collision_dV2_lat;
        results.final_dV2_mag_target = collision_dV2_mag;
        
        results.final_alpha_rad = collision_alpha_rad;
        results.final_alpha_deg = rad2deg(collision_alpha_rad);
        
        results.final_severity_S = S;
        results.final_pDV = pDV;
        
        results.final_lambdaR = scenario.regions.(collision_region).weight;
        results.final_lambdaA = compute_angle_weight_dv(results.final_dV2_lat_target, collision_alpha_rad, scenario);
    end
end

%% ===================== CONTROLLED SIMULATION =====================
function results = run_controlled_simulation(scenario)
% Run simulation with APF+MPC controller

    % Initialize vehicle states
    ego_state = initialize_veh_state(scenario.ego);
    tgt_state = initialize_veh_state(scenario.tgt);
    
    % Pre-compute target trajectory
    time = 0:scenario.dt:scenario.T_sim;
    N_steps = length(time);
    tgt_trajectory = precompute_target_trajectory(tgt_state, time, scenario);
    
    % Storage for states
    ego_traj = zeros(8, N_steps);
    collision_detected = false;
    impact_region_history = cell(1, N_steps);
    apf_reference_history = cell(1, N_steps);
    
    % Initialize trajectory
    ego_traj(:,1) = [ego_state.x; ego_state.y; ego_state.psi; ego_state.v; ...
                    ego_state.psi_dot; ego_state.beta; 0; 0];

    % Initialize last control for custom MPC (for rate penalties)
    lastMV = [0; 0];
    
    % Create figure
    fig = figure('Position', [100, 100, 1200, 800]);
    
    for k = 1:N_steps-1
        current_time = time(k);
        current_tgt_state = tgt_trajectory(k);
        
        clf;
        
        % === APF REFERENCE GENERATION ===
        [apf_reference, desired_region] = ...
            compute_apf_reference(ego_state, current_tgt_state, scenario);
        
        apf_reference_history{k} = apf_reference;
        impact_region_history{k} = desired_region;

        % DEBUG: log what APF is telling us to do (first point of ref)
        if scenario.apf.debug && ~isempty(apf_reference)
            fprintf(['t=%5.2f  APF ref[1]: x=%.2f  y=%.2f  psi=%.1fdeg  v=%.2f  ' ...
                     'desired_region=%s\n'], ...
                    current_time, ...
                    apf_reference(1,1), apf_reference(2,1), ...
                    rad2deg(apf_reference(3,1)), apf_reference(4,1), ...
                    desired_region);
        end

        % === NMPC via custom fmincon solver (APF-guided) ===
        try
            % Use APF reference (4 x N horizon) from compute_apf_reference
            [throttle_cmd, steering_cmd] = custom_mpc_solver(ego_state, current_tgt_state, scenario, apf_reference);
        
            ego_state.throttle = throttle_cmd;
            ego_state.steering = steering_cmd;
        
            lastMV = [throttle_cmd; steering_cmd];
        
            if scenario.apf.debug
                fprintf(['t=%5.2f  NMPC cmd: throttle=%.2f  steer=%.1fdeg\n'], ...
                        current_time, throttle_cmd, rad2deg(steering_cmd));
            end

        
        catch ME
            fprintf('Custom NMPC failed: %s\nUsing conservative control\n', ME.message);
            ego_state.throttle = -0.3;  % Gentle braking
            ego_state.steering = 0;
        end

  
        % Update dynamics
        ego_state = update_veh_dynamics(ego_state, scenario);

        if scenario.apf.debug
            fprintf(['          -> ego: x=%.2f  y=%.2f  psi=%.1fdeg  v=%.2f\n'], ...
                    ego_state.x, ego_state.y, ...
                    rad2deg(ego_state.psi + ego_state.beta), ego_state.v);
        end

        
        % === COLLISION CHECK WITH FULL SEVERITY OUTPUTS ===
        [collision, impact_point, vrel_mag, vrel_lat, vrel_long, dV2_mag, dV2_lat, alpha_rad] = ...
            check_collision_0(ego_state, current_tgt_state, scenario);
        
        if collision && ~collision_detected
            collision_detected = true;
            collision_time = current_time;
            collision_step = k;
        
            [impact_region, impact_side] = determine_impact_region(impact_point, scenario);
            collision_region = impact_region;
            collision_vrel_lat  = vrel_lat;
            collision_vrel_mag  = vrel_mag;
            collision_vrel_long = vrel_long;
            
            collision_dV2_lat = dV2_lat;
            collision_dV2_mag = dV2_mag;
            
            collision_alpha_rad = alpha_rad;
            
            S   = compute_severity_score(collision_dV2_lat, impact_region, collision_alpha_rad, scenario);
            pDV = compute_baseline_severity_deltaV(collision_dV2_mag, scenario);

            fprintf('*** COLLISION DETECTED at t=%.2fs ***\n', current_time);
            fprintf('Impact region: %s (%s side)\n', impact_region, impact_side);
            fprintf('v_rel_lat=%.2f m/s | ΔV2_lat=%.2f m/s\n', vrel_lat, dV2_lat);
            
            impact_region_history{k} = impact_region;
        end

        
        % Store trajectory
        ego_traj(:,k+1) = [ego_state.x; ego_state.y; ego_state.psi; ego_state.v; ...
                          ego_state.psi_dot; ego_state.beta; ego_state.throttle; ego_state.steering];
        
        % Visualization
        plot_apf_scenario(ego_state, current_tgt_state, tgt_trajectory, scenario, ...
                         current_time, true, impact_region_history{k}, ...
                         vrel_lat, apf_reference_history{k});
        drawnow;
        
        % Stop after collision
        if collision_detected && (k >= collision_step + 2)
            fprintf('Stopping simulation after collision\n');
            break;
        end
    end
    
    % Store results
    k_end = min(k + 1, N_steps);
    results.ego_traj = ego_traj(:,1:k_end);
    results.tgt_trajectory = tgt_trajectory(1:k_end);
    results.time = time(1:k_end);
    results.impact_region_history = impact_region_history(1:k_end);
    results.collision_detected = collision_detected;
    results.use_apf_mpc = true;


    % NEW: store APF reference used at each step
    results.apf_reference_history = apf_reference_history(1:k_end);
    
    if collision_detected
        results.collision_time  = collision_time;
        results.collision_step  = collision_step;
        results.final_region    = collision_region;
        results.final_region = collision_region;

        results.final_vrel_lat  = collision_vrel_lat;
        results.final_vrel_mag  = collision_vrel_mag;
        results.final_vrel_long = collision_vrel_long;
        
        results.final_dV2_lat_target = collision_dV2_lat;
        results.final_dV2_mag_target = collision_dV2_mag;
        
        results.final_alpha_rad = collision_alpha_rad;
        results.final_alpha_deg = rad2deg(collision_alpha_rad);
        
        results.final_severity_S = S;
        results.final_pDV = pDV;
        
        results.final_lambdaR = scenario.regions.(impact_region).weight;
        results.final_lambdaA = compute_angle_weight_dv(results.final_dV2_lat_target, collision_alpha_rad, scenario);
    end
end

%% ===================== VEHICLE MODELS =====================
function state = initialize_veh_state(params)
% Initialize single-track model state
    state.x = params.x0;
    state.y = params.y0;
    state.psi = params.heading0_rad;
    state.v = params.v0;
    state.psi_dot = 0;
    state.beta = 0;
    state.throttle = 0;
    state.steering = 0;
    state.length = params.length;
    state.width = params.width;
end

% function new_state = update_veh_dynamics(state, scenario)
% % Update vehicle state using proper ODE integration
%     dt = scenario.dt;
% 
%     % Current state vector for ODE solver
%     x0 = [state.x; state.y; state.psi; state.v; state.psi_dot; state.beta];
% 
%     % Control inputs
%     u = [state.throttle; state.steering];
% 
%     % Use ode45 for proper integration
%     t_span = [0, dt];
%     [~, x_out] = ode45(@(t,x) veh_continuous_dynamics(x, u, scenario), t_span, x0);
% 
%     % Extract final state
%     x_final = x_out(end, :)';
% 
%     % Update state struct
%     new_state = state;
%     new_state.x = x_final(1);
%     new_state.y = x_final(2);
%     new_state.psi = x_final(3);
%     new_state.v = max(0, x_final(4));  % Ensure non-negative velocity
%     new_state.psi_dot = x_final(5);
%     new_state.beta = x_final(6);
% end

% function dx = veh_continuous_dynamics(x, u, scenario)
% % Dynamic bicycle model WITHOUT drag/rolling.
% % States: [X; Y; psi; v; r; beta]
% % Inputs: u = [throttle_cmd; delta]
% %
% % throttle_cmd in [-1, 1] is mapped linearly to physical accel a_x in
% % [a_long_min, a_long_max].
% 
%     params = scenario.ego;
% 
%     % Extract states
%     X    = x(1);
%     Y    = x(2);
%     psi  = x(3);
%     v    = x(4);
%     r    = x(5);
%     beta = x(6);
% 
%     % Inputs
%     throttle_cmd = u(1);   % dimensionless [-1, 1]
%     delta        = u(2);   % steering [rad]
% 
%     % Saturate throttle in case MPC or code misbehaves
%     throttle_cmd = max(params.max_brake, ...
%                    min(params.max_throttle, throttle_cmd));
% 
%     % Map to physical longitudinal acceleration [m/s^2]
%     if throttle_cmd >= 0
%         a_x = throttle_cmd * params.a_long_max;
%     else
%         a_x = throttle_cmd * abs(params.a_long_min);  % braking limit
%     end
% 
% 
%     % Vehicle params
%     m   = params.mass;
%     Iz  = params.Iz;
%     lf  = params.lf;
%     lr  = params.lr;
%     csf = params.csf;
%     csr = params.csr;
% 
%     v_safe = max(v, 0.1);  % avoid div-by-zero
% 
%     % Linear tyre slip angles
%     alpha_f = delta - beta - (lf * r) / v_safe;
%     alpha_r =      - beta + (lr * r) / v_safe;
% 
%     % Lateral forces (2 tyres per axle)
%     F_yf = 2 * csf * alpha_f;
%     F_yr = 2 * csr * alpha_r;
% 
%     dx      = zeros(6,1);
%     dx(1)   = v * cos(psi + beta);        % Ẋ
%     dx(2)   = v * sin(psi + beta);        % Ẏ
%     dx(3)   = r;                          % ψ̇
% 
%     % *** No drag/rolling: pure commanded accel ***
%     dx(4)   = a_x;                        % v̇
% 
%     dx(5)   = (lf * F_yf - lr * F_yr) / Iz;               % ṙ
%     dx(6)   = (F_yf + F_yr) / (m * v_safe) - r;           % β̇
% end

function next_state = update_veh_dynamics(state, scenario)
% Optimized prediction for MPC: Uses Fiala Model + Euler Integration (No ODE45)
% This is theoretically robust for MPC because dt is small (0.02s).
params = scenario.ego;
    dt = scenario.dt;

    % --- 1. Extract States & Inputs ---
    x   = state.x;
    y   = state.y;
    psi = state.psi;
    v   = state.v;
    r   = state.psi_dot;
    beta= state.beta;
    
    % IMPORTANT: Read control inputs from the state struct
    throttle = state.throttle;
    steering = state.steering;

    % --- 2. Input Limits ---
    throttle = max(params.max_brake, min(params.max_throttle, throttle));
    
    % --- 3. Longitudinal Force ---
    if throttle >= 0
        ax = throttle * params.a_long_max;
    else
        ax = throttle * abs(params.a_long_min);
    end
    Fx_total = params.mass * ax;
    Fx_f = Fx_total * 0.5;
    Fx_r = Fx_total * 0.5;

    % --- 4. Vertical Loads (Static) ---
    g = 9.81;
    m = params.mass;
    lf = params.lf; lr = params.lr;
    Fz_f = (m * g * lr) / (lf + lr);
    Fz_r = (m * g * lf) / (lf + lr);

    % --- 5. Slip Angles ---
    v_safe = max(v, 1.0);
    alpha_f = steering - beta - (lf * r) / v_safe;
    alpha_r = -beta + (lr * r) / v_safe;

    % --- 6. Fiala Tire Forces ---
    mu = 0.9;
    
    % -- Front --
    Fy_max_f = sqrt((mu * Fz_f)^2 - Fx_f^2);
    alpha_sl_f = atan((3 * Fy_max_f) / (2 * params.csf));
    
    if abs(alpha_f) < alpha_sl_f
        tan_a = tan(alpha_f);
        Fyf_raw = -2*params.csf*tan_a + ...
                  ((2*params.csf)^2 / (3*Fy_max_f)) * abs(tan_a)*tan_a - ...
                  ((2*params.csf)^3 / (27*Fy_max_f^2)) * tan_a^3;
        Fyf = -Fyf_raw; % Invert for ISO convention
    else
        Fyf = Fy_max_f * sign(alpha_f);
    end

    % -- Rear --
    Fy_max_r = sqrt((mu * Fz_r)^2 - Fx_r^2);
    alpha_sl_r = atan((3 * Fy_max_r) / (2 * params.csr));
    
    if abs(alpha_r) < alpha_sl_r
        tan_a = tan(alpha_r);
        Fyr_raw = -2*params.csr*tan_a + ...
                  ((2*params.csr)^2 / (3*Fy_max_r)) * abs(tan_a)*tan_a - ...
                  ((2*params.csr)^3 / (27*Fy_max_r^2)) * tan_a^3;
        Fyr = -Fyr_raw; % Invert for ISO convention
    else
        Fyr = Fy_max_r * sign(alpha_r);
    end

    % --- 7. Dynamics (Euler Integration) ---
    next_state = state; 
    
    next_state.x       = x + (v * cos(psi + beta)) * dt;
    next_state.y       = y + (v * sin(psi + beta)) * dt;
    next_state.psi     = psi + r * dt;
    next_state.v       = max(0, v + ax * dt);
    next_state.psi_dot = r + ((lf * Fyf - lr * Fyr) / params.Iz) * dt;
    next_state.beta    = beta + ((Fyf + Fyr) / (m * v_safe) - r) * dt;
end

function trajectory = precompute_target_trajectory(initial_state, time_vector, scenario) %#ok<INUSD>
% KINEMATIC TARGET (constant v, constant heading). Ego stays dynamic elsewhere.

    N = numel(time_vector);
    trajectory = repmat(initial_state, N, 1);

    for i = 2:N
        dt = time_vector(i) - time_vector(i-1);

        % copy previous state
        trajectory(i) = trajectory(i-1);

        % kinematic propagation
        trajectory(i) = update_target_kinematic(trajectory(i), dt);

        % enforce constant speed + constant heading (deterministic)
        trajectory(i).v   = initial_state.v;
        trajectory(i).psi = initial_state.psi;

        % keep these consistent for a kinematic target
        trajectory(i).psi_dot  = 0;
        trajectory(i).beta     = 0;
        trajectory(i).throttle = 0;
        trajectory(i).steering = 0;
    end
end

%% ===================== NONLINEAR MPC CONTROLLER =====================
function [throttle, steering] = custom_mpc_solver(ego_state, tgt_state, scenario, apf_reference)
% Custom receding-horizon MPC using fmincon with:
% - prediction horizon Np
% - control horizon Nu ( >=1 )
% Decision vector: u_vec = [u1; u2; ...; uNu], each ui = [throttle; steer]

    % Extract current state
    x0 = [ego_state.x; ego_state.y; ego_state.psi; ego_state.v; ...
          ego_state.psi_dot; ego_state.beta];

    % Previous control (for rate limits)
    last_u = [ego_state.throttle; ego_state.steering];

    % If APF reference is empty, fall back to simple full-brake, no-steer
    % if isempty(apf_reference)
    %     throttle = -0.3;
    %     steering = 0.0;
    %     return;
    % end

    % --- Horizons ---
    Np_raw = scenario.prediction_horizon;
    Np     = min(Np_raw, size(apf_reference, 2));  % don't exceed ref length
    if Np < 1
        throttle = -0.3;
        steering = 0.0;
        return;
    end

    Nu_nominal = scenario.control_horizon;
    Nu         = min(Nu_nominal, Np);              % can't have more moves than steps

    % --- Physical bounds ---
    thr_min_phys = scenario.ego.max_brake;         % e.g. -1.0 (full brake)
    thr_max_phys = scenario.ego.max_throttle;      % e.g. +1.0
    steer_min_phys = -scenario.ego.max_steer;
    steer_max_phys =  scenario.ego.max_steer;

    % --- AEB behaviour: forbid positive throttle when "close" ---
    d_min_now = calculate_min_distance(ego_state, tgt_state);
    thr_max = thr_max_phys;
    if d_min_now < 15    % [m], tune threshold if needed
        thr_max = 0.0;   % pure braking / coasting only
    end

    % --- Rate limits (first move) ---
    % Approximate mapping:
    %  - throttle is normalised; choose rate s.t. 0 → -1 in ~0.1 s => 10 units/s
    %  - steering ~300 deg/s (typical electric steering limit)
    thr_rate_max   = scenario.mpc.throttle_rate_max;                   % [throttle units / s]
    steer_rate_max = scenario.mpc.steer_rate_max;           % [rad/s]

    thr_step_max   = thr_rate_max   * scenario.dt;
    steer_step_max = steer_rate_max * scenario.dt;

    thr_lb_first   = max(thr_min_phys, last_u(1) - thr_step_max);
    thr_ub_first   = min(thr_max,      last_u(1) + thr_step_max);

    steer_lb_first = max(steer_min_phys, last_u(2) - steer_step_max);
    steer_ub_first = min(steer_max_phys, last_u(2) + steer_step_max);

    % --- Bounds for ALL future moves ---
    % u_vec = [u1; u2; ...; uNu], each ui = [throttle; steer]
    lb = repmat([thr_min_phys; steer_min_phys], Nu, 1);
    ub = repmat([thr_max;      steer_max_phys], Nu, 1);

    % Apply rate limits to the FIRST move only (that's what we actually apply)
    lb(1) = thr_lb_first;
    ub(1) = thr_ub_first;
    lb(2) = steer_lb_first;
    ub(2) = steer_ub_first;

    % Debug print
    fprintf('Bounds first move: throttle [%+.2f, %+.2f], steer [%+.1f°, %+.1f°]\n', ...
        thr_lb_first, thr_ub_first, rad2deg(steer_lb_first), rad2deg(steer_ub_first));

    % --- Initial guess: repeat last_u over Nu moves ---
    u0 = repmat(last_u, Nu, 1);   % 2*Nu x 1 vector

    % --- Solver options ---
    options = optimoptions('fmincon', ...
                           'Display', 'off', ...    % keep output clean; set 'iter' if you want spam
                           'MaxIterations', 50, ...
                           'Algorithm', 'sqp');

    % Cost & constraints with explicit Nu
    cost_fun = @(u_vec) mpc_cost_function(u_vec, x0, last_u, ...
                                          apf_reference, scenario, tgt_state, Nu);
    nonl_con = @(u_vec) mpc_constraints(u_vec, x0, scenario, Nu);

    % --- Solve MPC ---
    u_opt = fmincon(cost_fun, u0, [], [], [], [], lb, ub, nonl_con, options);

    % Apply only the first move (receding horizon)
    u1 = u_opt(1:2);
    throttle = u1(1);
    steering = u1(2);
end

function J = mpc_cost_function(u_vec, x0, last_u, apf_reference, scenario, tgt_state, Nu)
    J = 0;

    U = reshape(u_vec, 2, []);
    if size(U,2) ~= Nu
        J = 1e6;
        return;
    end

    Np = min(scenario.prediction_horizon, size(apf_reference, 2));

    % Build state struct
    current_state = struct();
    current_state.x        = x0(1);
    current_state.y        = x0(2);
    current_state.psi      = x0(3);
    current_state.v        = x0(4);
    current_state.psi_dot  = x0(5);
    current_state.beta     = x0(6);
    current_state.length   = scenario.ego.length;
    current_state.width    = scenario.ego.width;
    current_state.throttle = 0;
    current_state.steering = 0;

    for k = 1:Np
        idx_u = min(Nu, k);          % simple mapping
        uk    = U(:, idx_u);

        current_state.throttle = uk(1);
        current_state.steering = uk(2);

        % Predict one step
        current_state = update_veh_dynamics(current_state, scenario);

        x_pred = [current_state.x; current_state.y; current_state.psi; current_state.v];
        ref_k  = apf_reference(:,k);   % [x_ref; y_ref; psi_ref; v_ref]

        % Tracking errors
        pos_err     = x_pred(1:2) - ref_k(1:2);
        heading_err = atan2( sin(x_pred(3) - ref_k(3)), ...
                             cos(x_pred(3) - ref_k(3)) );
        speed_err   = x_pred(4) - ref_k(4);

        % *** HEAVY weights on tracking ***
        J = J ...
            + 100 * (pos_err.' * pos_err) ...   % position dominates
            + 10  * (heading_err^2)       ...   % keep heading aligned
            + 50   * (speed_err^2);             % mild speed tracking
    end

    % Control effort penalty (small, symmetric)
    for i = 1:Nu
        thr   = U(1,i);
        steer = U(2,i);
        J = J + 0.1   * thr^2 ...
              + 0.001 * steer^2;
    end

    % Rate penalty (keep the old structure, just small weights)
    du1 = U(:,1) - last_u;
    J = J + 0.05 * du1(1)^2 + 0.0005 * du1(2)^2;

    for i = 2:Nu
        dui = U(:,i) - U(:,i-1);
        J = J + 0.02 * dui(1)^2 + 0.0002 * dui(2)^2;
    end

    if ~isfinite(J)
        J = 1e6;
    end
end

function [c, ceq] = mpc_constraints(u_vec, x0, scenario, Nu)
% MPC constraints (currently none)
    c   = [];     % Inequality constraints
    ceq = [];     % Equality constraints
end

%% ===================== ARTIFICIAL POTENTIAL FIELD =====================
function [apf_reference, desired_region] = compute_apf_reference(ego_state, tgt_state, scenario)

    % Persistent lock for side/region so the APF does not flip every step
    persistent locked_side locked_region is_locked
    if isempty(is_locked)
        is_locked     = false;
        locked_side   = 0;
        locked_region = '';
    end

    % ================= RELATIVE POSE =================
    dx = ego_state.x - tgt_state.x;
    dy = ego_state.y - tgt_state.y;
    
    % Target orientation
    cos_psi = cos(tgt_state.psi);
    sin_psi = sin(tgt_state.psi);
    
    % Ego in target frame
    x_rel = dx * cos_psi + dy * sin_psi;
    y_rel = -dx * sin_psi + dy * cos_psi;

    % Distance between ego and target (for locking)
    d_long = hypot(x_rel, y_rel);

        % ================= RELATIVE VELOCITY =================
    v_ego_global = [ego_state.v * cos(ego_state.psi + ego_state.beta);
                    ego_state.v * sin(ego_state.psi + ego_state.beta)];
    v_tgt_global = [tgt_state.v * cos(tgt_state.psi + tgt_state.beta);
                    tgt_state.v * sin(tgt_state.psi + tgt_state.beta)];
    v_rel_global = v_ego_global - v_tgt_global;

    % Relative velocity in target coordinates
    v_rel_target = [cos_psi,  sin_psi;
                   -sin_psi,  cos_psi] * v_rel_global;

    % ================= PDOF + ΔV2 for severity model =================
    [DeltaV2_target_apf, alpha_rad_apf] = ...
        compute_target_deltaV2_alpha(ego_state, tgt_state, scenario);

    % ================= SIDE SELECTION =================
    side_sign = sign(y_rel);
    if side_sign == 0
        side_sign = 1;
    end
    y_side = side_sign * (tgt_state.width / 2);   % physical side, no inflation

    % If we ever get far away longitudinally again, unlock for next encounter
    if d_long > 1.2 * scenario.apf.lock_distance
        is_locked     = false;
        locked_side   = 0;
        locked_region = '';
    end

    % ================= PREDICTED IMPACT X =================
    vy_rel = v_rel_target(2);
    use_prediction = true;

    if abs(vy_rel) < 1e-3
        % No meaningful lateral closing speed -> fall back
        use_prediction = false;
        x_impact_pred  = x_rel;
    else
        t_hit = (y_side - y_rel) / vy_rel;
        if t_hit <= 0
            % Moving away from the side line -> fall back
            use_prediction = false;
            x_impact_pred  = x_rel;
        else
            % Predicted x at side impact (in target frame)
            x_impact_pred  = x_rel + v_rel_target(1) * t_hit;
        end
    end

    % ================= REGION SELECTION =================
    region_names = {'front_region', 'front_door', 'b_pillar', 'back_door', 'rear_region'};
    
    current_region = determine_impact_region([x_rel, y_rel], scenario);
    if ~ismember(current_region, region_names)
        % clamp to nearest end if outside
        if x_rel >= 0
            current_region = 'front_region';
        else
            current_region = 'rear_region';
        end
    end

    desired_region = current_region;   % default

    % Candidate region from prediction
    if use_prediction
        w_severity  = 30.0;   % penalise high-severity zones
        w_proximity = 0.2;    % keep near predicted impact x

        best_cost   = Inf;
        best_region = current_region;

        for i = 1:length(region_names)
            name_i     = region_names{i};
            region_def = scenario.regions.(name_i);

            x_center = mean(region_def.range);  % region center in target x

            % FULL severity score: S_sev(ΔV2, R, α)
            S_sev_i = compute_severity_score(DeltaV2_target_apf, name_i, alpha_rad_apf, scenario);

            dist_term = (x_impact_pred - x_center)^2;

            % APF region cost = "how bad is this region" + "how far from predicted impact x"
            J_region = w_severity * S_sev_i + w_proximity * dist_term;

            tol = 1e-6;

            if (J_region < best_cost - tol)
                best_cost = J_region;
                best_region = name_i;
            
            elseif abs(J_region - best_cost) <= tol
                % tie-break: prefer rear-most region (more negative center x)
                x_center_new  = mean(scenario.regions.(name_i).range);
                x_center_best = mean(scenario.regions.(best_region).range);
            
                if x_center_new < x_center_best
                    best_region = name_i;
                end
            end
        end

        candidate_region = best_region;
    else
        candidate_region = current_region;
    end

    % ================= LOCK BEHAVIOUR =================
    if ~is_locked
        desired_region = candidate_region;
    
        if d_long <= scenario.apf.lock_distance
            is_locked   = true;
            locked_side = side_sign;   % lock only side
        end
    else
        side_sign      = locked_side;  % keep side stable
        desired_region = locked_region;  % region can still adapt
    end

    % Fallback if region is unknown / outside ranges
    if ~ismember(desired_region, region_names)
        if x_rel >= 0
            desired_region = 'front_region';
        else
            desired_region = 'rear_region';
        end
    end
    
    % ================= DEBUG PRINT =================
    if isfield(scenario,'apf') && isfield(scenario.apf,'debug') && scenario.apf.debug
        persistent apf_step last_print_region
        if isempty(apf_step)
            apf_step = 0;
            last_print_region = '';
        end
        apf_step = apf_step + 1;

        % only print when we're relatively close, otherwise you get spam
        if d_long < 20
            if use_prediction
                x_imp_dbg = x_impact_pred;
            else
                x_imp_dbg = NaN;
            end

            % only print every 3rd step OR when region changes
            if mod(apf_step,3)==0 || ~strcmp(desired_region,last_print_region)
                fprintf(['APF step %3d: d_long=%5.2f  x_rel=%6.2f  y_rel=%6.2f  ' ...
                         'x_pred=%6.2f  cur=%-12s  cand=%-12s  desired=%-12s  ' ...
                         'locked=%d  side=%+d\n'], ...
                        apf_step, d_long, x_rel, y_rel, x_imp_dbg, ...
                        current_region, candidate_region, desired_region, ...
                        is_locked, side_sign);
                last_print_region = desired_region;
            end
        end
    end
    
    if ~ismember(desired_region, region_names)
        if x_rel >= 0
            desired_region = 'front_region';
        else
            desired_region = 'rear_region';
        end
    end

    % ================= APF PATH GENERATION =================
    apf_reference = generate_apf_path(ego_state, tgt_state, desired_region, scenario, side_sign, ...
                          DeltaV2_target_apf, alpha_rad_apf);

    % (Optional) APF force, only for debug / plotting – not returned
    d_min = calculate_min_distance(ego_state, tgt_state);

    if ~isfinite(d_min) || d_min <= 0
        d_min = 0.1;  % clamp
    end
end

function reference_traj = generate_apf_path(ego_state, tgt_state, desired_region, scenario, side_sign, ...
                                DeltaV2_target_apf, alpha_rad_apf)

    N  = scenario.prediction_horizon;
    dt = scenario.dt;

    % Allocate output FIRST
    reference_traj = zeros(4, N);

    % === Front point of ego in global frame ===
    L_ego   = ego_state.length;
    psi_ego = ego_state.psi;

    x_front = ego_state.x + (L_ego/2)*cos(psi_ego);
    y_front = ego_state.y + (L_ego/2)*sin(psi_ego);

    % Transform that front point into TARGET coordinates
    dx = x_front - tgt_state.x;
    dy = y_front - tgt_state.y;

    cos_psi_t = cos(tgt_state.psi);
    sin_psi_t = sin(tgt_state.psi);

    % p = [x; y] in target frame (front point)
    p = [ dx*cos_psi_t + dy*sin_psi_t;
         -dx*sin_psi_t + dy*cos_psi_t ];

    % === Region centers ===
    region_names = {'front_region', 'front_door', 'b_pillar', 'back_door', 'rear_region',};
    nR = numel(region_names);
    x_c = zeros(1,nR);

    for i = 1:nR
        name_i = region_names{i};
        r_i    = scenario.regions.(name_i);
        x_c(i) = mean(r_i.range);
    end

    % Centre of chosen region
    idx_des = find(strcmp(region_names, desired_region), 1);
    if isempty(idx_des)
        idx_des = find(strcmp(region_names, 'rear_region'), 1);
    end
    x_des = x_c(idx_des);

    % Side line we want to sit on
    y_side = side_sign * (tgt_state.width/2);

    % Gains (keep your values)
    k_y   = 2.0;     % attraction to side line
    k_U   = 1.0;     % scale for base severity field

    % Longitudinal potential params (your fix)
    k_att_x = 2.5;
    sigma_x = 0.1;
    k_rep_x = 2.5;

    % ---- AEB-like speed reference profile for MPC/APF ----
    d_now = calculate_min_distance(ego_state, tgt_state);
    v0    = ego_state.v;

    if isfield(scenario,'aeb') && d_now < scenario.aeb.trigger_distance_m
        a_brake = abs(scenario.ego.a_long_min);   % e.g. 8 m/s^2
    else
        a_brake = 0.0;
    end

    % start CG reference from current ego CG
    x_cg_prev = ego_state.x;
    y_cg_prev = ego_state.y;

    for k = 1:N

        % Define v_ref INSIDE loop (and use it everywhere)
        v_ref = max(scenario.apf.v_ref_min, v0 - a_brake * ((k-1)*dt));
        reference_traj(4,k) = v_ref;

        x = p(1);
        y = p(2);

        % --- attraction to desired region center ---
        gradUx  = 2 * k_att_x * (x - x_des);

        % --- repulsion from dangerous regions ---
        danger_regions = {'b_pillar','front_door','back_door'};
        for jj = 1:numel(danger_regions)
            name_j = danger_regions{jj};
            idx_j  = find(strcmp(region_names, name_j), 1);
            if isempty(idx_j), continue; end

            xj = x_c(idx_j);

            S_j = compute_severity_score(DeltaV2_target_apf, name_j, alpha_rad_apf, scenario);
            rep_strength = k_rep_x * min(max(S_j, 0.0), 1.0);

            gradUx = gradUx + rep_strength * exp(-((x-xj)^2)/(sigma_x^2)) * (2*(x-xj)/(sigma_x^2));
        end

        % --- attraction to side line ---
        gradUy = 2 * k_y * (y - y_side);

        gradU = k_U * [gradUx; gradUy];

        if norm(gradU) < 1e-8
            step_dir = [0; 0];
        else
            step_dir = -gradU / norm(gradU);
        end

        step_len = v_ref * dt;
        p = p + step_len * step_dir;

        % back to global (front point)
        x_front_g = tgt_state.x + p(1)*cos_psi_t - p(2)*sin_psi_t;
        y_front_g = tgt_state.y + p(1)*sin_psi_t + p(2)*cos_psi_t;

        % heading = tangent of path (CG-based)
        heading = atan2(y_front_g - y_cg_prev, x_front_g - x_cg_prev);

        % CG reference behind front point
        x_cg_ref = x_front_g - (L_ego/2)*cos(heading);
        y_cg_ref = y_front_g - (L_ego/2)*sin(heading);

        reference_traj(1,k) = x_cg_ref;
        reference_traj(2,k) = y_cg_ref;
        reference_traj(3,k) = heading;

        x_cg_prev = x_cg_ref;
        y_cg_prev = y_cg_ref;
    end
end

function [region, side] = determine_impact_region(impact_point, scenario)
    x_rel = impact_point(1);
    y_rel = impact_point(2);

    % Determine region purely from x (along the side)
    region_names = fieldnames(scenario.regions);
    for i = 1:length(region_names)
        region_def = scenario.regions.(region_names{i});
        if x_rel >= region_def.range(1) && x_rel <= region_def.range(2)
            region = region_names{i};
            break;
        end
    end

    if ~exist('region', 'var')
        region = 'unknown';
    end

    % Determine side: convention: y_rel > 0 -> left, < 0 -> right
    if y_rel >= 0
        side = 'left';
    else
        side = 'right';
    end
end

%% ===================== COLLISION DETECTION & HELPER FUNCTIONS =====================
function tgt = update_target_kinematic(tgt, dt)
% Kinematic target: x,y propagate using constant v and heading psi.
    tgt.x = tgt.x + tgt.v * cos(tgt.psi) * dt;
    tgt.y = tgt.y + tgt.v * sin(tgt.psi) * dt;
end

function [collision, impact_point, vrel_mag, vrel_lat, vrel_long, ...
          dV2_mag, dV2_lat, alpha_rad] = check_collision_0(ego_state, tgt_state, scenario)

    % ---- DEFAULT OUTPUTS (no collision case) ----
    collision   = false;
    impact_point = [NaN NaN];

    vrel_mag  = 0;
    vrel_lat  = 0;
    vrel_long = 0;

    dV2_mag   = 0;
    dV2_lat   = 0;

    alpha_rad = 0;

    ego_poly = get_vehicle_polygon(ego_state);
    tgt_poly = get_vehicle_polygon(tgt_state);

    % --- robust polygon intersection using polyshape ---
    p_ego = polyshape(ego_poly(:,1), ego_poly(:,2));
    p_tgt = polyshape(tgt_poly(:,1), tgt_poly(:,2));
    inter_poly = intersect(p_ego, p_tgt);

    collision = ~isempty(inter_poly.Vertices);

    if ~collision
        return;
    end

    % Use centroid of actual overlap region as contact point (global frame)
    [cx, cy] = centroid(inter_poly);

    % Convert contact point to target-vehicle coordinates
    dx = cx - tgt_state.x;
    dy = cy - tgt_state.y;

    cos_psi = cos(tgt_state.psi);
    sin_psi = sin(tgt_state.psi);

    impact_x = dx * cos_psi + dy * sin_psi;
    impact_y = -dx * sin_psi + dy * cos_psi;

    % Clamp x within physical side length
    impact_x = max(-scenario.tgt.length/2, min(scenario.tgt.length/2, impact_x));
    % Clamp y exactly to side surface
    impact_y = sign(impact_y) * (tgt_state.width / 2);

    impact_point = [impact_x, impact_y];

    % ---- Approach speed (relative velocity) in target frame ----
    v_ego_global = [ego_state.v * cos(ego_state.psi + ego_state.beta);
                    ego_state.v * sin(ego_state.psi + ego_state.beta)];
    v_tgt_global = [tgt_state.v * cos(tgt_state.psi + tgt_state.beta);
                    tgt_state.v * sin(tgt_state.psi + tgt_state.beta)];

    v_rel_global = v_ego_global - v_tgt_global;

    cos_psi = cos(tgt_state.psi);
    sin_psi = sin(tgt_state.psi);
    R = [cos_psi, sin_psi; -sin_psi, cos_psi];      % global -> target
    v_rel_target = R * v_rel_global;

    vrel_long = abs(v_rel_target(1));
    vrel_lat  = abs(v_rel_target(2));
    vrel_mag  = norm(v_rel_target);

    % ---- Target delta-V proxy (inelastic momentum transfer), and angle ----
    [dV2_mag, dV2_lat, alpha_rad] = compute_target_deltaV2_alpha(ego_state, tgt_state, scenario);
end


function poly = get_vehicle_polygon(state)
    [corners_x, corners_y] = calculate_vehicle_corners(state);
    poly = [corners_x(1:end-1), corners_y(1:end-1)];
end

function [corners_x, corners_y] = calculate_vehicle_corners(state)
    half_length = state.length / 2;
    half_width = state.width / 2;
    
    local_corners = [
        half_length,  half_width;
        half_length, -half_width;  
        -half_length, -half_width;
        -half_length,  half_width
    ];
    
    cos_psi = cos(state.psi);
    sin_psi = sin(state.psi);
    
    corners_x = state.x + local_corners(:,1) * cos_psi - local_corners(:,2) * sin_psi;
    corners_y = state.y + local_corners(:,1) * sin_psi + local_corners(:,2) * cos_psi;
    
    corners_x = [corners_x; corners_x(1)];
    corners_y = [corners_y; corners_y(1)];
end

function d_min = calculate_min_distance(ego_state, tgt_state)
    ego_poly = get_vehicle_polygon(ego_state);
    tgt_poly = get_vehicle_polygon(tgt_state);
    
    % Simple bounding box distance
    ego_bb = [min(ego_poly(:,1)), min(ego_poly(:,2)), max(ego_poly(:,1)), max(ego_poly(:,2))];
    tgt_bb = [min(tgt_poly(:,1)), min(tgt_poly(:,2)), max(tgt_poly(:,1)), max(tgt_poly(:,2))];
    
    dx = max(0, max(ego_bb(1)-tgt_bb(3), tgt_bb(1)-ego_bb(3)));
    dy = max(0, max(ego_bb(2)-tgt_bb(4), tgt_bb(2)-ego_bb(4)));
    
    d_min = sqrt(dx^2 + dy^2);
end

function [DeltaV2_mag, DeltaV2_lat, alpha_clock_rad, vrel_target, dV2_target] = ...
    compute_target_deltaV2_alpha(ego_state, tgt_state, scenario)
% Proper meanings:
%   vrel_target  = relative velocity at impact in TARGET frame  [m/s]  (approach speed)
%   dV2_target   = target delta-V proxy vector (inelastic) in TARGET frame [m/s]
%   DeltaV2_lat  = abs(lateral component of dV2_target)         [m/s]
%   DeltaV2_mag  = magnitude of dV2_target                      [m/s]
%   alpha_clock  = clock angle of dV2_target (0=front, 90=right, 180=rear, 270=left)

    % --- Global velocities (CG) ---
    psi1 = ego_state.psi + ego_state.beta;
    psi2 = tgt_state.psi + tgt_state.beta;

    v1 = max(0, ego_state.v);
    v2 = max(0, tgt_state.v);

    v1_g = [v1*cos(psi1); v1*sin(psi1)];
    v2_g = [v2*cos(psi2); v2*sin(psi2)];

    % --- Relative velocity (approach speed) ---
    vrel_g = v1_g - v2_g;

    % --- Rotate global -> target frame ---
    c = cos(tgt_state.psi);
    s = sin(tgt_state.psi);
    R_gt = [ c,  s;
            -s,  c];          % global -> target

    vrel_target = R_gt * vrel_g;      % [vx; vy] in target frame

    % --- Inelastic momentum-transfer proxy for target delta-V vector ---
    m1 = scenario.ego.mass;
    m2 = scenario.tgt.mass;

    % If perfectly inelastic (stick), target delta-V is proportional to relative velocity
    dV2_target = (m1/(m1+m2)) * vrel_target;   % vector in target frame

    DeltaV2_mag = norm(dV2_target);
    DeltaV2_lat = abs(dV2_target(2));          % lateral component (near-side relevant)

    % --- Clock angle based on delta-V vector, using your convention ---
    % your convention: 90°=right, 270°=left -> use atan2(-vy, vx)
    alpha_clock_rad = mod(atan2(-dV2_target(2), dV2_target(1)), 2*pi);
end

function p = pelvis_AIS3_prob_from_logit(DeltaV2_mps, isAngle10, scenario)
% Uses pelvis paper logistic model: p = 1/(1+exp(-(b0 + b_lnV*ln(V) + b_angle10*isAngle10)))
% IMPORTANT: paper uses lateral delta-V; here we approximate using your DeltaV2_target.
% Units: use kph because your extracted curve points are in kph. :contentReference[oaicite:1]{index=1}

    m = scenario.severity.pelvis_logit;

    V_kph = max(DeltaV2_mps * 3.6, m.minV_kph);   % clamp for ln
    z = m.b0 + m.b_lnV * log(V_kph) + m.b_angle10 * double(isAngle10);

    p = 1 / (1 + exp(-z));
    p = max(0, min(1, p));
end

%% ===================== SEVERITY HELPER FUNCTIONS ===================  

function p_base = compute_baseline_severity_deltaV(DeltaV2_mps, scenario)
% Compute MAIS3+ thorax injury probability from ΔV2 and age using the
% logistic regression from "Characteristics of Crashes that Increase the
% Risk of Serious Injuries" (near-side, thorax AIS3+):
%
%   w = β0 + β1 * ΔV_mph + β2 * age_years
%   p = 1 / (1 + exp(-w))
%
% where:
%   - ΔV_mph is Delta-V in mph (the paper uses mph, not m/s),
%   - age_years is frozen at scenario.severity.age_ref (35 y).
%
% INPUT:
%   DeltaV2_mps : scalar or vector, ΔV2 of the TARGET [m/s]
%   scenario    : scenario struct with severity.thorax_MAIS3 fields

    coeff = scenario.severity.thorax_MAIS3;
    age   = scenario.severity.age_ref;       % 35 years

    % Convert ΔV from m/s to mph
    DeltaV_mph = DeltaV2_mps * 2.23694;

    % Linear predictor
    w = coeff.beta0 + coeff.beta_dv * DeltaV_mph + coeff.beta_age * age;

    % Logistic transform to probability (0–1)
    p_base = 1 ./ (1 + exp(-w));

    % Numerical safety
    p_base = max(0, min(1, p_base));
end

function S = compute_severity_score(dV2_lat, region_name, alpha_clock_rad, scenario)

    % Base term: pelvis MAIS3+ probability uses lateral delta-V (proxy)
    P_ref = pelvis_AIS3_prob_from_logit(dV2_lat, false, scenario);

    % Region weight λR
    if isfield(scenario.regions, region_name)
        lambda_R = scenario.regions.(region_name).weight;
    else
        lambda_R = 1.0;
    end

    % Angle weight λA(ΔV,α) (also lateral-based)
    lambda_A = compute_angle_weight_dv(dV2_lat, alpha_clock_rad, scenario);

    S = 10*(lambda_R * lambda_A * P_ref);
end

function lambda_A = compute_angle_weight_dv(DeltaV2_mps, alpha_clock_rad, scenario)
% Returns λA(ΔV,angle) = P(ΔV,angle) / P(ΔV,9h)
% Angle classification: map your clock angle to “9h band” or “10h band”.
% You can keep your bands or tighten them.

    a_deg = mod(rad2deg(alpha_clock_rad), 360);

    % Define your bands (tune boundaries if needed)
    band9  = [255 285];  % around 270°
    band10 = [285 315];  % around 300°

    in9  = (a_deg >= band9(1)  && a_deg < band9(2));
    in10 = (a_deg >= band10(1) && a_deg < band10(2));

    % Reference probability at 9 o'clock
    p9 = pelvis_AIS3_prob_from_logit(DeltaV2_mps, false, scenario);

    if in10
        pA = pelvis_AIS3_prob_from_logit(DeltaV2_mps, true, scenario);
    elseif in9
        pA = p9;
    else
        % Outside: neutral (don’t invent behaviour)
        lambda_A = 0.8;
        return;
    end

    % Ratio in probability space (ΔV-dependent). Clamp to avoid insane ratios at tiny p9.
    eps = 1e-6;
    lambda_A = pA / max(p9, eps);
    lambda_A = min(max(lambda_A, 0.2), 10.0);   % safety clamp
end

%% ===================== VISUALIZATION & RESULTS =====================
function plot_apf_scenario(ego_state, tgt_state, tgt_trajectory, scenario, ...
                          current_time, mode_label, impact_region, deltaV, apf_reference)

    clf;

    % ---------------- FIX: define use_apf_mpc + controller_type ----------------
    use_apf_mpc = false;

    if nargin < 6 || isempty(mode_label)
        controller_type = 'Uncontrolled';

    elseif islogical(mode_label) || (isnumeric(mode_label) && isscalar(mode_label))
        use_apf_mpc = logical(mode_label);
        if use_apf_mpc
            controller_type = 'APF+MPC';
        else
            controller_type = 'Uncontrolled';
        end

    else
        controller_type = char(mode_label);   % e.g. 'AEB only'
        % keep APF/MPC panel OFF unless label clearly indicates it
        low = lower(controller_type);
        use_apf_mpc = contains(low,'apf') || contains(low,'mpc') || contains(low,'controlled');
    end
    % -------------------------------------------------------------------------

    % Main trajectory + potential field
    subplot(2,3,[1,2,4,5]);
    cla; hold on; axis equal;

    % Plot road
    plot_road(scenario);

    % Draw potential field
    if ~isempty(apf_reference)
        plot_potential_field(tgt_state, ego_state, scenario, impact_region);
    end

    % Plot target trajectory
    tgt_x = [tgt_trajectory.x];
    tgt_y = [tgt_trajectory.y];
    plot(tgt_x, tgt_y, 'r--', 'LineWidth', 1, 'DisplayName', 'Target Path');

    % Plot APF reference if available
    if ~isempty(apf_reference)
        L_ego   = scenario.ego.length;
        x_cg    = apf_reference(1,:);
        y_cg    = apf_reference(2,:);
        psi_ref = apf_reference(3,:);

        x_front = x_cg + (L_ego/2)*cos(psi_ref);
        y_front = y_cg + (L_ego/2)*sin(psi_ref);

        plot(x_front, y_front, 'g--', 'LineWidth', 2, 'DisplayName', 'APF Ref (front)');
    end

    % Plot vehicles
    plot_vehicle_with_regions(ego_state, scenario.ego, 'ego', 'Ego Vehicle', scenario.regions);
    plot_vehicle_with_regions(tgt_state, scenario.tgt, 'target', 'Target Vehicle', scenario.regions);

    title(sprintf('%s Scenario - Time: %.2fs', controller_type, current_time), ...
          'FontSize', 12, 'FontWeight', 'bold');

    info_text = sprintf('Region: %s\nv_{rel,lat}: %.2f m/s', impact_region, deltaV);
    text(-18, 15, info_text, 'HorizontalAlignment', 'left', 'BackgroundColor', 'white', ...
         'FontSize', 9, 'EdgeColor', 'black', 'Margin', 2);

    xlabel('X [m]'); ylabel('Y [m]');
    legend('Location', 'southwest');
    xlim([-18, 18]); ylim([-18, 18]);

    % Control inputs panel
    subplot(2,3,3);
    if use_apf_mpc
        yyaxis left;
        plot(1, ego_state.throttle, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
        ylabel('Throttle [-]');
        ylim([scenario.ego.max_brake-1, scenario.ego.max_throttle+1]);
        grid on;

        yyaxis right;
        plot(1, rad2deg(ego_state.steering), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
        ylabel('Steer [deg]');
        ylim(rad2deg([-scenario.ego.max_steer-0.1, scenario.ego.max_steer+0.1]));
        title('Control Inputs');
    else
        text(0.5, 0.5, 'No Control Active', 'HorizontalAlignment', 'center', ...
             'FontSize', 12, 'Units', 'normalized', 'FontWeight', 'bold');
        axis off;
    end

    % Status panel
    subplot(2,3,6);
    axis off;
    status_text = sprintf(['Controller: %s\n' ...
                          'Target Region: %s\n' ...
                          'v_{rel,lat}: %.2f m/s\n' ...
                          'Time: %.2f s'], ...
                         controller_type, impact_region, deltaV, current_time);
    text(0.1, 0.7, status_text, 'FontSize', 9, 'Units', 'normalized', ...
         'VerticalAlignment', 'top', 'BackgroundColor', [0.95 0.95 0.95]);

    set(gcf, 'Color', 'white');
    drawnow;
end

function plot_road(scenario)
    main_road_width = scenario.road.width;
    main_road_length = scenario.road.length;
    
    % Main road (horizontal)
    main_road_x = [-main_road_length/2, main_road_length/2, main_road_length/2, -main_road_length/2];
    main_road_y = [-main_road_width/2, -main_road_width/2, main_road_width/2, main_road_width/2];
    fill(main_road_x, main_road_y, [0.8 0.8 0.8], 'EdgeColor', 'k', 'LineWidth', 1, 'HandleVisibility', 'off');
    
    % Cross road (vertical)
    cross_road_x = [-main_road_width/2, -main_road_width/2, main_road_width/2, main_road_width/2];
    cross_road_y = [-main_road_length/2, main_road_length/2, main_road_length/2, -main_road_length/2];
    fill(cross_road_x, cross_road_y, [0.8 0.8 0.8], 'EdgeColor', 'k', 'LineWidth', 1, 'HandleVisibility', 'off');
    
    % Center lines
    plot([-main_road_length/2, main_road_length/2], [0, 0], 'y-', 'LineWidth', 2, 'HandleVisibility', 'off');
    plot([0, 0], [-main_road_length/2, main_road_length/2], 'y-', 'LineWidth', 2, 'HandleVisibility', 'off');
end

function plot_potential_field(tgt_state, ego_state, scenario, ~)
% VISUALIZE FULL FIELD (Original Style)
% Plots the potential landscape for ALL regions simultaneously.
% - Danger Regions = Positive Peaks (Hills)
% - Safe Regions   = Negative Wells (Valleys)

    % ----- 1. Grid in target frame (Original Setup) -----
    R = 2.5;      % Range [m]
    N = 45;       % Grid resolution
    xr = linspace(-(R+4), (R+4), N);
    yr = linspace(-(R+4), (R+4), N);
    [XR, YR] = meshgrid(xr, yr);

    % Target orientation
    cos_psi = cos(tgt_state.psi);
    sin_psi = sin(tgt_state.psi);

    % Ego position in target frame (to know which side we are on)
    dx = ego_state.x - tgt_state.x;
    dy = ego_state.y - tgt_state.y;
    y_rel = -dx*sin_psi + dy*cos_psi;

    side_sign = sign(y_rel);
    if side_sign == 0, side_sign = 1; end

    W_half = scenario.tgt.width/2;
    y_side = side_sign * W_half;

    % ----- 2. Recompute ΔV2 and α (Original Math) -----
    [DeltaV2_target_pf, alpha_rad_pf] = compute_target_deltaV2_alpha(ego_state, tgt_state, scenario);

    % ----- 3. Parameters (Original Gains) -----
    k_y     = 2.0;
    sigma_x = 0.1;
    k_rep_x = 2.5;  % Strength of repulsion hills
    k_att_x = 2.5;  % Strength of attraction wells

    % ----- 4. Build U(XR,YR) by summing ALL regions -----
    % Initialize with Lateral Potential (Wall along the side)
    U = k_y * (YR - y_side).^2;

    region_names = fieldnames(scenario.regions);

    for i = 1:numel(region_names)
        r_name = region_names{i};
        r_def  = scenario.regions.(r_name);
        x_c    = mean(r_def.range);

        % Calculate Severity S for this region
        S_j = compute_severity_score(DeltaV2_target_pf, r_name, alpha_rad_pf, scenario);

        % LOGIC:
        % If S is HIGH (> 0.3) -> It's a Hill (Repulsion)
        % If S is LOW  (<= 0.3) -> It's a Valley (Attraction)

        if r_def.weight > 0.3
            % --- DANGER (Positive Gaussian) ---
            strength = k_rep_x * min(max(S_j, 0.5), 2.0); % Scale height by severity
            U = U + strength .* exp(-((XR - x_c).^2) ./ (sigma_x^2));
        else
            % --- SAFE (Negative Gaussian / Well) ---
            % This creates the "Blue Valley" effect for safe spots
            strength = k_att_x; 
            U = U - strength .* exp(-((XR - x_c).^2) ./ (0.5^2)); 
        end
    end

    % ----- 5. Plotting (Original Style) -----
    % Shift and clamp for nice gradients
    % We clamp the max value so the "mountains" don't wash out the color scale.
    finiteU = U(isfinite(U));
    if ~isempty(finiteU)
        Umax  = prctile(finiteU, 85);
        Umin  = min(finiteU);
        Uplot = max(Umin, min(U, Umax));
    else
        Uplot = U;
    end

    % Map grid back to global coordinates
    Xg = tgt_state.x + XR*cos_psi - YR*sin_psi;
    Yg = tgt_state.y + XR*sin_psi + YR*cos_psi;

    [~, h] = contourf(Xg, Yg, Uplot, 30, 'LineStyle', 'none');
    set(h, 'HandleVisibility', 'off');
    set(h, 'FaceAlpha', 0.7);

    colormap(parula);
    cb = colorbar('Location','eastoutside');
    cb.Label.String = 'Potential U';

    % ----- 6. Angle Wedge Overlay (Original Code) -----
    if isfield(scenario,'severity') && isfield(scenario.severity,'angle_band')
        ab = scenario.severity.angle_band;

        r_inner = 0.0;
        r_outer = R;

        ang1 = deg2rad(ab.deg_med1);    
        ang2 = deg2rad(ab.deg_high2);   

        nTheta = 40;
        theta = linspace(ang1, ang2, nTheta);

        x_in  = r_inner * cos(theta);
        y_in  = r_inner * sin(theta);
        x_out = r_outer * cos(fliplr(theta));
        y_out = r_outer * sin(fliplr(theta));

        x_wedge_t = [x_in,  x_out];
        y_wedge_t = [y_in,  y_out];

        x_wedge_g = tgt_state.x + x_wedge_t * cos_psi - y_wedge_t * sin_psi;
        y_wedge_g = tgt_state.y + x_wedge_t * sin_psi + y_wedge_t * cos_psi;

        patch(x_wedge_g, y_wedge_g, [1 0 0], ...
              'FaceAlpha', 0.15, 'EdgeColor', 'r', 'LineStyle', '--', ...
              'LineWidth', 1.0, 'HandleVisibility','off');
    end
end

% function plot_potential_field(tgt_state, ego_state, scenario, impact_region)
% % Visualise the same potential used by generate_apf_path (in TARGET frame),
% % then map it back to global coordinates.
% 
%     % ----- Grid in target frame -----
%     R = 2.5;      % [m] half-range around target (used also for wedge radius)
%     N = 41;     % grid resolution
%     xr = linspace(-(R+4), (R+4), N);
%     yr = linspace(-(R+4), (R+4), N);
%     [XR, YR] = meshgrid(xr, yr);
% 
%     % Target orientation
%     cos_psi = cos(tgt_state.psi);
%     sin_psi = sin(tgt_state.psi);
% 
%     % Ego position in target frame (to know which side we are on)
%     dx = ego_state.x - tgt_state.x;
%     dy = ego_state.y - tgt_state.y;
%     x_rel = dx*cos_psi + dy*sin_psi;
%     y_rel = -dx*sin_psi + dy*cos_psi;
% 
%     side_sign = sign(y_rel);
%     if side_sign == 0, side_sign = 1; end
% 
%     W_half = scenario.tgt.width/2;
%     y_side = side_sign * W_half;
% 
%     % ----- Region centers -----
%     region_names = {'front_region', 'front_door', 'b_pillar', 'back_door', 'rear_region'};
%     nR = numel(region_names);
%     x_c = zeros(1,nR);
%     for i = 1:nR
%         r_i    = scenario.regions.(region_names{i});
%         x_c(i) = mean(r_i.range);
%     end
% 
%     % Recompute ΔV2 and α at this instant (same as APF)
%     [DeltaV2_target_pf, alpha_rad_pf] = compute_target_deltaV2_alpha(ego_state, tgt_state, scenario);
% 
%     % ----- Gains must match generate_apf_path -----
%     k_y     = 2.0;
%     k_U     = 1.0;
%     k_att_x = 2.5;
%     sigma_x = 0.1;
%     k_rep_x = 2.5;
% 
%     % Determine x_des from impact_region being displayed
%     idx_des = find(strcmp(region_names, impact_region), 1);
%     if isempty(idx_des)
%         idx_des = find(strcmp(region_names, 'rear_region'), 1);
%     end
%     x_des = x_c(idx_des);
% 
%     % ----- Build U(XR,YR) in target frame -----
%     Ux_att = k_att_x * (XR - x_des).^2;
% 
%     % Repulsion from dangerous regions
%     danger_regions = {'b_pillar','front_door','back_door'};
%     Ux_rep = zeros(size(XR));
%     for jj = 1:numel(danger_regions)
%         name_j = danger_regions{jj};
%         idx_j  = find(strcmp(region_names, name_j), 1);
%         if isempty(idx_j), continue; end
% 
%         xj = x_c(idx_j);
% 
%         S_j = compute_severity_score(DeltaV2_target_pf, name_j, alpha_rad_pf, scenario);
%         rep_strength = k_rep_x * min(max(S_j, 0.0), 1.0);
% 
%         Ux_rep = Ux_rep + rep_strength .* exp(-((XR - xj).^2) ./ (sigma_x^2));
%     end
% 
%     Uy = k_y * (YR - y_side).^2;
% 
%     U = k_U * (Ux_att + Ux_rep) + Uy;
% 
%     % Shift and clamp for nicer plotting
%     U = U - min(U(:));
%     finiteU = U(isfinite(U));
%     if ~isempty(finiteU)
%         Umax  = prctile(finiteU, 80);
%         Uplot = min(U, Umax);
%     else
%         Uplot = U;
%     end
% 
%     % ----- Optional: danger angle wedge overlay -----
%     if isfield(scenario,'severity') && isfield(scenario.severity,'angle_band')
%         ab = scenario.severity.angle_band;
% 
%         r_inner = 0.0;
%         r_outer = R;
% 
%         % NOTE: if your clock-angle convention flips Y, this wedge may look mirrored.
%         ang1 = deg2rad(ab.deg_med1);    % 265°
%         ang2 = deg2rad(ab.deg_high2);   % 305°
% 
%         nTheta = 40;
%         theta = linspace(ang1, ang2, nTheta);
% 
%         x_in  = r_inner * cos(theta);
%         y_in  = r_inner * sin(theta);
%         x_out = r_outer * cos(fliplr(theta));
%         y_out = r_outer * sin(fliplr(theta));
% 
%         x_wedge_t = [x_in,  x_out];
%         y_wedge_t = [y_in,  y_out];
% 
%         x_wedge_g = tgt_state.x + x_wedge_t * cos_psi - y_wedge_t * sin_psi;
%         y_wedge_g = tgt_state.y + x_wedge_t * sin_psi + y_wedge_t * cos_psi;
% 
%         patch(x_wedge_g, y_wedge_g, [1 0 0], ...
%               'FaceAlpha', 0.15, 'EdgeColor', 'r', 'LineStyle', '--', ...
%               'LineWidth', 1.0, 'HandleVisibility','off');
%     end
% 
%     % ----- Map grid back to global coordinates -----
%     Xg = tgt_state.x + XR*cos_psi - YR*sin_psi;
%     Yg = tgt_state.y + XR*sin_psi + YR*cos_psi;
% 
%     [~, h] = contourf(Xg, Yg, Uplot, 20, 'LineStyle', 'none');
%     set(h, 'HandleVisibility', 'off');
%     set(h, 'FaceAlpha', 0.7);
%     colormap(parula);
% 
%     cb = colorbar('Location','eastoutside');
%     cb.Label.String = 'Side-impact potential U';
% end


function plot_vehicle_with_regions(state, vehicle_params, vehicle_type, display_name, regions)
    [corners_x, corners_y] = calculate_vehicle_corners(state);
    
    if strcmp(vehicle_type, 'ego')
        color = 'b';
    else
        color = 'r';
    end
    
    plot(corners_x, corners_y, [color '-'], 'LineWidth', 2, 'DisplayName', display_name);
    
    % Plot heading with sideslip
    actual_heading = state.psi + state.beta;
    quiver(state.x, state.y, cos(actual_heading), sin(actual_heading), ...
           2, 'Color', color, 'LineWidth', 2, 'MaxHeadSize', 1);
    
    % Plot regions for target vehicle
    if strcmp(vehicle_type, 'target')
        plot_impact_regions(state, regions);
    end
end

function plot_impact_regions(state, regions)
    % Draw side-perimeter segments for the regions on BOTH sides.

    side_region_names = {'front_region', 'front_door', 'b_pillar', 'back_door', 'rear_region'};

    half_width = state.width / 2;
    cos_psi = cos(state.psi);
    sin_psi = sin(state.psi);

    for i = 1:length(side_region_names)
        name = side_region_names{i};
        if ~isfield(regions, name)
            continue;
        end

        region = regions.(name);
        x1 = region.range(1);
        x2 = region.range(2);

        switch name
            case 'front_region'
                color = [0 0 0];      % black
            case 'front_door'
                color = [1 0 0];      % red
            case 'b_pillar'
                color = [1 1 0];      % yellow
            case 'back_door'
                color = [1 0 0];      % red
            case 'rear_region'
                color = [0 0 0];      % black
            otherwise
                color = [0.5 0.5 0.5];
        end

        % Draw both sides: y_local = ±half_width
        for side = [-1, 1]
            y_local = side * half_width;

            % Local → global for both endpoints
            p1x = state.x + x1 * cos_psi - y_local * sin_psi;
            p1y = state.y + x1 * sin_psi + y_local * cos_psi;
            p2x = state.x + x2 * cos_psi - y_local * sin_psi;
            p2y = state.y + x2 * sin_psi + y_local * cos_psi;

            plot([p1x p2x], [p1y p2y], '-', 'Color', color, 'LineWidth', 3, ...
                 'HandleVisibility', 'off');
        end
    end
end

function compare_three_case_results(res_un, res_aeb, res_ctrl, scenario, ang)
% Produces:
%  - Trajectory overlay
%  - Speed profiles
%  - Control timelines (throttle + steering)
%  - Summary panel (Region, ΔV2, S)

    figure('Name',sprintf('3-Case Results (angle %+d°)', ang), 'Color','w', 'Position',[80 80 1300 750]);
    tiledlayout(2,3,'Padding','compact','TileSpacing','compact');

    % ---------- (1) Trajectories ----------
    nexttile;
    hold on; grid on; axis equal;
    title('Trajectories (Ego + Target)');
    xlabel('X [m]'); ylabel('Y [m]');

    plot(res_un.ego_traj(1,:),  res_un.ego_traj(2,:),  'LineWidth', 2, 'DisplayName','Ego - Uncontrolled');
    plot(res_aeb.ego_traj(1,:), res_aeb.ego_traj(2,:), 'LineWidth', 2, 'DisplayName','Ego - AEB');
    plot(res_ctrl.ego_traj(1,:),res_ctrl.ego_traj(2,:), 'LineWidth', 2, 'DisplayName','Ego - Controlled');

    tgt_x = [res_un.tgt_trajectory.x];
    tgt_y = [res_un.tgt_trajectory.y];
    plot(tgt_x, tgt_y, 'LineWidth', 2, 'DisplayName','Target');

    legend('Location','best');

    % ---------- (2) Speed profiles ----------
    nexttile;
    hold on; grid on;
    title('Ego speed v(t)');
    xlabel('Time [s]'); ylabel('v [m/s]');
    plot(res_un.time,  res_un.ego_traj(4,:),  'LineWidth', 2, 'DisplayName','Uncontrolled');
    plot(res_aeb.time, res_aeb.ego_traj(4,:), 'LineWidth', 2, 'DisplayName','AEB');
    plot(res_ctrl.time,res_ctrl.ego_traj(4,:), 'LineWidth', 2, 'DisplayName','Controlled');
    legend('Location','best');

    % ---------- (3) Throttle timelines ----------
    nexttile;
    hold on; grid on;
    title('Throttle u_{thr}(t)');
    xlabel('Time step'); ylabel('Throttle [-]');
    plot(res_un.ego_traj(7,:),  'LineWidth', 2, 'DisplayName','Uncontrolled');
    plot(res_aeb.ego_traj(7,:), 'LineWidth', 2, 'DisplayName','AEB');
    plot(res_ctrl.ego_traj(7,:), 'LineWidth', 2, 'DisplayName','Controlled');
    yline(scenario.ego.max_throttle,'--','HandleVisibility','off');
    yline(scenario.ego.max_brake,'--','HandleVisibility','off');
    legend('Location','best');

    % ---------- (4) Steering timelines ----------
    nexttile;
    hold on; grid on;
    title('Steering \delta(t)');
    xlabel('Time step'); ylabel('\delta [deg]');
    plot(rad2deg(res_un.ego_traj(8,:)),  'LineWidth', 2, 'DisplayName','Uncontrolled');
    plot(rad2deg(res_aeb.ego_traj(8,:)), 'LineWidth', 2, 'DisplayName','AEB');
    plot(rad2deg(res_ctrl.ego_traj(8,:)), 'LineWidth', 2, 'DisplayName','Controlled');
    yline(rad2deg(scenario.ego.max_steer),'--','HandleVisibility','off');
    yline(-rad2deg(scenario.ego.max_steer),'--','HandleVisibility','off');
    legend('Location','best');

    % ---------- (5) Summary panel ----------
    nexttile([1 2]);
    axis off;
    title('Impact metrics at collision (if any)');

    rows = {
        'Uncontrolled', res_un;
        'AEB only',     res_aeb;
        'Controlled',   res_ctrl
    };

    % Build a table for workspace + show nicely
    Case = strings(3,1);
    Collision = false(3,1);
    Region = strings(3,1);
    dV2_lat_target = nan(3,1);   % momentum-transfer lateral delta-V proxy
    vrel_lat = nan(3,1);         % approach lateral speed
    Alpha_deg = nan(3,1);
    S = nan(3,1);

    y = 0.9;
    dy = 0.09;

    text(0.02, y, sprintf('Angle: %+d deg', ang), 'Units','normalized', 'FontSize', 12, 'FontWeight','bold');
    y = y - dy*0.8;

    text(0.02, y, 'Case', 'Units','normalized','FontWeight','bold');
    text(0.30, y, 'Collision', 'Units','normalized','FontWeight','bold');
    text(0.48, y, 'Region', 'Units','normalized','FontWeight','bold');
    text(0.60, y, '\DeltaV2_{lat} [m/s]', 'Units','normalized','FontWeight','bold');
    text(0.78, y, 'v_{rel,lat} [m/s]', 'Units','normalized','FontWeight','bold');
    text(0.92, y, 'S', 'Units','normalized','FontWeight','bold');
    y = y - dy*0.7;

    for i = 1:3
        label = rows{i,1};
        r = rows{i,2};

        Case(i) = string(label);
        Collision(i) = isfield(r,'collision_detected') && r.collision_detected;

        if Collision(i)
            Region(i)    = string(r.final_region);
            dV2_lat_target(i) = r.final_dV2_lat_target;
            vrel_lat(i)       = r.final_vrel_lat;
            Alpha_deg(i)      = r.final_alpha_deg;
            S(i)              = r.final_severity_S;

            coll_str = "YES";
        else
            Region(i) = "no_collision";
            coll_str = "NO";
        end

        text(0.02, y, label, 'Units','normalized');
        text(0.32, y, coll_str, 'Units','normalized');
        text(0.48, y, char(Region(i)), 'Units','normalized');
        text(0.60, y, sprintf('%.2f', dV2_lat_target(i)), 'Units','normalized');
        text(0.78, y, sprintf('%.2f', vrel_lat(i)), 'Units','normalized');
        text(0.92, y, sprintf('%.4f', S(i)), 'Units','normalized');
        y = y - dy;
    end

    ResultsTable = table(Case, Collision, Region, dV2_lat_target, vrel_lat, Alpha_deg, S);
    assignin('base','results_table_3cases', ResultsTable);

    disp('=== 3-CASE RESULTS TABLE ===');
    disp(ResultsTable);
end

%% ===================== TABLE HELPERS =====================
function s = get_coll_str(res)
    if res.collision_detected
        s = res.final_region;
    else
        s = 'AVOIDED';
    end
end

function score = get_severity(res)
    if res.collision_detected
        score = res.final_severity_S;
    else
        score = 0;
    end
end

function dist = get_start_dist_from_map(MapData, ve, vt, ang)
    % Extracts critical distance from the loaded map structure
    
    E_grid = MapData.ego_speeds;
    T_grid = MapData.tgt_speeds;
    A_grid = MapData.angles_deg;
    D_grid = MapData.crit_dist;
    
    % Handle Scalar vs Vector dimensions (MATLAB drops dims of size 1)
    if isscalar(E_grid), E_grid = [E_grid]; end
    if isscalar(T_grid), T_grid = [T_grid]; end
    
    % Check bounds
    if ang < min(A_grid) || ang > max(A_grid)
        dist = NaN; return;
    end
    
    % Robust Lookup (Nearest Neighbor)
    try
        if numel(E_grid) == 1 && numel(T_grid) == 1
            % Simple 1D interpolation if speeds are fixed
            dist = interp1(A_grid, squeeze(D_grid), ang, 'nearest');
        else
            % Full 3D interpolation
            dist = interpn(E_grid, T_grid, A_grid, D_grid, ve, vt, ang, 'nearest');
        end
    catch
        % Fallback for edge cases
        [~, idx] = min(abs(A_grid - ang));
        dist = D_grid(idx); 
    end
end

function dv = get_deltaV(res)
    if res.collision_detected
        % Using dV2_lat_target (Severity Proxy)
        dv = res.final_dV2_lat_target;
    else
        dv = 0;
    end
end
