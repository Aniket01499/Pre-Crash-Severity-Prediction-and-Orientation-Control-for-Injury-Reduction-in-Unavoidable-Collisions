%% Scenario_validation1.m
% Offline tool to build a map of UNAVOIDABLE side-impact scenarios.
%
% LOGIC:
% 1. Uses 'Strategy Sweep' (Full Brake + various steering angles).
% 2. Geometry Aware: Uses full polygon intersection.
% 3. Definition: Impacts within 0.25m of the target's front/rear ends 
%    are considered "Avoided/Glancing". Central hits are "Critical".
% 4. BUFFER: Applies a 0.5m conservative buffer (User requested reduction).
%
% Output: unavoidable_map.mat

clear; clc;

%% ===== base scenario definition =====
scenario_base = setup_scenario_for_scan();

% Scan Parameters (SYNCED WITH MAIN SIM)
ego_speeds = 20.84;       % [m/s] 13.89, 22.22, 30.56 - [K/h] 50, 80, 110
tgt_speeds = 16.67;       % [m/s]

% Scan Angles
angles_deg = -15;  % -30:5:10

nE = numel(ego_speeds);
nT = numel(tgt_speeds);
nA = numel(angles_deg);

crit_dist = nan(nE, nT, nA); 

fprintf('=== Scanning unavoidable side-impact map (High-Res Sweep) ===\n');
fprintf('    Target X0: %.1f m (Fixed)\n', scenario_base.tgt.x0);
fprintf('    Definition: Impacts within 0.25m of ends are AVOIDED.\n');
fprintf('    Buffer: Subtracting 0.2m from critical boundary.\n\n');

for ie = 1:nE
    for it = 1:nT
        for ia = 1:nA

            scen = setup_scenario_for_scan();

            % override speeds + heading
            scen.ego.v0           = ego_speeds(ie);
            scen.tgt.v0           = tgt_speeds(it);
            scen.tgt.heading0_rad = deg2rad(angles_deg(ia));
            scen.ego.max_steer    = deg2rad(35); 
            scen.mpc.steer_rate_max = steer_rate_from_speed(scen.ego.v0);
            
            fprintf('[%d/%d] Angle=%+3.0f° ... ', ia, nA, angles_deg(ia));

            % Find raw critical distance (The absolute edge of chaos)
            d_raw = find_critical_unavoidable_distance(scen);
            
            % --- APPLY CONSERVATIVE BUFFER (0.2m) ---
            if ~isnan(d_raw)
                d_final = max(4.0, d_raw - 0.2); 
                crit_dist(ie,it,ia) = d_final;
                fprintf('Unavoidable > %.2f m (Raw: %.2f - 0.2)\n', d_final, d_raw);
            else
                crit_dist(ie,it,ia) = NaN;
                fprintf('Avoidable (Target passes or Ego stops)\n');
            end
        end
    end
end

save('unavoidable_map.mat', 'ego_speeds', 'tgt_speeds', 'angles_deg', 'crit_dist');
fprintf('\nScan Complete. Saved unavoidable_map.mat\n');


%% ====================================================================
%%  Sweep Logic
%% ====================================================================

function dcrit = find_critical_unavoidable_distance(scenario_base)
% Finds the closest distance where a CENTRAL CRASH is INEVITABLE.
% Scans from 4m to 25m.

    ego_dist_vec = 4:0.1:25; 
    nD = numel(ego_dist_vec);
    
    unavoidable = false(nD, 1);

    for id = 1:nD
        d_start = ego_dist_vec(id);
        scen = scenario_base;
        scen.ego.y0 = -d_start;

        % If ANY strategy (Steer Left/Right/Straight + Brake/Gas) works, it's Avoidable.
        is_avoidable = check_if_avoidable_sweep(scen);
        
        unavoidable(id) = ~is_avoidable;
    end

    idx = find(unavoidable);
    if isempty(idx)
        dcrit = NaN; 
    else
        dcrit = ego_dist_vec(max(idx)); 
    end
end

function is_avoidable = check_if_avoidable_sweep(scenario)
% Tests fixed steering AND throttle strategies. 
% Returns TRUE if at least ONE combination avoids a critical crash.
    
    max_s = scenario.ego.max_steer;
    steer_targets = [-max_s, -max_s*0.5, 0, max_s*0.5, max_s];
    
    % NEW: Scan both Full Brake AND Full Throttle
    throttle_targets = [scenario.ego.max_brake, scenario.ego.max_throttle];
    
    for throttle_cmd = throttle_targets
        for target_delta = steer_targets
            
            % Pass both steer and throttle to the simulation
            critical_crash = run_trajectory_with_ramp(scenario, target_delta, throttle_cmd);
            
            if ~critical_crash
                % Found a safe path (either by braking OR accelerating)
                is_avoidable = true; 
                return;
            end
        end
    end
    
    % If we get here, absolutely nothing worked.
    is_avoidable = false;
end

function critical_collision = run_trajectory_with_ramp(scenario, target_delta, target_throttle)
% Simulates trajectory. Returns TRUE only if CRITICAL SIDE IMPACT occurs.
% Returns FALSE if:
% 1. Miss (Target passes or Ego stops)
% 2. Glancing Blow (Bumper/Fender hit)

    dt = scenario.dt;
    N  = floor(scenario.T_sim / dt);
    
    ego = initialize_veh_state(scenario.ego);
    tgt = initialize_veh_state(scenario.tgt);
    
    % Precompute target
    time = 0:dt:scenario.T_sim;
    tgt_traj = precompute_target_trajectory(tgt, time, scenario);
    
    critical_collision = false;
    current_steer = 0;
    
    % Critical X-zone (Target Frame)
    % Target Length 4.6m -> Ends at +/- 2.3m. 
    % Margin 0.25m -> Critical zone is [-2.05, 2.05]
    crit_limit = (scenario.tgt.length / 2) - 0.25; 

    for k = 1:N
        current_tgt = tgt_traj(k);
        
        % Control
        rate_limit = steer_rate_from_speed(ego.v);
        max_step   = rate_limit * dt;
        step  = max(-max_step, min(max_step, target_delta - current_steer));
        current_steer = current_steer + step;
        
        ego.throttle = target_throttle; % UPDATED: Uses input (Brake OR Gas)
        ego.steering = current_steer;
        
        % Dynamics
        ego = update_veh_dynamics(ego, scenario);
        
        % Collision Check (Polyshape)
        [is_hit, impact_pt] = check_collision_simple(ego, current_tgt, scenario);
        
        if is_hit
            % impact_pt is relative to Target Center in Target Frame
            imp_x_tgt = impact_pt(1);
            
            % Check if hit is Central (Critical)
            if abs(imp_x_tgt) < crit_limit
                critical_collision = true;
                return;
            else
                % Glancing hit (Bumper) -> Treated as Avoided for this map
                critical_collision = false; 
                return; 
            end
        end
    end
end


%% ====================================================================
%%  Helpers
%% ====================================================================

function scenario = setup_scenario_for_scan()
    scenario.dt    = 0.02;   
    scenario.T_sim = 2.5;    

    % Ego
    scenario.ego.x0 = -2.3;
    scenario.ego.y0 = -7.0; % Placeholder
    scenario.ego.v0 = 18.0;
    scenario.ego.heading0_rad = deg2rad(90);
    scenario.ego.length = 4.6; scenario.ego.width = 1.9;
    scenario.ego.mass = 1600; scenario.ego.Iz = 2500;
    scenario.ego.lf = 1.35; scenario.ego.lr = 1.35;
    scenario.ego.csf = 8e4; scenario.ego.csr = 1e5;
    
    scenario.ego.a_long_max = 6.5; scenario.ego.a_long_min = -7.0;
    scenario.ego.max_throttle = 1.0; scenario.ego.max_brake = -1.0;

    % Target (FIXED: Explicitly defined, NO overwrite)
    scenario.tgt.x0 = -10.0;
    scenario.tgt.y0 = 2.2;
    scenario.tgt.v0 = 18.0; 
    scenario.tgt.heading0_rad = deg2rad(0);
    scenario.tgt.length = 4.6; scenario.tgt.width = 1.9;
    
    % Copy physics params from Ego to Target (manual copy, NOT struct overwrite)
    scenario.tgt.mass = scenario.ego.mass;
    scenario.tgt.Iz   = scenario.ego.Iz;
    scenario.tgt.lf   = scenario.ego.lf;
    scenario.tgt.lr   = scenario.ego.lr;
    scenario.tgt.csf  = scenario.ego.csf;
    scenario.tgt.csr  = scenario.ego.csr;

    scenario.regions.front_door = struct('range', [ 0.21,  1.40]);
end

function steer_rate = steer_rate_from_speed(v_mps)
    v_kph = v_mps * 3.6;
    if v_kph <= 60
        steer_rate = deg2rad(60); % VERY AGILE (Wheel angle rate)
    elseif v_kph <= 100
        steer_rate = deg2rad(45); % AGILE EVASION
    else
        steer_rate = deg2rad(15); % STABLE
    end
end

function next_state = update_veh_dynamics(state, scenario)
% Fiala Model + Euler + Clamps
    params = scenario.ego; dt = scenario.dt;
    x=state.x; y=state.y; psi=state.psi; v=state.v; r=state.psi_dot; beta=state.beta;
    thr=state.throttle; str=state.steering;

    % CLAMPS (Wheel limits)
    thr = max(params.max_brake, min(params.max_throttle, thr));
    str = max(-params.max_steer, min(params.max_steer, str));

    if thr >= 0, ax = thr * params.a_long_max;
    else, ax = thr * abs(params.a_long_min); end
    
    Fx = params.mass * ax / 2; 
    g=9.81; m=params.mass; lf=params.lf; lr=params.lr;
    Fz_f = m*g*lr/(lf+lr); Fz_r = m*g*lf/(lf+lr);
    v_safe = max(v, 1.0);
    
    af = str - beta - lf*r/v_safe; ar = -beta + lr*r/v_safe;
    mu=0.9;

    % Front
    Fy_max_f = sqrt((mu*Fz_f)^2 - Fx^2);
    if Fy_max_f<=0, Fyf=0; else
        a_sl_f = atan(3*Fy_max_f/(2*params.csf));
        if abs(af)<a_sl_f, Fyf = -sign(af)*(2*params.csf*abs(tan(af)) - (2*params.csf)^2/(3*Fy_max_f)*tan(af)^2 + (2*params.csf)^3/(27*Fy_max_f^2)*abs(tan(af))^3);
        else, Fyf = -Fy_max_f*sign(af); end
    end
    
    % Rear
    Fy_max_r = sqrt((mu*Fz_r)^2 - Fx^2);
    if Fy_max_r<=0, Fyr=0; else
        a_sl_r = atan(3*Fy_max_r/(2*params.csr));
        if abs(ar)<a_sl_r, Fyr = -sign(ar)*(2*params.csr*abs(tan(ar)) - (2*params.csr)^2/(3*Fy_max_r)*tan(ar)^2 + (2*params.csr)^3/(27*Fy_max_r^2)*abs(tan(ar))^3);
        else, Fyr = -Fy_max_r*sign(ar); end
    end

    Fyf = -Fyf; Fyr = -Fyr; 

    next_state = state; 
    next_state.x = x + v*cos(psi+beta)*dt;
    next_state.y = y + v*sin(psi+beta)*dt;
    next_state.psi = psi + r*dt;
    next_state.v = max(0, v + ax*dt);
    next_state.psi_dot = r + (lf*Fyf - lr*Fyr)/params.Iz*dt;
    next_state.beta = beta + (Fyf+Fyr)/(m*v_safe)*dt - r*dt;
end

function s = initialize_veh_state(p)
    s=struct('x',p.x0,'y',p.y0,'psi',p.heading0_rad,'v',p.v0,'psi_dot',0,'beta',0,'throttle',0,'steering',0,'length',p.length,'width',p.width);
end
function t = precompute_target_trajectory(s, time, scen)
    t = repmat(s, numel(time), 1);
    for i=2:numel(time), dt=time(i)-time(i-1);
    t(i).x = t(i-1).x + s.v*cos(s.psi)*dt; t(i).y = t(i-1).y + s.v*sin(s.psi)*dt; end
end
function [c, pt] = check_collision_simple(e, t, scen)
    ep=get_poly(e); tp=get_poly(t); 
    int=intersect(polyshape(ep(:,1),ep(:,2)), polyshape(tp(:,1),tp(:,2)));
    c=~isempty(int.Vertices); pt=[0,0];
    if c
        [cx,cy]=centroid(int); 
        dx=cx-t.x; dy=cy-t.y;
        pt=[dx*cos(t.psi)+dy*sin(t.psi), -dx*sin(t.psi)+dy*cos(t.psi)]; 
    end
end
function p = get_poly(s)
    hl=s.length/2; hw=s.width/2;
    lc=[hl,hw; hl,-hw; -hl,-hw; -hl,hw];
    c=cos(s.psi); sn=sin(s.psi);
    x=s.x+lc(:,1)*c-lc(:,2)*sn; y=s.y+lc(:,1)*sn+lc(:,2)*c;
    p=[x,y];
end
