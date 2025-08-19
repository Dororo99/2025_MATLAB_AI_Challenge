%% Sample Time
Ts = 0.1;               % Simulation sample time                (s)

%% Scenario Authoring
[scenario,egoVehicle] = RL_fault_tolerance_scenario;

% Define road curvature
R = inf;

%% Tracking and Sensor Fusion Parameters                        Units
clusterSize = 4;        % Distance for clustering               (m)
assigThresh = 50;       % Tracker assignment threshold          (N/A)
M           = 2;        % Tracker M value for M-out-of-N logic  (N/A)
N           = 3;        % Tracker M value for M-out-of-N logic  (N/A)
numCoasts   = 5;        % Number of track coasting steps        (N/A)
numTracks   = 20;       % Maximum number of tracks              (N/A)
% numSensors 숫자 변경
numSensors  = 2;        % Maximum number of sensors             (N/A)

% Position and velocity selectors from track state
% The filter initialization function used in this example is initcvekf that 
% defines a state that is: [x;vx;y;vy;z;vz]. 
posSelector = [1,0,0,0,0,0; 0,0,1,0,0,0]; % Position selector   (N/A)
velSelector = [0,1,0,0,0,0; 0,0,0,1,0,0]; % Velocity selector   (N/A)

%% Ego Car 
% Dynamics modeling parameters
m       = 1575;     % Total mass of vehicle                          (kg)
Iz      = 2875;     % Yaw moment of inertia of vehicle               (m*N*s^2)
lf      = 1.2;      % Longitudinal distance from c.g. to front tires (m)
lr      = 1.6;      % Longitudinal distance from c.g. to rear tires  (m)
Cf      = 19000;    % Cornering stiffness of front tires             (N/rad)
Cr      = 33000;    % Cornering stiffness of rear tires              (N/rad)
tau     = 0.5;      % Longitudinal time constant                     (N/A)

% Initial condition for the ego car
v0_ego = 20.6;         % Initial speed of the ego car           (m/s)
x0_ego = 0;            % Initial x position of ego car          (m)
y0_ego = -2;           % Initial y position of ego car          (m)

% System matrices
A = [-(2*Cf+2*Cr)/m/v0_ego,0,-v0_ego-(2*Cf*lf-2*Cr*lr)/m/v0_ego,0,0;...
     0,0,1,0,0;...
     -(2*Cf*lf-2*Cr*lr)/Iz/v0_ego,0,-(2*Cf*lf^2+2*Cr*lr^2)/Iz/v0_ego,0,0;...
     0,0,0,0,1;...
     0,0,0,0,-1/tau];
B = [0,0,0,0,1/tau]';
C = [0,0,0,1,0];
G = minreal(ss(A,B,C,0));

% Ego Vehicle Actor ID
egoID = egoVehicle.ActorID; 

%% Controller Parameters
v_set           = 21.5; % set speed                         (m/s)
time_gap        = 1.5;  % time gap                          (s)
default_spacing = 5;    % default spacing                   (m)
verr_gain       = 0.5;  % velocity error gain               (N/A)
xerr_gain       = 0.2;  % spacing error gain                (N/A)
vx_gain         = 0.4;  % relative velocity gain            (N/A)
max_ac          = 2;    % Maximum acceleration                  (m/s^2)
min_ac          = -3;   % Minimum acceleration                  (m/s^2)

%% Driver steering control paramaters
driver_P        = 0.2;  % Proportional gain                     (N/A)
driver_I        = 0.1;  % Integral gain                         (N/A)
yawerr_gain     = 2;    % Yaw error gain                        (N/A)

%% Bus Creation
% Create the bus of actors from the scenario reader
modelName = 'RL_fault_tolerance_simulation';

wasModelLoaded = bdIsLoaded(modelName);
if ~wasModelLoaded
    load_system(modelName)
end
blk=find_system(modelName,'System','driving.scenario.internal.ScenarioReader');
s = get_param(blk{1},'PortHandles');
get(s.Outport(1),'SignalHierarchy');

% Create bus for detections (Input to the referenced model)
blk=find_system(modelName,'System','visionDetectionGenerator');
visionDetectionGenerator.createBus(blk{1});
blk=find_system(modelName,'System','drivingRadarDataGenerator');
radarDetectionGenerator.createBus(blk{1});

% Create the bus of tracks
wasReModelLoaded = bdIsLoaded(modelName);
if ~wasReModelLoaded
    load_system(modelName)
    blk=find_system(modelName,'MatchFilter', @Simulink.match.allVariants,'Name','Multi-Object Tracker');
    multiObjectTracker.createBus(blk{1});
    close_system(modelName)
else
    blk=find_system(modelName,'MatchFilter', @Simulink.match.allVariants,'Name','Multi-Object Tracker');
    multiObjectTracker.createBus(blk{1});
end

if ~wasModelLoaded
    close_system(modelName)
end
