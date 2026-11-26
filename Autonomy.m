function [commands,scanthecode,data]=Autonomy(state,LidarScan,cameraImage,data)
%[commands,scanthecode,data]=Autonomy(state,LidarScan,cameraImage,data)
%This function is where all processing related to autonomy takes place.
%Students can re-write this function to do whatever they desire to complete
%a given assignment. The function I/O is defined as follows:
%
%Inputs:
%state - a structure containing the vehicle state
%       state.time - time in seconds
%       state.Pos - NED location in meters [N,E,D]
%       state.Vel - velocity in meters/sec [N_dot,E_dot,D_dot]
%       state.rot - angular position in radians [roll, pitch, yaw]
%       state.rVel - angular velocity in rad/s [roll, pitch, yaw]
%LidarScan - is a point cloud expressed as an nx3 array. Each row is then a
%       point (x,y,z) in the sensor frame.
%cameraImage - A RGB picture as seen by the camera
%data - a structure that used to pass data from one iteration to the next.
%       The only data here is what you previously assigned. So it is empty
%       initially.
%
%Outputs:
%commands - a structure containing all commands
%commands.actuators - a 2x1 array of commands sent to the thrusters. The
%       first value is the port thruster and the 2nd the starboard thruster
%commands.projectiles - a 2x1 array of logicals. First value indicates if
%       the water cannon should be on. Second value is for the ball
%       launcher.
%scanthecode - a 3 element string indicating the observed light sequence.
%       e.g. "RGB". Only the first assigned value is accepted.
%data - a structure that used to pass data from one iteration to the next.
%       The only data here is what you previously assigned. So it is empty
%       initially.
%
% Layout of this file
%   1. Initialization  - everything that is created once lives in the
%                        "if isempty(data)" block (parameters, figures, handles)
%   2. LiDAR filtering - raw scan -> LidarFiltered (3 x n NED points), shared by
%                        classification and (later) obstacle avoidance
%   3. Classification  - Classify.m turns LidarFiltered into objects
%   4. Plotting        - updatePlots() only sets properties on existing handles
%   5. Control         - RC (joystick) or waypoint following, chosen by data.useRC.
%                        waypointControl drives to data.goal, which tasks set.

%Default scanthecode report
scanthecode='';

%% ========================================================================
%  1. INITIALIZATION (runs once)
%  ========================================================================
if isempty(data)

    % ---- Mode ----
    data.useRC = false;          % true = joystick (RC), false = waypoint following
    data.objectCal = false;      % true = turn on prints for object data (clusters, area, height)

    % ---- Classifier memory (Classify.m fills it in on the first call) ----
    data.cls = [];

    % ---- RC control ----
    data.deadzone     = 0.08;
    data.buttonThresh = 0.5;
    data.joy = [];
    if data.useRC
        data.joy = sim3d.io.Joystick(ID=1);
    end

    % ---- Goal (the ONLY input to navigation) ----
    %   Tasks (search, gates, dock, ...) write [N E yaw] here (yaw in rad, NED;
    %   NaN = no final yaw, end facing the direction of travel); [] = no goal,
    %   hold still. waypointControl drives to whatever is in data.goal.
    data.goal = [];
    % data.goal = [30 0 pi]; % uncomment to test navigation on its own (end facing East)
    data.search = [];

    % ---- Navigation (waypointControl) ----
    %   Read-only for tasks: data.nav.status ("idle"/"turn"/"drive"/"arrived"),
    %   data.nav.arrived, data.nav.distErr, data.nav.waypoint
    data.nav.status      = "idle";
    data.nav.arrived     = false;
    data.nav.waypoint    = [];      % [N E] goal currently being driven to
    data.nav.goalYaw     = NaN;     % rad, final yaw (NaN = none)
    data.nav.distErr     = inf;     % m, distance to waypoint (inf = no waypoint)
    data.nav.lastYawErr  = [];
    data.nav.lastDistErr = [];
    data.nav.dt          = 0.1;     % s, assumed control period for the D terms
    data.nav.yawKP       = 0.5;
    data.nav.yawKD       = 0.3;
    data.nav.forwardKP   = 0.2;
    data.nav.forwardKD   = 0.02;
    data.nav.maxForward  = 0.75;    % forward command limit while driving
    data.nav.CEP99       = 0.4459;  % m, waypoint reached when closer than this
    data.nav.yawStd      = 0.0152;  % rad, heading "good enough" to start driving
    data.nav.repivotYaw  = deg2rad(45); % rad, stop and turn in place if heading error grows past this
    data.nav.holdRadius  = 1.5;     % m, after arriving, re-approach if pushed farther than this
    data.nav.goalTol     = 0.5;     % m, goal moves less than this = same goal (no reset)
    data.nav.realignYaw  = deg2rad(5); % rad, after arriving, re-align if yaw drifts past this

    % ---- Plot settings ----
    data.maxClusters = 30;      % hull / centroid slots preallocated per frame
    data.maxTracks   = 30;
    data.plotEvery   = 1;       % update the plots every N-th call (raise to go faster)
    data.frame       = 0;
    plotXLim = [-30 30];        % East
    plotYLim = [-10 60];        % North

    % ---- LiDAR filtering (used by filterLidar below) ----
    data.lidar.minPointDist   = 0.05;             % m, drop returns closer than this to the sensor
    data.lidar.sensorOffset   = [0.27 0 -0.34]';  % m, sensor position in the boat FRD frame
    data.lidar.boatHalfLength = 1.0;              % m, self-return box (front-to-back half size)
    data.lidar.boatHalfWidth  = 0.6;              % m, self-return box (side-to-side half size)
    data.lidar.waterline      = -0.05;            % m (D), drop points lower than this
    data.lidar.maxRange       = inf;              % m, OPTIONAL speed-up: drop points farther
                                                  %    than this from the boat (inf = off)

    % ---- Run Classify once on an empty point set so the class table exists now ----
    [~, data.cls] = Classify(zeros(3, 0), state, data.cls);

    classes = data.cls.classes;
    data.classNames  = [string({classes.name}) "UNKNOWN"];
    data.classColors = [vertcat(classes.color); 0.5 0.5 0.5];   % UNKNOWN = grey
    nClass = numel(data.classNames);

    % ---- Figure 1: mapped lidar points (Easting, Northing, Up) ----
    data.fig1 = figure;
    data.plot = plot3(0, 0, 0, 'b.');
    grid on
    axis equal
    xlabel('Easting (m)')
    ylabel('Northing (m)')
    zlabel('Up (m)')
    axis([-30 30 -10 60 -5 5]);

    % ---- Figure 6: clusters ----
    data.clusterFig = figure(6);
    data.clusterAx  = axes(data.clusterFig);
    data.clusterScatter = scatter(data.clusterAx, nan, nan, 12, 1, 'filled');
    grid(data.clusterAx, 'on');
    xlabel(data.clusterAx, 'Easting (m)');
    ylabel(data.clusterAx, 'Northing (m)');
    xlim(data.clusterAx, plotXLim);
    ylim(data.clusterAx, plotYLim);
    daspect(data.clusterAx, [1 1 1]);
    colormap(data.clusterAx, lines(data.maxClusters));      % one color per cluster slot
    clim(data.clusterAx, [0.5, data.maxClusters + 0.5]);

    % ---- Figure 7: hulls (left) and centroids (right) ----
    data.hullFig = figure(7);
    data.tl = tiledlayout(data.hullFig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(data.tl, 'Cluster hulls and centroids');

    % Left tile: this frame's cluster hulls (red dotted) + remembered track hulls
    data.hullAx = nexttile(data.tl);
    hold(data.hullAx, 'on');
    grid(data.hullAx, 'on');
    daspect(data.hullAx, [1 1 1]);
    xlabel(data.hullAx, 'Easting (m)');
    ylabel(data.hullAx, 'Northing (m)');
    title(data.hullAx, 'Raw Hulls');
    xlim(data.hullAx, plotXLim);
    ylim(data.hullAx, plotYLim);

    data.hHull  = gobjects(data.maxClusters, 1);   % live cluster hulls
    for k = 1:data.maxClusters
        data.hHull(k) = patch(data.hullAx, 'Faces', zeros(0, 3), 'Vertices', zeros(0, 2), ...
            'FaceColor', 'none', 'EdgeColor', 'r', 'LineStyle', ':', 'Visible', 'off');
        data.hHull(k).UserData = 0;                % slot count last frame (first slot only)
    end
    data.hTrack = gobjects(data.maxTracks, 1);     % confirmed track hulls
    for k = 1:data.maxTracks
        data.hTrack(k) = patch(data.hullAx, 'Faces', zeros(0, 3), 'Vertices', zeros(0, 2), ...
            'FaceColor', 'none', 'EdgeColor', [0.5 0.5 0.5], 'Visible', 'off');
        data.hTrack(k).UserData = 0;
    end
    hold(data.hullAx, 'off');

    % Right tile: track centroids. filled = confirmed, hollow = unconfirmed
    data.centroidAx = nexttile(data.tl);
    hold(data.centroidAx, 'on');
    grid(data.centroidAx, 'on');
    daspect(data.centroidAx, [1 1 1]);
    xlabel(data.centroidAx, 'Easting (m)');
    ylabel(data.centroidAx, 'Northing (m)');
    title(data.centroidAx, 'Centroids');
    xlim(data.centroidAx, plotXLim);
    ylim(data.centroidAx, plotYLim);
    data.centroidConfirmed   = scatter(data.centroidAx, nan, nan, 50, [0 0 0], 'filled');
    data.centroidUnconfirmed = scatter(data.centroidAx, nan, nan, 50, [0 0 0]);
    hold(data.centroidAx, 'off');

    % ---- Figure 2: the simulator's 2D map (one marker set + one hull line per class) ----
    data.fig2 = findobj('Type', 'figure', 'Number', 2);
    if isempty(data.fig2)
        data.fig2 = figure(2);          % simulator hasn't created it yet
    end
    data.ax2 = gca(data.fig2);
    hold(data.ax2, 'on');               % left on, so the simulator can't wipe our objects

    data.fig2Hulls   = gobjects(1, nClass);
    data.fig2Markers = gobjects(1, nClass);
    for p = 1:nClass
        data.fig2Hulls(p) = plot(data.ax2, nan, nan, '-', ...
            'Color', data.classColors(p, :), 'LineWidth', 1);
        data.fig2Markers(p) = scatter(data.ax2, nan, nan, 18, 'filled', ...
            'MarkerFaceColor', data.classColors(p, :), ...
            'MarkerEdgeColor', data.classColors(p, :));
    end

    % Waypoint marker on the simulator map
    data.waypointPlot = plot(data.ax2, nan, nan, 'bo');
end

%% ========================================================================
%  2. LIDAR FILTERING (raw scan -> NED points)
%  ========================================================================
%   LidarFiltered - 3 x n points this frame, [N; E; D] in the NED frame, with
%                   self-returns, near-sensor returns and water-line points removed.
%                   Classify uses it now; obstacle avoidance can use it later.
LidarFiltered = filterLidar(LidarScan, state, data.lidar);

%% ========================================================================
%  3. CLASSIFICATION (filtered points -> object list)
%  ========================================================================
%   objects  - confirmed objects (id, className, center [E N], area, height, hull, ...)
%   data.cls - classifier memory; also holds points/clusters/tracks for plotting
[objects, data.cls] = Classify(LidarFiltered, state, data.cls);

%% ========================================================================
%  4. PLOTTING (property updates only - nothing is created here)
%  ========================================================================
data.frame = data.frame + 1;
if mod(data.frame, data.plotEvery) == 0
    updatePlots(data, objects);
end

%% ========================================================================
%  5. CONTROL
%  ========================================================================
%   (Task manager goes here later: it picks the active task, and the task sets
%    data.goal. Navigation below just drives to data.goal.)
if data.useRC
    commands = rcControl(data);
else
    % Everything to stay out of: this frame's clusters (incl. dock, unconfirmed)
    % plus confirmed objects remembered by the tracker
    hulls = data.cls.clusterHulls(data.cls.clusterAreas > 0);
    if ~isempty(objects)
        hulls = [hulls; {objects.hull}'];
    end
    % [data.goal, data.search] = search(state, hulls, data.search);   % temporary: until the task manager exists
    [commands, data.nav] = waypointControl(state, data.nav, data.goal);
    commands.projectiles = [false false];
end

% Waypoint marker on the simulator map (hidden when there is no goal)
if isempty(data.nav.waypoint)
    set(data.waypointPlot, 'XData', nan, 'YData', nan);
else
    set(data.waypointPlot, 'XData', data.nav.waypoint(2), 'YData', data.nav.waypoint(1));
end

drawnow;

%% Calibration: average area and height of clusters in view
if data.objectCal
    printObjectCalibration(data, state);
end

end



%%  LiDAR filtering: raw sensor-frame scan -> 3 x n NED points [N; E; D]
function LidarFiltered = filterLidar(LidarScan, state, f)

% Drop returns right at the sensor (compare squared distance: no sqrt needed)
d2 = sum(LidarScan .^ 2, 2);
LidarScan = LidarScan(d2 > f.minPointDist ^ 2, :);

% Sensor frame -> FRD orientation (x, -y, -z), then move origin to the boat center.
% Each column is a point.
FRDPoints = [LidarScan(:, 1)'; -LidarScan(:, 2)'; -LidarScan(:, 3)'] + f.sensorOffset;

% Drop returns that hit our own boat
onBoat = abs(FRDPoints(1, :)) < f.boatHalfLength & ...
         abs(FRDPoints(2, :)) < f.boatHalfWidth;
FRDPoints = FRDPoints(:, ~onBoat);

% FRD -> NED (yaw only)
yaw = state.rot(3);
R = [cos(yaw) -sin(yaw) 0;
     sin(yaw)  cos(yaw) 0;
     0         0        1];
LidarFiltered = R * FRDPoints + state.Pos';

% Drop points at the water line
LidarFiltered = LidarFiltered(:, LidarFiltered(3, :) < f.waterline);

% Optional range crop (off by default)
if isfinite(f.maxRange)
    dN = LidarFiltered(1, :) - state.Pos(1);
    dE = LidarFiltered(2, :) - state.Pos(2);
    LidarFiltered = LidarFiltered(:, dN .^ 2 + dE .^ 2 <= f.maxRange ^ 2);
end

end

%%  RC (joystick) control
function commands = rcControl(data)

AllSticks = axis(data.joy);
buttons   = button(data.joy);

deadzone     = data.deadzone;
buttonThresh = data.buttonThresh;

% Right stick inputs
yaw      = AllSticks(4);
throttle = -AllSticks(5);

if abs(yaw) <= deadzone
    yaw = 0;
else
    yaw = sign(yaw) * (abs(yaw) - deadzone) / (1 - deadzone);
end

if abs(throttle) <= deadzone
    throttle = 0;
else
    throttle = sign(throttle) * (abs(throttle) - deadzone) / (1 - deadzone);
end

% Differential steering
portCmd      = throttle + yaw;
starboardCmd = throttle - yaw;

% Normalize to keep outputs between -1 and 1
maxMag = max([1, abs(portCmd), abs(starboardCmd)]);
portCmd      = portCmd / maxMag;
starboardCmd = starboardCmd / maxMag;

% Deadman behavior (right bumper button)
if buttons(6) > buttonThresh 
    commands.actuators   = [0 0];
    commands.projectiles = [false false];
else
    commands.actuators   = [portCmd, starboardCmd];
    commands.projectiles = [buttons(1) > buttonThresh, buttons(2) > buttonThresh];
end

end

%% Object calibration data
function printObjectCalibration(data, state)
% Print average area and height of clusters within view radius.

if isempty(data.cls.clusterAreas)
    return
end

boatEN     = [state.Pos(2) state.Pos(1)];   % [East North]
viewRadius = 15;                            % m, what counts as "in view"

areasInView   = [];
heightsInView = [];

for k = 1:numel(data.cls.clusterAreas)
    clusterCenter = data.cls.clusterCenters(:, k)';

    if data.cls.clusterAreas(k) == 0 || any(isnan(clusterCenter))
        continue
    end

    distToBoat = norm(clusterCenter - boatEN);
    if distToBoat > viewRadius || distToBoat < 1.0
        continue
    end

    areasInView(end+1)   = data.cls.clusterAreas(k);   %#ok<AGROW>
    heightsInView(end+1) = data.cls.clusterHeights(k); %#ok<AGROW>
end

if ~isempty(areasInView)
    fprintf('In view: %d clusters | avg area %.2f m^2 | avg height %.2f m (min %.2f, max %.2f)\n', ...
        numel(areasInView), mean(areasInView), mean(heightsInView), ...
        min(heightsInView), max(heightsInView));
end
end

%%  Navigation: drive to a [N E] goal, optionally finish at a yaw. Handles ALL boat navigation.
function [commands, nav] = waypointControl(state, nav, goal)
%[commands, nav] = waypointControl(state, nav, goal)
%   goal - [N E yaw] (m, m, rad NED: 0 = North, positive = clockwise),
%          [N E] or [N E NaN] for "no final yaw" (ends facing the direction
%          it traveled), or [] for "no goal" (thrusters off).
%   nav  - navigation memory/settings (data.nav). Tasks may read:
%            nav.status   "idle"    no goal
%                         "turn"    turning in place toward the goal
%                         "drive"   driving to the goal (heading held)
%                         "align"   at the goal, turning to the final yaw
%                         "arrived" at the goal (and yaw, if given); thrusters off
%            nav.arrived  true while status is "arrived"
%            nav.distErr  m to the current waypoint (inf when idle)
%
%   The goal can be changed on any call. A position move bigger than
%   nav.goalTol is a new goal (fresh D terms, turn first if not already
%   driving); smaller moves just update the target. A yaw change on its own
%   is picked up by "arrived" (re-aligns past nav.realignYaw).

commands.actuators = [0 0];   % safe default for every path

% ---- Goal handling ----
if isempty(goal)
    nav.status      = "idle";
    nav.arrived     = false;
    nav.waypoint    = [];
    nav.goalYaw     = NaN;
    nav.distErr     = inf;
    nav.lastYawErr  = [];
    nav.lastDistErr = [];
    return
end

goal   = goal(:)';
goalNE = goal(1:2);
nav.goalYaw = NaN;
if numel(goal) >= 3
    nav.goalYaw = goal(3);
end
hasYaw = ~isnan(nav.goalYaw);

if isempty(nav.waypoint) || norm(goalNE - nav.waypoint) > nav.goalTol
    % New goal
    nav.waypoint    = goalNE;
    nav.lastYawErr  = [];
    nav.lastDistErr = [];
    if nav.status ~= "drive"
        nav.status = "turn";
    end
else
    nav.waypoint = goalNE;   % small nudge: same goal
end

% ---- Errors (shared by every state) ----
dN = nav.waypoint(1) - state.Pos(1);
dE = nav.waypoint(2) - state.Pos(2);
distErr    = hypot(dN, dE);
bearingErr = wrapToPi(state.rot(3) - atan2(dE, dN));   % >0: goal is to port, turn CCW
finalErr   = NaN;
if hasYaw
    finalErr = wrapToPi(state.rot(3) - nav.goalYaw);   % same sign convention
end
nav.distErr = distErr;
if isempty(nav.lastDistErr), nav.lastDistErr = distErr; end

% ---- State machine ----
%   Any switch to a state that steers on a DIFFERENT error clears lastYawErr
%   so the D term doesn't spike.
switch nav.status
    case "turn"      % turn in place until pointed at the goal
        if distErr < nav.CEP99
            nav = atGoal(nav, hasYaw);
        else
            [hc, nav] = headingPD(nav, bearingErr);
            commands.actuators = [-hc hc];
            if abs(bearingErr) < nav.yawStd
                nav.status      = "drive";
                nav.lastDistErr = distErr;   % fresh distance D term
            end
        end

    case "drive"     % drive forward while holding heading
        [hc, nav] = headingPD(nav, bearingErr);
        forwardCmd = nav.forwardKP * distErr + ...
                     nav.forwardKD * (distErr - nav.lastDistErr) / nav.dt;
        forwardCmd = max(min(forwardCmd, nav.maxForward), -nav.maxForward);

        port      = max(min(-hc + forwardCmd, 1), -1);
        starboard = max(min( hc + forwardCmd, 1), -1);
        commands.actuators = [port starboard];

        if distErr < nav.CEP99
            nav = atGoal(nav, hasYaw);
            commands.actuators = [0 0];
        elseif abs(bearingErr) > nav.repivotYaw
            % Overshot, got pushed, or goal jumped sideways: stop and turn
            nav.status = "turn";
            commands.actuators = [-hc hc];
        end

    case "align"     % at the goal: turn in place to the final yaw
        if distErr > nav.holdRadius
            nav.status = "turn";            % pushed off the point: go back
            nav.lastYawErr = [];
        elseif ~hasYaw
            nav.status = "arrived";         % final yaw was removed
        else
            [hc, nav] = headingPD(nav, finalErr);
            commands.actuators = [-hc hc];
            if abs(finalErr) < nav.yawStd
                nav.status = "arrived";
                commands.actuators = [0 0];
            end
        end

    case "arrived"   % thrusters off; react only if pushed away or rotated
        if distErr > nav.holdRadius
            nav.status = "turn";
            nav.lastYawErr = [];
        elseif hasYaw && abs(finalErr) > nav.realignYaw
            nav.status = "align";
            nav.lastYawErr = [];
        end

    otherwise        % "idle" with a goal (shouldn't happen) -> start turning
        nav.status = "turn";
end

nav.lastDistErr = distErr;
nav.arrived     = nav.status == "arrived";

end


function nav = atGoal(nav, hasYaw)
% Reached the goal position: align to the final yaw if there is one.
if hasYaw
    nav.status = "align";
else
    nav.status = "arrived";
end
nav.lastYawErr = [];   % align steers on a different error than turn/drive
end


function [hc, nav] = headingPD(nav, yawErr)
% Heading PD on yawErr (rad). Output clamped to [-1 1]; positive = turn CCW,
% applied as actuators [-hc hc].
if isempty(nav.lastYawErr)
    nav.lastYawErr = yawErr;
end
hc = nav.yawKP * yawErr + nav.yawKD * (yawErr - nav.lastYawErr) / nav.dt;
hc = max(min(hc, 1), -1);
nav.lastYawErr = yawErr;
end

%%  Plot updates - set properties on handles made in the init block
function updatePlots(data, objects)
cls = data.cls;

% ---- Figure 1: mapped points ----
set(data.plot, 'XData', cls.points(2, :), ...
               'YData', cls.points(1, :), ...
               'ZData', -cls.points(3, :));

% ---- Figure 6: clustered (non-noise) points, one color per cluster ----
if isempty(cls.clusterLabels)
    keep = false(0, 1);
else
    keep = cls.clusterLabels > 0;
end

if any(keep)
    pts = cls.points(:, keep);
    lab = mod(cls.clusterLabels(keep) - 1, data.maxClusters) + 1;   % wrap into palette
    set(data.clusterScatter, 'XData', pts(2, :)', 'YData', pts(1, :)', 'CData', lab);
else
    set(data.clusterScatter, 'XData', nan, 'YData', nan, 'CData', 1);
end

% ---- Figure 7, left: this frame's cluster hulls ----
valid = find(cls.clusterAreas > 0);
setPatchPool(data.hHull, cls.clusterHulls(valid), []);

% ---- Figure 7: tracks (left = remembered hulls, right = centroids) ----
tracks = cls.tracks;

if isempty(tracks)
    setPatchPool(data.hTrack, {}, []);
    set(data.centroidConfirmed,   'XData', nan, 'YData', nan, 'CData', [0 0 0]);
    set(data.centroidUnconfirmed, 'XData', nan, 'YData', nan, 'CData', [0 0 0]);
else
    % Color of each track from the class table (grey if UNKNOWN)
    [~, row] = ismember([tracks.className], data.classNames);
    row(row == 0) = numel(data.classNames);
    colors  = data.classColors(row, :);
    centers = vertcat(tracks.center);            % [E N]
    conf    = [tracks.confirmed];

    % Confirmed track hulls
    confIdx = find(conf);
    hulls   = {tracks(confIdx).hull};
    hasHull = cellfun(@(h) size(h, 1) >= 3, hulls);
    setPatchPool(data.hTrack, hulls(hasHull), colors(confIdx(hasHull), :));

    % Centroids: filled = confirmed, hollow = unconfirmed
    setScatter(data.centroidConfirmed,   centers(conf, :),  colors(conf, :));
    setScatter(data.centroidUnconfirmed, centers(~conf, :), colors(~conf, :));
end

% ---- Figure 2: confirmed objects on the simulator map ----
if isempty(objects)
    objNames   = strings(1, 0);
    objCenters = zeros(0, 2);
else
    objNames   = [objects.className];
    objCenters = vertcat(objects.center);
end

for p = 1:numel(data.classNames)
    members = find(objNames == data.classNames(p));

    if isempty(members)
        set(data.fig2Markers(p), 'XData', nan, 'YData', nan);
        set(data.fig2Hulls(p),   'XData', nan, 'YData', nan);
        continue
    end

    set(data.fig2Markers(p), 'XData', objCenters(members, 1), ...
                             'YData', objCenters(members, 2));

    % Stack the hulls into one line with NaN between them (so each draws separately)
    hullsE = [];
    hullsN = [];
    for i = members
        h = objects(i).hull;
        if size(h, 1) >= 3
            hullsE = [hullsE; h(:, 1); NaN]; %#ok<AGROW>
            hullsN = [hullsN; h(:, 2); NaN]; %#ok<AGROW>
        end
    end
    if isempty(hullsE), hullsE = nan; hullsN = nan; end
    set(data.fig2Hulls(p), 'XData', hullsE, 'YData', hullsN);
end

end

%%    filling in colors
function setPatchPool(pool, hulls, colors)
% Fill a preallocated pool of patch objects with closed [E N] hulls, and hide
% the slots that were used last frame but are not needed now. The slot count
% from last frame is kept in pool(1).UserData.
n     = min(numel(hulls), numel(pool));
nPrev = pool(1).UserData;

for k = 1:n
    V = hulls{k}(1:end-1, :);                  % drop the repeated closing point
    set(pool(k), 'Vertices', V, 'Faces', 1:size(V, 1), 'Visible', 'on');
    if ~isempty(colors)
        set(pool(k), 'EdgeColor', colors(k, :));
    end
end

for k = n+1:nPrev
    set(pool(k), 'Visible', 'off');
end

pool(1).UserData = n;
end

%%    updating points
function setScatter(h, centers, colors)
% Update one scatter object; NaN keeps it hidden when there is nothing to show.
if isempty(centers)
    set(h, 'XData', nan, 'YData', nan, 'CData', [0 0 0]);
else
    set(h, 'XData', centers(:, 1), 'YData', centers(:, 2), 'CData', colors);
end
end
