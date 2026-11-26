function [objects, cls] = Classify(LidarFiltered, state, cls)
%CLASSIFY  Turn filtered lidar points into a list of classified objects.
%
%   [objects, cls] = Classify(LidarFiltered, state, cls)
%
%   Inputs
%     LidarFiltered - 3 x n lidar points ALREADY filtered and transformed to NED
%                     [N; E; D] (done in Autonomy.m: self-returns, water line,
%                     and sensor -> NED). Pass zeros(3, 0) for "no points".
%     state     - vehicle state (uses state.time and state.Pos)
%     cls       - classifier memory. Pass [] the first time, then pass back
%                 whatever Classify returned last time.
%
%   Outputs
%     objects   - CONFIRMED objects only, one struct per object:
%                   id, className, center [E N], area, height,
%                   hull [E N], firstSeen, lastSeen, confirmed
%     cls       - updated classifier memory. Also holds things that are
%                 useful for plotting/debugging:
%                   cls.points        3 x n mapped points [N; E; D]
%                   cls.clusterLabels DBSCAN label per point (-1 = noise)
%                   cls.clusterHulls  one closed [E N] hull per cluster
%                   cls.clusterAreas / clusterHeights / clusterCenters
%                   cls.detections    this frame's labeled clusters
%                   cls.tracks        ALL tracks (confirmed + unconfirmed)
%                   cls.classes       the class table (names, colors, limits)
%
%   Usage in Autonomy.m:
%     % init block:   data.cls = [];
%     [objects, data.cls] = Classify(LidarFiltered, state, data.cls);
%
%   NOTE: settings and classes are read once on the first call.
%   Restart the sim after editing them.

% ---- First call: set up memory ----
if isempty(cls)
    cls.settings = defineSettings();
    cls.classes  = defineClasses();

    cls.points = zeros(3, 0);   % mapped points [N; E; D]
    cls.time   = zeros(1, 0);   % time each point was added

    cls.tracks = struct('id', {}, 'className', {}, 'center', {}, 'area', {}, ...
                        'height', {}, 'hull', {}, 'firstSeen', {}, ...
                        'lastSeen', {}, 'confirmed', {});
    cls.nextTrackId = 1;
end

% ---- The pipeline ----
cls        = updatePointMap(cls, LidarFiltered, state);   % filtered NED points -> rolling map
cls        = findClusters(cls);                       % DBSCAN + hulls + area/height
detections = makeDetections(cls, state);              % label each cluster
cls        = updateTracks(cls, detections, state);    % persistence + timers
cls.detections = detections;

% ---- Output: confirmed tracks only ----
objects = cls.tracks([cls.tracks.confirmed]);

end


%% ========================================================================
%  SETTINGS - every tuning number lives here
%  ========================================================================
function s = defineSettings()

% Point map
s.mapMemory      = 2.0;              % s, how long points stay in the map

% Clustering
s.dbscanEps    = 0.7;                % m, DBSCAN neighbor radius
s.dbscanMinPts = 10;                 % points needed to form a cluster

% Detection
s.minDetectDist = 1.0;               % m, clusters closer than this are our own boat

% Tracker timers
s.matchDistance = 1.0;               % m, detection this close to a track = same object
s.confirmTime   = 2.5;               % s, must be longer than mapMemory
s.forgetTime    = 1.5;               % s, unconfirmed track unseen this long = noise
s.dropTime      = 300;                % s, confirmed track unseen this long = removed

end


%% ========================================================================
%  CLASSES - to add an object type, copy a block and change the number
%  ========================================================================
%   name       - label given to the object
%   minArea    - smallest hull area allowed (m^2)
%   maxArea    - largest hull area allowed (m^2)
%   minHeight  - shortest allowed (m above water)
%   maxHeight  - tallest allowed (m above water)
%   minRange   - only trust area/height if farther than this (m)
%   maxRange   - only trust area/height if closer than this (m)
%   useTracker - true: tracked here; false: exception handled elsewhere (dock)
%   color      - plot color (not the object's real color)
%   First class whose area AND height both fit wins, so order matters.
function classes = defineClasses()

classes = struct([]);

classes(1).name       = "A0";
classes(1).minArea    = 0.15;
classes(1).maxArea    = 0.28;
classes(1).minHeight  = 0.10;
classes(1).maxHeight  = 0.21;
classes(1).minRange   = 1.0;
classes(1).maxRange   = 15;
classes(1).useTracker = true;
classes(1).color      = [1 0.5 0];

classes(2).name       = "DOCK";
classes(2).minArea    = 2.0;
classes(2).maxArea    = inf;
classes(2).minHeight  = 0;
classes(2).maxHeight  = inf;
classes(2).minRange   = 0;
classes(2).maxRange   = 25;
classes(2).useTracker = false;      % dock gets its own section later
classes(2).color      = [0.6 0.4 0.2];

classes(3).name       = "GateBuoy";
classes(3).minArea    = 0.35;
classes(3).maxArea    = 0.55;
classes(3).minHeight  = 0.8;
classes(3).maxHeight  = 1.0;
classes(3).minRange   = 1.0;
classes(3).maxRange   = 15;
classes(3).useTracker = true;
classes(3).color      = [0 1 0];

classes(4).name       = "CourseBoat";
classes(4).minArea    = 0.6;
classes(4).maxArea    = 1.4;
classes(4).minHeight  = 0.4;
classes(4).maxHeight  = 0.8;
classes(4).minRange   = 1.0;
classes(4).maxRange   = 15;
classes(4).useTracker = true;
classes(4).color      = [1 1 0];

classes(5).name       = "LightTower";
classes(5).minArea    = 1.3;
classes(5).maxArea    = 3.0;
classes(5).minHeight  = 0.9;
classes(5).maxHeight  = 3.2;
classes(5).minRange   = 1.0;
classes(5).maxRange   = 15;
classes(5).useTracker = true;
classes(5).color      = [1 1 1];

classes(6).name       = "A2";
classes(6).minArea    = 0.3;
classes(6).maxArea    = 0.6;
classes(6).minHeight  = 0.22;
classes(6).maxHeight  = 0.4;
classes(6).minRange   = 1.0;
classes(6).maxRange   = 15;
classes(6).useTracker = true;
classes(6).color      = [0.5 0 0.5];

end


%% ========================================================================
%  STEP 1: filtered NED points -> rolling point map
%  ========================================================================
function cls = updatePointMap(cls, LidarFiltered, state)
s = cls.settings;

% Forget points older than mapMemory
notOld = (state.time - cls.time) < s.mapMemory;
cls.points = cls.points(:, notOld);
cls.time   = cls.time(notOld);

% Add the new points (already filtered and in NED: [N; E; D])
cls.points = [cls.points LidarFiltered];
cls.time   = [cls.time state.time * ones(1, size(LidarFiltered, 2))];

end


%% ========================================================================
%  STEP 2: DBSCAN clusters -> hull, center, area, height per cluster
%  ========================================================================
%  Speed notes (results are the same as the polyshape version):
%   - points are grouped by label once (sort), not by a full-array mask per cluster
%   - area + centroid come straight from the hull polygon (shoelace formula);
%     building a polyshape per cluster per frame was the expensive part
%   - the collinearity test is a 2x2 determinant instead of rank() (an SVD)
function cls = findClusters(cls)
s = cls.settings;

% Start empty so the outputs always exist
cls.clusterLabels  = [];
cls.clusterHulls   = {};
cls.clusterCenters = zeros(2, 0);
cls.clusterAreas   = zeros(1, 0);
cls.clusterHeights = zeros(1, 0);

if isempty(cls.points)
    return
end

% One label per point: -1 = noise, 1, 2, 3... = cluster number
labels = dbscan(cls.points', s.dbscanEps, s.dbscanMinPts);
cls.clusterLabels = labels;

% Group the clustered points: sort by cluster so each cluster is a contiguous run
validIdx = find(labels > 0);
if isempty(validIdx)
    nClusters = 0;
else
    [~, ~, g] = unique(labels(validIdx));      % g = 1..nClusters, ascending id order
    [g, order] = sort(g);
    validIdx  = validIdx(order);
    nClusters = g(end);
    runEnd    = [find(diff(g)); numel(g)];
    runStart  = [1; runEnd(1:end-1) + 1];
end

cls.clusterHulls   = cell(nClusters, 1);     % closed [E N] hulls
cls.clusterCenters = nan(2, nClusters);      % [E; N]
cls.clusterAreas   = zeros(1, nClusters);    % 0 = no valid hull
cls.clusterHeights = zeros(1, nClusters);    % tallest point above water

for k = 1:nClusters
    clusterPts = cls.points(:, validIdx(runStart(k):runEnd(k)));

    % Height above water = tallest point. D points down, so "up" is -D.
    cls.clusterHeights(k) = max(-clusterPts(3, :));

    % Top-down [E N], without exact duplicates
    pts2D = unique([clusterPts(2, :)' clusterPts(1, :)'], 'rows', 'stable');

    % Need at least 3 points that aren't all on one line
    if size(pts2D, 1) < 3 || isCollinear(pts2D)
        cls.clusterHulls{k} = pts2D;
        continue
    end

    % Closed hull (last point repeats the first)
    try
        hullIdx = convhull(pts2D(:, 1), pts2D(:, 2));
    catch
        cls.clusterHulls{k} = pts2D;     % qhull says it is flat: no valid hull
        continue
    end
    hull = pts2D(hullIdx, :);
    cls.clusterHulls{k} = hull;

    % Area and center from the hull polygon (shoelace formula, counter-clockwise)
    x = hull(:, 1) - hull(1, 1);          % shift to the first vertex for precision
    y = hull(:, 2) - hull(1, 2);
    cr = x(1:end-1) .* y(2:end) - x(2:end) .* y(1:end-1);
    A2 = sum(cr);                         % twice the signed area

    if A2 > 0
        cls.clusterAreas(k) = A2 / 2;
        cls.clusterCenters(:, k) = [ ...
            hull(1, 1) + sum((x(1:end-1) + x(2:end)) .* cr) / (3 * A2); ...
            hull(1, 2) + sum((y(1:end-1) + y(2:end)) .* cr) / (3 * A2)];
    end
end

end


function tf = isCollinear(pts2D)
% True if the 2D points lie (numerically) on one line.
X  = pts2D - mean(pts2D, 1);
C  = X' * X;                              % 2x2 scatter matrix
tr = C(1, 1) + C(2, 2);
dt = C(1, 1) * C(2, 2) - C(1, 2) ^ 2;
tf = dt <= 1e-12 * tr ^ 2;
end


%% ========================================================================
%  STEP 3: label every valid cluster using the class table
%  ========================================================================
function detections = makeDetections(cls, state)
s = cls.settings;
classes = cls.classes;

% Class limits as arrays, so one comparison tests every class at once
minA = [classes.minArea];    maxA = [classes.maxArea];
minH = [classes.minHeight];  maxH = [classes.maxHeight];

boatEN = [state.Pos(2) state.Pos(1)];   % [East North]

detections = struct('center', {}, 'area', {}, 'height', {}, 'hull', {}, ...
                    'className', {}, 'inRange', {}, 'useTracker', {});

for k = 1:numel(cls.clusterAreas)
    clusterArea   = cls.clusterAreas(k);
    clusterHeight = cls.clusterHeights(k);
    clusterCenter = cls.clusterCenters(:, k)';   % [East North]

    % Skip clusters with no valid hull
    if clusterArea == 0 || any(isnan(clusterCenter))
        continue
    end

    % Skip anything centered on our own boat
    distToBoat = norm(clusterCenter - boatEN);
    if distToBoat < s.minDetectDist
        continue
    end

    % Default: unknown
    className  = "UNKNOWN";
    inRange    = false;
    useTracker = true;

    % First class whose area AND height both fit wins
    c = find(clusterArea   >= minA & clusterArea   <= maxA & ...
             clusterHeight >= minH & clusterHeight <= maxH, 1);

    if ~isempty(c)
        className  = classes(c).name;
        inRange    = distToBoat >= classes(c).minRange && distToBoat <= classes(c).maxRange;
        useTracker = classes(c).useTracker;
    end

    n = numel(detections) + 1;
    detections(n).center     = clusterCenter;
    detections(n).area       = clusterArea;
    detections(n).height     = clusterHeight;
    detections(n).hull       = cls.clusterHulls{k};
    detections(n).className  = className;
    detections(n).inRange    = inRange;
    detections(n).useTracker = useTracker;
end

end


%% ========================================================================
%  STEP 4: tracker - keep objects after they leave the lidar view
%  ========================================================================
%   1. Each detection is matched to the nearest existing track.
%   2. If no track is close enough, a new (unconfirmed) track is made.
%   3. A track that keeps being seen for confirmTime becomes confirmed.
%   4. Unconfirmed + unseen for forgetTime -> deleted (noise).
%      Confirmed   + unseen for dropTime   -> deleted (stale).
function cls = updateTracks(cls, detections, state)
s = cls.settings;
tracks = cls.tracks;

% Track centers as a 2 x nTracks matrix [E; N], kept in step with the tracks
% (matched tracks move, new tracks are appended) so the nearest search is one
% vectorized call instead of a loop.
centers = reshape([tracks.center], 2, numel(tracks));

% ---- Steps 1 & 2: match or create ----
for d = 1:numel(detections)
    thisDetection = detections(d);

    % Exceptions (like the dock) are handled elsewhere
    if ~thisDetection.useTracker
        continue
    end

    c = thisDetection.center(:);          % [E; N]

    % Nearest existing track
    if isempty(centers)
        nearestTrack = 0;
        nearestDist  = inf;
    else
        [nearestDist, nearestTrack] = min(hypot(centers(1, :) - c(1), ...
                                                centers(2, :) - c(2)));
    end

    if nearestDist <= s.matchDistance
        % Same object: update it
        t = nearestTrack;
        tracks(t).center   = thisDetection.center;
        tracks(t).area     = thisDetection.area;
        tracks(t).height   = thisDetection.height;
        tracks(t).hull     = thisDetection.hull;
        tracks(t).lastSeen = state.time;
        centers(:, t)      = c;

        % Only change the class when close enough to trust it
        if thisDetection.inRange && thisDetection.className ~= "UNKNOWN"
            tracks(t).className = thisDetection.className;
        end
    else
        % New object
        t = numel(tracks) + 1;
        tracks(t).id        = cls.nextTrackId;
        tracks(t).className = "UNKNOWN";
        tracks(t).center    = thisDetection.center;
        tracks(t).area      = thisDetection.area;
        tracks(t).height    = thisDetection.height;
        tracks(t).hull      = thisDetection.hull;
        tracks(t).firstSeen = state.time;
        tracks(t).lastSeen  = state.time;
        tracks(t).confirmed = false;
        centers(:, t)       = c;

        if thisDetection.inRange
            tracks(t).className = thisDetection.className;
        end

        cls.nextTrackId = cls.nextTrackId + 1;
    end
end

% ---- Steps 3 & 4: confirm / delete (backwards so deleting doesn't skip) ----
for t = numel(tracks):-1:1
    timeTracked   = tracks(t).lastSeen - tracks(t).firstSeen;
    timeSinceSeen = state.time - tracks(t).lastSeen;

    if tracks(t).confirmed
        if timeSinceSeen > s.dropTime
            tracks(t) = [];
        end
    else
        if timeTracked >= s.confirmTime
            tracks(t).confirmed = true;
        elseif timeSinceSeen > s.forgetTime
            tracks(t) = [];
        end
    end
end

cls.tracks = tracks;

end
