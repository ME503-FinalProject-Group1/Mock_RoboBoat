function [goal, S] = search(state, hulls, S)
%SEARCH  Cover the search area so other tasks can find their objects.
%
%   [goal, S] = search(state, hulls, S)
%
%   Inputs
%     state   - vehicle state (uses state.Pos and state.rot(3) for yaw)
%     hulls   - cell array of closed [E N] hulls of everything to stay out of
%               (e.g. this frame's cluster hulls + confirmed object hulls).
%               Entries with fewer than 3 points are ignored.
%     S       - search memory. Pass [] the first time, then pass back whatever
%               search returned last time.
%
%   Outputs
%     goal    - [N E] waypoint for waypointControl, or [] when the search is
%               complete and restartWhenDone is false
%     S       - updated search memory. Useful fields:
%                 S.status       "searching" or "complete"
%                 S.waypoints    m x 2 list of search waypoints [N E]
%                 S.current      index of the active waypoint (0 = none)
%                 S.done         1 x m, true once a waypoint is finished
%                 S.coverage     1 x m, fraction of each waypoint's patch seen
%                 S.seen         seen grid (rows = North, cols = East)
%                 S.cellN/cellE  cell-center coordinates for S.seen
%                 S.arriveRadius arrival distance used for the active waypoint
%                 S.passes       number of completed passes
%
%   How it works
%     1. Waypoints are laid on a grid over northRange x eastRange.
%     2. Every map cell within seenRadius of the boat AND inside the lidar
%        field of view (fov, centered on the bow) is marked "seen".
%     3. Each cell belongs to its nearest waypoint (its "patch"). A waypoint
%        is done when the boat reaches it OR its patch is mostly seen, so
%        ground covered in transit is not visited again.
%     4. The next waypoint is the nearest one not yet done.
%     5. If a waypoint sits inside/near a hull, the arrival distance grows so
%        the boat stops outside it (quick fix until the tangent bug planner
%        is in).
%
%   Usage in Autonomy.m:
%     % init block:  data.search = [];
%     [data.goal, data.search] = search(state, hulls, data.search);
%
%   NOTE: settings are read once on the first call. Restart the sim after
%   editing them.

% ---- First call: set up memory ----
if isempty(S)
    S = initSearch(defineSettings());
end
s = S.settings;

boatNE = [state.Pos(1) state.Pos(2)];   % [N E]

% ---- 1. Mark cells inside the lidar view cone as seen ----
dN = S.cellN - boatNE(1);
dE = S.cellE - boatNE(2);
inRange = dN .^ 2 + dE .^ 2 <= s.seenRadius ^ 2;
inFov   = abs(wrapToPi(atan2(dE, dN) - state.rot(3))) <= deg2rad(s.fov) / 2;
S.seen  = S.seen | (inRange & inFov);

% ---- 2. Coverage of each waypoint's patch ----
covered    = accumarray(S.owner(S.counted), double(S.seen(S.counted)), [S.nWp 1]);
S.coverage = (covered ./ S.patchCells)';
S.done     = S.done | S.coverage >= s.doneFraction;

% ---- 3. Is the active waypoint finished? ----
if S.current > 0
    wp = S.waypoints(S.current, :);
    S.arriveRadius = arrivalRadius(wp, hulls, s);
    if norm(wp - boatNE) < S.arriveRadius
        S.done(S.current) = true;
    end
    if S.done(S.current)
        S.current = 0;
    end
end

% ---- 4. Pick the next waypoint ----
if S.current == 0
    if all(S.done)
        S.passes = S.passes + 1;
        if s.restartWhenDone
            % Start a fresh pass (objects may have moved / been missed)
            S.seen(:) = false;
            S.done(:) = false;
        else
            S.status = "complete";
            S.arriveRadius = s.arriveRadius;
            goal = [];
            return
        end
    end

    open = find(~S.done);
    [~, i] = min(vecnorm(S.waypoints(open, :) - boatNE, 2, 2));
    S.current = open(i);
    S.arriveRadius = arrivalRadius(S.waypoints(S.current, :), hulls, s);
end

S.status = "searching";
goal = [S.waypoints(S.current, :) NaN];   % [N E yaw], no final yaw
end


%% ========================================================================
%  SETTINGS - every tuning number lives here
%  ========================================================================
function s = defineSettings()

% Search area (waypoint grid)
s.northRange = [10 50];          % m, first/last waypoint row (North)
s.eastRange  = [-20 20];         % m, first/last waypoint column (East)
s.spacing    = [20 20];          % m, [North East] target spacing between waypoints
                                 %    (rounded so the ranges' ends are always waypoints)

% Map / seen grid
s.mapNorth   = [0 60];           % m, map limits (North)
s.mapEast    = [-30 30];         % m, map limits (East)
s.cellSize   = 1.0;              % m, seen-grid resolution
s.seenRadius = 12;               % m, cells closer than this to the boat are "seen".
                                 %    Set to the range where your smallest object
                                 %    still clusters AND classifies.
s.fov        = 120;              % deg, lidar horizontal field of view (centered on
                                 %    the bow). 360 = sees all around.

% Waypoint completion
s.doneFraction = 0.95;           % patch this fraction seen = waypoint done
s.arriveRadius = 1.0;            % m, normal arrival distance (keep >= nav.CEP99)
s.objectMargin = 1.0;            % m, clearance kept from a hull when a
                                 %    waypoint lands on/near it

% After every waypoint is done
s.restartWhenDone = true;        % true: clear and search again; false: goal = []

end


%% ========================================================================
%  SETUP - waypoint grid, seen grid, and which cells belong to which waypoint
%  ========================================================================
function S = initSearch(s)
S.settings = s;

% ---- Waypoint grid [N E] ----
[northPts, dN] = gridPoints(s.northRange, s.spacing(1));
[eastPts,  dE] = gridPoints(s.eastRange,  s.spacing(2));
[WE, WN] = meshgrid(eastPts, northPts);
S.waypoints = [WN(:) WE(:)];
S.nWp = size(S.waypoints, 1);

% ---- Seen grid (rows = North, cols = East) ----
nCenters = (s.mapNorth(1) + s.cellSize / 2) : s.cellSize : s.mapNorth(2);
eCenters = (s.mapEast(1)  + s.cellSize / 2) : s.cellSize : s.mapEast(2);
[S.cellE, S.cellN] = meshgrid(eCenters, nCenters);
S.seen = false(size(S.cellN));

% ---- Patch ownership: each cell belongs to its nearest waypoint ----
S.owner = zeros(size(S.cellN));
best = inf(size(S.cellN));
for k = 1:S.nWp
    dk = (S.cellN - S.waypoints(k, 1)) .^ 2 + (S.cellE - S.waypoints(k, 2)) .^ 2;
    closer = dk < best;
    S.owner(closer) = k;
    best(closer)    = dk(closer);
end

% Only count cells inside the search area plus half a spacing around it
S.counted = S.cellN >= s.northRange(1) - dN / 2 & S.cellN <= s.northRange(2) + dN / 2 & ...
            S.cellE >= s.eastRange(1)  - dE / 2 & S.cellE <= s.eastRange(2)  + dE / 2;
S.patchCells = max(accumarray(S.owner(S.counted), 1, [S.nWp 1]), 1);

% ---- Progress ----
S.done         = false(1, S.nWp);
S.coverage     = zeros(1, S.nWp);
S.current      = 0;
S.arriveRadius = s.arriveRadius;
S.passes       = 0;
S.status       = "searching";

end


function [pts, step] = gridPoints(range, spacing)
% Evenly spaced points from range(1) to range(2), close to the requested
% spacing. A zero-width range (or spacing wider than it) gives one point.
n = max(1, round(diff(range) / spacing) + 1);
if n == 1
    pts  = mean(range);
    step = spacing;
else
    pts  = linspace(range(1), range(2), n);
    step = diff(range) / (n - 1);
end
end


%% ========================================================================
%  ARRIVAL DISTANCE - grow it when the waypoint is on/near a hull
%  ========================================================================
function r = arrivalRadius(wp, hulls, s)
% wp is [N E]; hulls are closed [E N] polygons.
% If the waypoint is inside a hull or within objectMargin of its edge, the
% arrival distance becomes (distance to the hull's farthest vertex + margin).
% A convex hull lies entirely within its farthest-vertex distance of the
% waypoint, so when arrival triggers the boat is still at least objectMargin
% from every point of the hull.
r  = s.arriveRadius;
pE = wp(2);
pN = wp(1);

for i = 1:numel(hulls)
    h = hulls{i};
    if size(h, 1) < 3
        continue
    end

    if inpolygon(pE, pN, h(:, 1), h(:, 2)) || ...
            distToEdges(pE, pN, h) < s.objectMargin
        farthest = max(hypot(h(:, 1) - pE, h(:, 2) - pN));
        r = max(r, farthest + s.objectMargin);
    end
end

end


function d = distToEdges(pE, pN, h)
% Shortest distance from point (pE, pN) to the edges of closed polygon h [E N].
aE = h(1:end-1, 1);  aN = h(1:end-1, 2);     % edge start points
bE = h(2:end,   1);  bN = h(2:end,   2);     % edge end points
eE = bE - aE;        eN = bN - aN;
len2 = max(eE .^ 2 + eN .^ 2, eps);
t = ((pE - aE) .* eE + (pN - aN) .* eN) ./ len2;   % projection onto each edge
t = min(max(t, 0), 1);
d = min(hypot(aE + t .* eE - pE, aN + t .* eN - pN));
end
