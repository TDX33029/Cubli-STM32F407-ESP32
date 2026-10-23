%% Attitude error vs. wheel speed / control voltage — one figure per chart.
%  The three charts are produced as independent figures, each with its own
%  animated GIF and final static PNG:
%    attitude_error_animated - tilt error, Pitch / Roll
%    wheel_speed_animated    - wheel speeds S1 / S2 / S3
%    control_voltage_animated- LQR control voltage U1 / U2 / U3
%  Curves are drawn one after another with an ease-out profile (fast at the
%  beginning, slowing towards the end).  Academic style: Times New Roman,
%  boxed axes, inward ticks, dashed grid, framed legend.
%
%  Run:  matlab -batch "cd('<this folder>'); plot_attitude_wheel_animated"

clear; clc; close all;

dataDir = fileparts(mfilename('fullpath'));
if isempty(dataDir), dataDir = pwd; end

% ----------------------------- configuration -----------------------------
FPS      = 12;
DT       = 0.04;
T_CURVE  = 1.6;
T_PAUSE  = 0.25;
T_HOLD   = 1.6;
EASE     = 2.0;

fontName = 'Times New Roman';
cP = [0.10 0.10 0.10];
cR = [0.00 0.35 0.75];
cW = [0.85 0.45 0.10; 0.00 0.35 0.75; 0.20 0.55 0.20];

% ------------------------------ read data --------------------------------
M = readTELE(fullfile(dataDir, 'corner_run1.csv'));
sel = M.bal == 3;
if ~any(sel), error('No Bal=3 (BALANCING) segment in the data'); end

t = M.t(sel); t = t - t(1);
errP = (M.pit - M.refpit); errP = errP(sel);
errR = (M.rol - M.refrol); errR = errR(sel);
S = [M.s1(sel), M.s2(sel), M.s3(sel)];
U = [M.u1(sel), M.u2(sel), M.u3(sel)];
dur = t(end);

% ------------------------------ metrics ----------------------------------
rmsP = rms(errP);      rmsR = rms(errR);
p2pP = max(errP) - min(errP);
p2pR = max(errR) - min(errR);
sEnd = S(end, :);
slope = zeros(1, 3);
for i = 1:3
    p = polyfit(t, S(:, i), 1);
    slope(i) = p(1);
end
uMean = mean(U, 1);

dt = 0.05;
tu = (0:dt:dur)';
peakUS = zeros(1, 3); lagUS = zeros(1, 3);
for i = 1:3
    uu = interp1(t, U(:, i), tu, 'linear');
    ss = interp1(t, S(:, i), tu, 'linear');
    [peakUS(i), lagUS(i)] = corrLag(uu, ss, dt, 2.0);
end

% ------------------- dense uniform grid, one entry/curve -----------------
[tu2, iu] = unique(t, 'stable');
tg = (0:DT:dur)';
rs = @(v) interp1(tu2, v(iu), tg, 'linear');

% ------------------------- shared figure helpers -------------------------
mkFig = @(pos) figure('Color', 'w', 'Position', pos, 'ToolBar', 'none', ...
                      'Name', 'Attitude vs. wheel speed');

% ================= panel 1: attitude error ===============================
fig = mkFig([80 80 900 500]);
ax = axes(fig); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
styleAx(ax, fontName, 11);
hP = plot(ax, NaN, NaN, '-', 'Color', cP, 'LineWidth', 1.4);
hR = plot(ax, NaN, NaN, '-', 'Color', cR, 'LineWidth', 1.4);
yline(ax, 0, '-', 'Color', [0.75 0.75 0.75], 'LineWidth', 0.9);
ylim(ax, [min([errP; errR]) - 0.10 * range([errP; errR]), ...
          max([errP; errR]) + 0.10 * range([errP; errR])]);
xlabel(ax, 'Time from balance onset (s)', 'FontName', fontName, 'FontSize', 11.5);
ylabel(ax, 'Tilt error (deg)', 'FontName', fontName, 'FontSize', 11.5);
title(ax, {'Attitude error during corner balancing', ...
           sprintf('RMS Pitch %.2f° / Roll %.2f°,  p-p %.2f° / %.2f°', rmsP, rmsR, p2pP, p2pR)}, ...
      'FontName', fontName, 'FontSize', 12, 'FontWeight', 'bold');
legend(ax, [hP hR], {'Pitch error', 'Roll error'}, 'Location', 'northeast', ...
       'FontName', fontName, 'FontSize', 9, 'Box', 'on');
xlim(ax, [0 dur]);

D1 = struct('Pit', rs(errP), 'Rol', rs(errR));
H1 = struct('Pit', hP, 'Rol', hR);
plan1 = buildPlan({'Pit', 'Rol'}, numel(tg), T_CURVE, T_PAUSE, T_HOLD, FPS, EASE);
upd1 = @(ph) updLines(ph, H1, D1, tg);
renderFrames(fig, plan1, upd1, ...
             fullfile(dataDir, 'attitude_error_animated.gif'), ...
             fullfile(dataDir, 'attitude_error_animated.png'), FPS);

% ================= panel 2: wheel speeds =================================
fig = mkFig([100 100 900 500]);
ax = axes(fig); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
styleAx(ax, fontName, 11);
hS = gobjects(1, 3);
for i = 1:3
    hS(i) = plot(ax, NaN, NaN, '-', 'Color', cW(i, :), 'LineWidth', 1.4);
end
yline(ax, 0, '-', 'Color', [0.75 0.75 0.75], 'LineWidth', 0.9);
ylim(ax, [min(S(:)) - 0.08 * range(S(:)), max(S(:)) + 0.08 * range(S(:))]);
xlabel(ax, 'Time from balance onset (s)', 'FontName', fontName, 'FontSize', 11.5);
ylabel(ax, 'Wheel speed (rad/s)', 'FontName', fontName, 'FontSize', 11.5);
title(ax, {'Wheel speeds during corner balancing', ...
           sprintf(['drift to %.0f / %.0f / %.0f rad/s in %.1f s,  mean slope ', ...
                    '%.2f / %.2f / %.2f rad/s^2 (continuous reaction torque)'], ...
                   sEnd(1), sEnd(2), sEnd(3), dur, slope(1), slope(2), slope(3))}, ...
      'FontName', fontName, 'FontSize', 12, 'FontWeight', 'bold');
legend(ax, hS, {'S1', 'S2', 'S3'}, 'Location', 'northeast', ...
       'FontName', fontName, 'FontSize', 9, 'Box', 'on');
xlim(ax, [0 dur]);

D2 = struct('S1', rs(S(:, 1)), 'S2', rs(S(:, 2)), 'S3', rs(S(:, 3)));
H2 = struct('S1', hS(1), 'S2', hS(2), 'S3', hS(3));
plan2 = buildPlan({'S1', 'S2', 'S3'}, numel(tg), T_CURVE, T_PAUSE, T_HOLD, FPS, EASE);
upd2 = @(ph) updLines(ph, H2, D2, tg);
renderFrames(fig, plan2, upd2, ...
             fullfile(dataDir, 'wheel_speed_animated.gif'), ...
             fullfile(dataDir, 'wheel_speed_animated.png'), FPS);

% ================= panel 3: control voltage ==============================
fig = mkFig([120 120 900 500]);
ax = axes(fig); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
styleAx(ax, fontName, 11);
hU = gobjects(1, 3);
for i = 1:3
    hU(i) = plot(ax, NaN, NaN, '-', 'Color', cW(i, :), 'LineWidth', 1.4);
end
yline(ax, 0, '-', 'Color', [0.75 0.75 0.75], 'LineWidth', 0.9);
ylim(ax, [min(U(:)) - 0.08 * range(U(:)), max(U(:)) + 0.08 * range(U(:))]);
xlabel(ax, 'Time from balance onset (s)', 'FontName', fontName, 'FontSize', 11.5);
ylabel(ax, 'Control voltage (V)', 'FontName', fontName, 'FontSize', 11.5);
title(ax, {'LQR control voltage during corner balancing', ...
           sprintf(['mean %.2f / %.2f / %.2f V;  U->S correlation %.2f / %.2f / %.2f, ', ...
                    'lag %.2f / %.2f / %.2f s'], ...
                   uMean(1), uMean(2), uMean(3), ...
                   peakUS(1), peakUS(2), peakUS(3), lagUS(1), lagUS(2), lagUS(3))}, ...
      'FontName', fontName, 'FontSize', 12, 'FontWeight', 'bold');
legend(ax, hU, {'U1', 'U2', 'U3'}, 'Location', 'northeast', ...
       'FontName', fontName, 'FontSize', 9, 'Box', 'on');
xlim(ax, [0 dur]);

D3 = struct('U1', rs(U(:, 1)), 'U2', rs(U(:, 2)), 'U3', rs(U(:, 3)));
H3 = struct('U1', hU(1), 'U2', hU(2), 'U3', hU(3));
plan3 = buildPlan({'U1', 'U2', 'U3'}, numel(tg), T_CURVE, T_PAUSE, T_HOLD, FPS, EASE);
upd3 = @(ph) updLines(ph, H3, D3, tg);
renderFrames(fig, plan3, upd3, ...
             fullfile(dataDir, 'control_voltage_animated.gif'), ...
             fullfile(dataDir, 'control_voltage_animated.png'), FPS);

% ------------------------------- summary ---------------------------------
fprintf('Balance segment: %.2f s, %d frames\n', dur, sum(sel));
fprintf('Attitude RMS: Pitch %.2f / Roll %.2f deg\n', rmsP, rmsR);
for i = 1:3
    fprintf('M%d: U->S correlation %.2f, lag %.2f s\n', i, peakUS(i), lagUS(i));
end

% ======================================================================
function styleAx(ax, fontName, fs)
set(ax, 'FontName', fontName, 'FontSize', fs, ...
        'TickDir', 'in', 'TickLength', [0.012 0.025], ...
        'XMinorTick', 'on', 'YMinorTick', 'on', ...
        'GridLineStyle', '--', 'GridColor', [0.72 0.72 0.72], ...
        'GridAlpha', 0.9, 'GridLineWidth', 0.55, 'Layer', 'bottom');
end

% ======================================================================
function plan = buildPlan(keys, n, Tc, Tp, Th, fps, easeExp)
plan = cell(0, 1);
for c = 1:numel(keys)
    kk = seqFor(n, Tc, fps, easeExp);
    plan = [plan; cellfun(@(k) struct('kind', 'curve', 'key', keys{c}, 'k', k), ...
                          num2cell(kk), 'uni', 0)];                 %#ok<AGROW>
    plan{end+1} = struct('kind', 'finish', 'key', keys{c});         %#ok<SAGROW>
    if c < numel(keys)
        plan = [plan; repmat({struct('kind', 'pause')}, round(Tp * fps), 1)]; %#ok<AGROW>
    end
end
plan = [plan; repmat({struct('kind', 'pause')}, round(Th * fps), 1)]; %#ok<AGROW>
end

% ======================================================================
function updLines(ph, H, D, tg)
switch ph.kind
    case 'curve'
        set(H.(ph.key), 'XData', tg(1:ph.k), 'YData', D.(ph.key)(1:ph.k));
    case 'finish'
        set(H.(ph.key), 'XData', tg, 'YData', D.(ph.key));
    otherwise
        % pause: nothing changes
end
end

% ======================================================================
function kk = seqFor(N, T, fps, easeExp)
nPh = max(2, round(T * fps));
u = (1:nPh)' / nPh;
kk = max(2, round((1 - (1 - u).^easeExp) * N));
kk(end) = N;
end

% ======================================================================
function renderFrames(fig, plan, upd, gifFile, pngFile, fps)
%RENDERFRAMES run the plan, capture a GIF, export the final static PNG.
dtF = 1 / fps;
haveGif = false;
fprintf('Writing %s (%d frames, %.1f s) ...\n', gifFile, numel(plan), numel(plan) * dtF);
for ii = 1:numel(plan)
    upd(plan{ii});
    drawnow;
    im = frame2im(getframe(fig));
    [A, map] = rgb2ind(im, 128);
    if ~haveGif
        imwrite(A, map, gifFile, 'gif', 'LoopCount', Inf, 'DelayTime', dtF);
        haveGif = true;
    else
        imwrite(A, map, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', dtF);
    end
end
exportgraphics(fig, pngFile, 'Resolution', 200);
close(fig);
fprintf('Saved:\n  %s\n  %s\n', gifFile, pngFile);
end

% ======================================================================
function [peak, lag] = corrLag(u, s, dt, maxLag)
n = round(maxLag / dt);
m = numel(u);
cc = zeros(2 * n + 1, 1);
lags = (-n:n) * dt;
for k = -n:n
    if k >= 0
        a = s(1 + k:m); b = u(1:m - k);
    else
        a = s(1:m + k); b = u(1 - k:m);
    end
    a = a - mean(a); b = b - mean(b);
    den = sqrt((a' * a) * (b' * b));
    if den > 0, cc(k + n + 1) = (a' * b) / den; end
end
[~, j] = max(abs(cc));
peak = cc(j); lag = lags(j);
end

% ======================================================================
function M = readTELE(fname)
lines = readlines(fname, 'Encoding', 'UTF-8');
keys = {'Pit','Rol','Yaw','RefPit','RefRol','Wx','Wy','Wz','S1','S2','S3','U1','U2','U3'};
fields = {'pit','rol','yaw','refpit','refrol','wx','wy','wz','s1','s2','s3','u1','u2','u3'};
M = struct(); M.t = zeros(0,1); M.bal = zeros(0,1);
for k = 1:numel(fields), M.(fields{k}) = zeros(0,1); end

framePat = '^([0-9.]+),([-0-9.]+),"(\$TELE,.*)"$';
for i = 1:numel(lines)
    tok = regexp(lines(i), framePat, 'tokens', 'once');
    if isempty(tok), continue; end
    body = tok{3};
    bal = regexp(body, 'Bal:([0-9]+)', 'tokens', 'once');
    if isempty(bal), continue; end
    vals = nan(1, numel(keys));
    ok = true;
    for j = 1:numel(keys)
        m = regexp(body, ['(?:^|,)', keys{j}, ':([-0-9.]+)'], 'tokens', 'once');
        if isempty(m), ok = false; break; end
        vals(j) = str2double(m{1});
    end
    if ~ok, continue; end
    M.t(end+1,1) = str2double(tok{2}) / 1000;
    M.bal(end+1,1) = str2double(bal{1});
    for j = 1:numel(fields), M.(fields{j})(end+1,1) = vals(j); end
end
end
