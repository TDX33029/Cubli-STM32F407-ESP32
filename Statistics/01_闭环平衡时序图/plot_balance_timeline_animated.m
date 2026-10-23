%% Closed-loop corner balancing timeline, animated — one figure per chart.
%  The three charts of the timeline are produced as independent figures,
%  each with its own animated GIF and final static PNG:
%    balance_attitude_animated  - attitude angles (Pitch/Roll left, Yaw right)
%    balance_rates_animated     - body angular rates
%    balance_wheels_animated    - wheel speeds
%  Curves are drawn one after another with an ease-out profile (fast at the
%  beginning, slowing towards the end).  Academic style: Times New Roman,
%  boxed axes, inward ticks, dashed grid, framed legend.
%
%  Run:  matlab -batch "cd('<this folder>'); plot_balance_timeline_animated"

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
cPit = [0.10 0.10 0.10];
cRol = [0.00 0.35 0.75];
cYaw = [0.55 0.55 0.55];
cWx  = [0.78 0.20 0.16];
cWy  = [0.18 0.49 0.20];
cWz  = [0.45 0.45 0.45];
cS   = [0.85 0.45 0.10; 0.00 0.35 0.75; 0.20 0.55 0.20];

% ------------------------------ read data --------------------------------
M = readTELE(fullfile(dataDir, 'corner_run1.csv'));
iBal = find(M.bal == 3, 1);
if isempty(iBal), error('No Bal=3 (BALANCING) segment in the data'); end

t0 = M.t(iBal);
x  = M.t - t0;
WIN = [-2.0, 26.0];
inWin = x >= WIN(1) & x <= WIN(2);

% ------------------------------ metrics ----------------------------------
errP = M.pit - M.refpit;
errR = M.rol - M.refrol;

settleT = NaN;
for i = iBal:numel(M.t)
    seg = i:min(i+4, numel(M.t));
    if numel(seg) >= 3 && all(abs(errP(seg)) < 1) && all(abs(errR(seg)) < 1)
        settleT = M.t(i) - t0;
        break;
    end
end

steady = find(M.t >= t0 + 1.0 & M.bal == 3);
rmsP = rms(errP(steady));  rmsR = rms(errR(steady));
p2pP = max(errP(steady)) - min(errP(steady));
p2pR = max(errR(steady)) - min(errR(steady));
dur  = M.t(steady(end)) - M.t(steady(1));
wheelPeak = max(abs([M.s1(steady); M.s2(steady); M.s3(steady)]));

% ------------------- dense uniform grid, one entry/curve -----------------
[xu, iu] = unique(x, 'stable');
tg = (WIN(1):DT:WIN(2))';
rs = @(v) interp1(xu, v(iu), tg, 'linear');
yPit = rs(M.pit);  yRol = rs(M.rol);  yYaw = rs(M.yaw);
yWx = rs(M.wx);  yWy = rs(M.wy);  yWz = rs(M.wz);
yS1 = rs(M.s1);  yS2 = rs(M.s2);  yS3 = rs(M.s3);

% ================= panel 1: attitude angles ==============================
fig = figure('Color', 'w', 'Position', [80 80 900 540], 'ToolBar', 'none', ...
             'Name', 'Corner balancing timeline');
ax = axes(fig); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
styleAx(ax, fontName, 11);
yyaxis(ax, 'left');
hPit = plot(ax, NaN, NaN, '-', 'Color', cPit, 'LineWidth', 1.5);
hRol = plot(ax, NaN, NaN, '-', 'Color', cRol, 'LineWidth', 1.5);
hRefP = yline(ax, M.refpit(iBal), 'k--', 'LineWidth', 1.0);
hRefR = yline(ax, M.refrol(iBal), '--', 'Color', cRol, 'LineWidth', 1.0);
ylim(ax, [20 55]);
ylabel(ax, 'Tilt angle (deg)', 'FontName', fontName, 'FontSize', 11.5);
ax.YAxis(1).Color = cPit;
yyaxis(ax, 'right');
hYaw = plot(ax, NaN, NaN, '-', 'Color', cYaw, 'LineWidth', 1.1);
ylim(ax, [min(M.yaw(inWin)) - 1, max(M.yaw(inWin)) + 1]);
ylabel(ax, 'Yaw (deg)', 'FontName', fontName, 'FontSize', 11.5);
ax.YAxis(2).Color = cYaw;
xline(ax, 0, '--', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.0);
title(ax, {'Attitude angles during corner balancing (BAUTO capture)', ...
           sprintf('settle %.1f s (±1°), steady RMS Pitch %.2f° / Roll %.2f°, p-p %.2f° / %.2f°', ...
                   settleT, rmsP, rmsR, p2pP, p2pR)}, ...
      'FontName', fontName, 'FontSize', 12, 'FontWeight', 'bold');
legend(ax, [hPit hRol hYaw hRefP hRefR], ...
       {'Pitch', 'Roll', 'Yaw (right)', 'Pitch target', 'Roll target'}, ...
       'Location', 'east', 'FontName', fontName, 'FontSize', 9, 'Box', 'on');
xlabel(ax, 'Time from balance onset (s)', 'FontName', fontName, 'FontSize', 11.5);
xlim(ax, WIN);
yyaxis(ax, 'left');

D1 = struct('Pit', yPit, 'Rol', yRol, 'Yaw', yYaw);
H1 = struct('Pit', hPit, 'Rol', hRol, 'Yaw', hYaw);
plan1 = buildPlan({'Pit', 'Rol', 'Yaw'}, numel(tg), T_CURVE, T_PAUSE, T_HOLD, FPS, EASE);
upd1 = @(ph) updLines(ph, H1, D1, tg);
renderFrames(fig, plan1, upd1, ...
             fullfile(dataDir, 'balance_attitude_animated.gif'), ...
             fullfile(dataDir, 'balance_attitude_animated.png'), FPS);

% ================= panel 2: body angular rates ===========================
fig = figure('Color', 'w', 'Position', [100 100 900 500], 'ToolBar', 'none', ...
             'Name', 'Corner balancing timeline');
ax = axes(fig); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
styleAx(ax, fontName, 11);
hWx = plot(ax, NaN, NaN, '-', 'Color', cWx, 'LineWidth', 1.3);
hWy = plot(ax, NaN, NaN, '-', 'Color', cWy, 'LineWidth', 1.3);
hWz = plot(ax, NaN, NaN, '-', 'Color', cWz, 'LineWidth', 1.2);
yline(ax, 0, '-', 'Color', [0.82 0.82 0.82], 'LineWidth', 0.8);
xline(ax, 0, '--', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.0);
ylim(ax, [-1.05, 1.05] * max(abs([M.wx(inWin); M.wy(inWin); M.wz(inWin)])));
xlabel(ax, 'Time from balance onset (s)', 'FontName', fontName, 'FontSize', 11.5);
ylabel(ax, 'Angular rate (deg/s)', 'FontName', fontName, 'FontSize', 11.5);
title(ax, 'Body angular rates during corner balancing (IMU-calibrated, bias removed)', ...
      'FontName', fontName, 'FontSize', 12, 'FontWeight', 'bold');
legend(ax, [hWx hWy hWz], {'\omega_x', '\omega_y', '\omega_z'}, ...
       'Location', 'east', 'FontName', fontName, 'FontSize', 9, 'Box', 'on');
xlim(ax, WIN);

D2 = struct('Wx', yWx, 'Wy', yWy, 'Wz', yWz);
H2 = struct('Wx', hWx, 'Wy', hWy, 'Wz', hWz);
plan2 = buildPlan({'Wx', 'Wy', 'Wz'}, numel(tg), T_CURVE, T_PAUSE, T_HOLD, FPS, EASE);
upd2 = @(ph) updLines(ph, H2, D2, tg);
renderFrames(fig, plan2, upd2, ...
             fullfile(dataDir, 'balance_rates_animated.gif'), ...
             fullfile(dataDir, 'balance_rates_animated.png'), FPS);

% ================= panel 3: wheel speeds =================================
fig = figure('Color', 'w', 'Position', [120 120 900 500], 'ToolBar', 'none', ...
             'Name', 'Corner balancing timeline');
ax = axes(fig); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
styleAx(ax, fontName, 11);
hS1 = plot(ax, NaN, NaN, '-', 'Color', cS(1, :), 'LineWidth', 1.3);
hS2 = plot(ax, NaN, NaN, '-', 'Color', cS(2, :), 'LineWidth', 1.3);
hS3 = plot(ax, NaN, NaN, '-', 'Color', cS(3, :), 'LineWidth', 1.3);
yline(ax, 0, '-', 'Color', [0.82 0.82 0.82], 'LineWidth', 0.8);
xline(ax, 0, '--', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.0);
ylim(ax, [-1.08, 1.08] * max(abs([M.s1(inWin); M.s2(inWin); M.s3(inWin)])));
xlabel(ax, 'Time from balance onset (s)', 'FontName', fontName, 'FontSize', 11.5);
ylabel(ax, 'Wheel speed (rad/s)', 'FontName', fontName, 'FontSize', 11.5);
title(ax, {'Wheel speeds during corner balancing', ...
           sprintf('steady peak %.0f rad/s, balancing for %.1f s', wheelPeak, dur)}, ...
      'FontName', fontName, 'FontSize', 12, 'FontWeight', 'bold');
legend(ax, [hS1 hS2 hS3], {'S1', 'S2', 'S3'}, 'Location', 'east', ...
       'FontName', fontName, 'FontSize', 9, 'Box', 'on');
xlim(ax, WIN);

D3 = struct('S1', yS1, 'S2', yS2, 'S3', yS3);
H3 = struct('S1', hS1, 'S2', hS2, 'S3', hS3);
plan3 = buildPlan({'S1', 'S2', 'S3'}, numel(tg), T_CURVE, T_PAUSE, T_HOLD, FPS, EASE);
upd3 = @(ph) updLines(ph, H3, D3, tg);
renderFrames(fig, plan3, upd3, ...
             fullfile(dataDir, 'balance_wheels_animated.gif'), ...
             fullfile(dataDir, 'balance_wheels_animated.png'), FPS);

% ------------------------------- summary ---------------------------------
fprintf('Balance onset: t=%.1f s (from capture start), duration %.1f s\n', t0, dur);
fprintf('Settling (±1 deg): %.1f s; steady RMS Pitch %.2f / Roll %.2f deg\n', settleT, rmsP, rmsR);

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
%BUILDPLAN sequential ease-out reveal plan for a list of curves.
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
%UPDLINES apply one animation phase to a set of data lines.
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
%SEQFOR ease-out reveal index sequence over N points in T seconds.
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
