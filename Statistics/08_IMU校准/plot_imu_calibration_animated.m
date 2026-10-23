%% IMU calibration before/after yaw drift, animated overlay (two independent
%  reset-referenced 0-30 s recordings).
%  The two curves are drawn one after another into a single axes: first the
%  uncalibrated run, then the calibrated run.  Each curve is revealed with an
%  ease-out profile (fast at the beginning, slowing towards the end); after a
%  curve completes, its linear-fit drift line and the end-point drift label
%  appear.  Academic publication style (Times New Roman, boxed axes, inward
%  ticks, dashed grid, framed legend), following ../img references.
%
%  Outputs (written next to this script):
%    imu_calibration_animated.gif  - the animation
%    imu_calibration_animated.png  - final static overlay (all elements)
%
%  Run:  matlab -batch "cd('<this folder>'); plot_imu_calibration_animated"
%    or execute directly in the MATLAB desktop.

clear; clc; close all;

dataDir = fileparts(mfilename('fullpath'));
if isempty(dataDir), dataDir = pwd; end

% ----------------------------- configuration -----------------------------
FPS      = 20;      % animation frame rate
T_CURVE  = 5.0;     % drawing time per curve (s)
T_PAUSE  = 0.5;     % pause between curves (s)
T_HOLD   = 1.8;     % hold on the finished plot (s)
EASE     = 2.0;     % ease-out exponent: 1 = constant, larger = stronger deceleration
T_END    = 30;      % display window (s)
DT       = 0.05;    % resampled grid step (s)

C_UNCAL  = [0.776 0.157 0.157];   % deep red
C_CAL    = [0.180 0.490 0.196];   % deep green
fontName = 'Times New Roman';

% ------------------------------ read data --------------------------------
U = readTelemetry(fullfile(dataDir, 'uncal_30s.csv'));   % segment A, Cal=0
C = readTelemetry(fullfile(dataDir, 'cal_30s.csv'));     % segment B, Cal=1

selU = U.t >= 0 & U.cal == 0;
selC = C.t >= 0 & C.cal == 1;
tU = U.t(selU);  yU = U.yaw(selU);
tC = C.t(selC);  yC = C.yaw(selC);
if numel(tU) < 50 || numel(tC) < 50
    error('Not enough telemetry rows: uncalibrated %d, calibrated %d', numel(tU), numel(tC));
end

% zero both segments to their own first frame
xU = tU - tU(1);  yU = yU - yU(1);
xC = tC - tC(1);  yC = yC - yC(1);

% drop duplicate timestamps (retransmitted rows share the same t_rel_ms)
[xU, iu] = unique(xU, 'stable');  yU = yU(iu);
[xC, ic] = unique(xC, 'stable');  yC = yC(ic);

% drift rate: linear fit slope
pU = polyfit(xU, yU, 1);
pC = polyfit(xC, yC, 1);
ratio = abs(pU(1)) / max(abs(pC(1)), 1e-9);

% dense uniform grid for a smooth reveal
tg  = (0:DT:T_END)';
yUg = interp1(xU, yU, tg, 'linear', 'extrap');
yCg = interp1(xC, yC, tg, 'linear', 'extrap');

% ------------------------------- figure ----------------------------------
fig = figure('Name', 'IMU calibration yaw drift', 'Color', 'w', ...
             'Position', [100 100 980 500], 'ToolBar', 'none');
ax = axes(fig); hold(ax, 'on'); box(ax, 'on'); grid(ax, 'on');
set(ax, 'FontName', fontName, 'FontSize', 12, 'LineWidth', 0.9, ...
        'TickDir', 'in', 'TickLength', [0.012 0.025], ...
        'XMinorTick', 'on', 'YMinorTick', 'on');
ax.GridLineStyle = '--';
ax.GridColor     = [0.72 0.72 0.72];
ax.GridAlpha     = 0.9;
ax.GridLineWidth = 0.55;
ax.Layer         = 'bottom';

hZero = yline(ax, 0, '-');
set(hZero, 'Color', [0.75 0.75 0.75], 'LineWidth', 0.8);

hU  = plot(ax, NaN, NaN, '-',  'Color', C_UNCAL,       'LineWidth', 1.9);
hC  = plot(ax, NaN, NaN, '-',  'Color', C_CAL,         'LineWidth', 1.9);
hFU = plot(ax, NaN, NaN, '--', 'Color', C_UNCAL*0.72,  'LineWidth', 1.1);
hFC = plot(ax, NaN, NaN, '--', 'Color', C_CAL*0.72,    'LineWidth', 1.1);

hTU = text(ax, T_END, yUg(end), '', 'Color', C_UNCAL, 'FontName', fontName, ...
           'FontSize', 11, 'VerticalAlignment', 'bottom');
hTC = text(ax, T_END, yCg(end), '', 'Color', C_CAL,   'FontName', fontName, ...
           'FontSize', 11, 'VerticalAlignment', 'bottom');

xlim(ax, [0 T_END]);
ylim(ax, [-1 11]);
xlabel(ax, 'Time after reset (s)', 'FontName', fontName, 'FontSize', 13);
ylabel(ax, 'Yaw drift (deg)', 'FontName', fontName, 'FontSize', 13);
title(ax, {'IMU calibration: yaw drift overlay (0-30 s after reset)', ...
           sprintf('Drift suppressed %.0f\\times by CAL\\_IMU gyro-bias compensation', ratio)}, ...
      'FontName', fontName, 'FontSize', 12.5, 'FontWeight', 'bold');
legend(ax, [hU hC], ...
       {sprintf('Uncalibrated (Cal=0, %.2f °/s)', pU(1)), ...
        sprintf('Calibrated   (Cal=1, %.3f °/s)', pC(1))}, ...
       'Location', 'northwest', 'FontName', fontName, 'FontSize', 11, 'Box', 'on');

% --------------------------- animation plan ------------------------------
ease = @(u) 1 - (1 - u).^EASE;                 % fast -> slow drawing speed
nPh  = round(T_CURVE * FPS);
nU   = numel(tg);
kSeq = max(2, round(ease((1:nPh)' / nPh) * nU));   % grid-index progress per frame
kSeq(end) = nU;

plan = cell(0, 1);
addCurvePhase = @(which, kvec) ...              % append one curve phase
    cellfun(@(k) struct('kind', 'curve', 'which', which, 'k', k), ...
            num2cell(kvec), 'uni', 0);
plan = [plan; addCurvePhase('U', kSeq)];
plan{end+1} = struct('kind', 'finish', 'which', 'U');
plan = [plan; repmat({struct('kind', 'pause')}, round(T_PAUSE * FPS), 1)];
plan = [plan; addCurvePhase('C', kSeq)];
plan{end+1} = struct('kind', 'finish', 'which', 'C');
plan = [plan; repmat({struct('kind', 'pause')}, round(T_PAUSE * FPS), 1)];
plan = [plan; repmat({struct('kind', 'pause')}, round(T_HOLD  * FPS), 1)];

% ------------------------------ render loop ------------------------------
gifFile = fullfile(dataDir, 'imu_calibration_animated.gif');
pngFile = fullfile(dataDir, 'imu_calibration_animated.png');
dtF = 1 / FPS;
haveGif = false;

fprintf('Writing %s (%d frames, %.1f s) ...\n', gifFile, numel(plan), numel(plan) * dtF);
for ii = 1:numel(plan)
    ph = plan{ii};
    switch ph.kind
        case 'curve'
            if strcmp(ph.which, 'U')
                set(hU, 'XData', tg(1:ph.k), 'YData', yUg(1:ph.k));
            else
                set(hC, 'XData', tg(1:ph.k), 'YData', yCg(1:ph.k));
            end
        case 'finish'
            if strcmp(ph.which, 'U')
                set(hU,  'XData', tg, 'YData', yUg);
                set(hFU, 'XData', [0 T_END], 'YData', polyval(pU, [0 T_END]));
                set(hTU, 'String', sprintf('  %.1f°', yUg(end)));
            else
                set(hC,  'XData', tg, 'YData', yCg);
                set(hFC, 'XData', [0 T_END], 'YData', polyval(pC, [0 T_END]));
                set(hTC, 'String', sprintf('  %.1f°', yCg(end)));
            end
        otherwise
            % pause: nothing changes
    end
    drawnow;

    im = frame2im(getframe(fig));
    [A, map] = rgb2ind(im, 256);
    if ~haveGif
        imwrite(A, map, gifFile, 'gif', 'LoopCount', Inf, 'DelayTime', dtF);
        haveGif = true;
    else
        imwrite(A, map, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', dtF);
    end
end

% --------------------------- final static PNG ----------------------------
set(hU,  'XData', tg, 'YData', yUg);
set(hC,  'XData', tg, 'YData', yCg);
set(hFU, 'XData', [0 T_END], 'YData', polyval(pU, [0 T_END]));
set(hFC, 'XData', [0 T_END], 'YData', polyval(pC, [0 T_END]));
set(hTU, 'String', sprintf('  %.1f°', yUg(end)));
set(hTC, 'String', sprintf('  %.1f°', yCg(end)));
exportgraphics(fig, pngFile, 'Resolution', 300);

% ------------------------------- summary ---------------------------------
fprintf('Uncalibrated: %d frames, %.1f s, yaw drift %.1f deg (%.2f deg/s, %.1f deg/min)\n', ...
        numel(xU), xU(end), yU(end), pU(1), pU(1)*60);
fprintf('Calibrated  : %d frames, %.1f s, yaw drift %.1f deg (%.3f deg/s, %.1f deg/min)\n', ...
        numel(xC), xC(end), yC(end), pC(1), pC(1)*60);
fprintf('Drift suppression: %.0fx\n', ratio);
fprintf('Estimated gyro bias Bx/By/Bz: %+.2f / %+.2f / %+.2f deg/s\n', ...
        C.bx(end), C.by(end), C.bz(end));
fprintf('Saved:\n  %s\n  %s\n', gifFile, pngFile);

if ~batchMode()
    figure(fig);
end

% ======================================================================
function M = readTelemetry(fname)
%READTELEMETRY Parse $TELE CSV captured by imu_capture.py.
%   Row format: t_local_ms,t_rel_ms,"$TELE,...,Yaw:..,Bx:..,By:..,Bz:..,Cal:..,...#"
lines = readlines(fname, 'Encoding', 'UTF-8');
M.t = zeros(0,1); M.yaw = zeros(0,1); M.cal = zeros(0,1);
M.bx = zeros(0,1); M.by = zeros(0,1); M.bz = zeros(0,1);

framePat = '^([0-9.]+),([-0-9.]+),"(\$TELE,.*)"$';
for i = 1:numel(lines)
    tok = regexp(lines(i), framePat, 'tokens', 'once');
    if isempty(tok), continue; end
    body = tok{3};
    yaw = regexp(body, 'Yaw:([-0-9.]+)', 'tokens', 'once');
    cal = regexp(body, 'Cal:([0-9]+)',   'tokens', 'once');
    if isempty(yaw) || isempty(cal), continue; end
    bx = regexp(body, 'Bx:([-0-9.]+)', 'tokens', 'once');
    by = regexp(body, 'By:([-0-9.]+)', 'tokens', 'once');
    bz = regexp(body, 'Bz:([-0-9.]+)', 'tokens', 'once');

    M.t(end+1,1)   = str2double(tok{2}) / 1000;   % t_rel_ms -> s
    M.yaw(end+1,1) = str2double(yaw{1});
    M.cal(end+1,1) = str2double(cal{1});
    if ~isempty(bx), M.bx(end+1,1) = str2double(bx{1}); end
    if ~isempty(by), M.by(end+1,1) = str2double(by{1}); end
    if ~isempty(bz), M.bz(end+1,1) = str2double(bz{1}); end
end
end

function tf = batchMode()
%BATCHMODE True when running under matlab -batch (no desktop).
tf = usejava('desktop') == 0 || ~feature('ShowFigureWindows');
if isempty(tf) || ~islogical(tf), tf = false; end
end
