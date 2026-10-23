%% Phase portrait, animated — one figure per chart (corner balancing).
%  The two phase planes are produced as independent figures, each with its
%  own animated GIF and final static PNG:
%    phase_pitch_animated - Pitch phase plane (theta = Pit - target, omega = Wx)
%    phase_roll_animated  - Roll phase plane  (theta = Rol - target, omega = Wy)
%  Within each figure the time-coloured scatter and the smoothed early
%  trajectory grow together along the state history, with a moving marker
%  riding the trajectory head; after the trajectory completes, the direction
%  arrows, the 1-sigma envelope ellipse and the start/end markers appear.
%  The reveal uses an ease-out profile (fast at the beginning, slowing towards
%  the end).  Academic style (Times New Roman, boxed axes, inward ticks,
%  framed legend); no grid on the phase planes, following ../img/02.
%
%  Run:  matlab -batch "cd('<this folder>'); plot_phase_portrait_animated"

clear; clc; close all;

dataDir = fileparts(mfilename('fullpath'));
if isempty(dataDir), dataDir = pwd; end

% ----------------------------- configuration -----------------------------
FPS      = 15;
T_CURVE  = 4.5;
T_PAUSE  = 0.5;
T_HOLD   = 1.6;
EASE     = 2.0;

fontName = 'Times New Roman';
lineColor = [0.05 0.05 0.05];

% ------------------------------ read data --------------------------------
M = readTELE(fullfile(dataDir, 'corner_run1.csv'));
iBal = find(M.bal == 3, 1);
if isempty(iBal), error('No Bal=3 (BALANCING) segment in the data'); end
nb  = find(M.bal == 3, 1, 'last');
sel = iBal:nb;
t   = M.t(sel) - M.t(iBal);

thP = M.pit(sel) - M.refpit(sel);   % Pitch error (deg)
omP = M.wx(sel);                    % Pitch rate (deg/s)
thR = M.rol(sel) - M.refrol(sel);   % Roll error (deg)
omR = M.wy(sel);                    % Roll rate (deg/s)

sm  = @(v) movmean(v, 5);
thPs = sm(thP); omPs = sm(omP);
thRs = sm(thR); omRs = sm(omR);

earlyLim = 8;
earlyP = t <= earlyLim;

envP = envelopePath(thP, omP);
envR = envelopePath(thR, omR);
arrP = arrowSegs([thPs(earlyP), omPs(earlyP)]);
arrR = arrowSegs([thRs(earlyP), omRs(earlyP)]);

plan = buildPlan(numel(t), T_CURVE, T_PAUSE, T_HOLD, FPS, EASE);

% ================= panel 1: Pitch phase plane ============================
fig = figure('Color', 'w', 'Position', [80 80 860 620], 'ToolBar', 'none', ...
             'Name', 'Phase portrait');
ax = axes(fig); hold(ax, 'on'); box(ax, 'on');
set(ax, 'FontName', fontName, 'FontSize', 11, 'TickDir', 'in', ...
        'TickLength', [0.012 0.025], 'XMinorTick', 'on', 'YMinorTick', 'on', ...
        'Layer', 'bottom');
hSc = scatter(ax, NaN, NaN, 10, NaN, 'filled', 'MarkerFaceAlpha', 0.55);
hL  = plot(ax, NaN, NaN, '-', 'Color', lineColor, 'LineWidth', 1.2);
hHd = plot(ax, NaN, NaN, 'o', 'MarkerSize', 5, 'MarkerFaceColor', lineColor, ...
           'MarkerEdgeColor', 'w');
hAr = quiver(ax, arrP.X, arrP.Y, arrP.U, arrP.V, 0, 'Color', [0.15 0.15 0.15], ...
             'LineWidth', 0.8, 'MaxHeadSize', 1.2, 'Visible', 'off');
hEnv = plot(ax, NaN, NaN, '--', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.2);
hS = plot(ax, NaN, NaN, 'o', 'MarkerSize', 8, ...
          'MarkerFaceColor', [0.10 0.65 0.25], 'MarkerEdgeColor', 'k');
hE = plot(ax, NaN, NaN, 's', 'MarkerSize', 8, ...
          'MarkerFaceColor', [0.85 0.15 0.10], 'MarkerEdgeColor', 'k');
xline(ax, 0, ':', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.0);
yline(ax, 0, ':', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.0);
xlim(ax, [-1.2 1.2]); ylim(ax, [-12 12]);
xlabel(ax, '\theta = Pit - target (deg)', 'FontName', fontName, 'FontSize', 11.5);
ylabel(ax, '\omega = Wx (deg/s)', 'FontName', fontName, 'FontSize', 11.5);
title(ax, sprintf(['Pitch phase plane (corner balancing):  |\\theta|_{max} = %.2f°,  ', ...
                   '|\\omega|_{max} = %.1f °/s,  RMS \\theta = %.2f°,  RMS \\omega = %.2f °/s'], ...
                  max(abs(thP)), max(abs(omP)), rms(thP), rms(omP)), ...
      'FontName', fontName, 'FontSize', 11, 'FontWeight', 'bold');
legend(ax, [hS hE hEnv], {'Start (capture)', 'End (STOP)', '1\sigma envelope'}, ...
       'Location', 'northeast', 'FontName', fontName, 'FontSize', 9, 'Box', 'on');
cb = colorbar(ax);
cb.Label.String = 'Time (s)';
cb.Label.FontName = fontName;
colormap(ax, turbo(256));
clim(ax, [0, t(end)]);

upd = @(ph) updPhase(ph, hSc, hL, hHd, hAr, hEnv, hS, hE, ...
                     thP, omP, thPs, omPs, t, earlyP, envP, arrP);
renderFrames(fig, plan, upd, ...
             fullfile(dataDir, 'phase_pitch_animated.gif'), ...
             fullfile(dataDir, 'phase_pitch_animated.png'), FPS);

% ================= panel 2: Roll phase plane =============================
fig = figure('Color', 'w', 'Position', [100 100 860 620], 'ToolBar', 'none', ...
             'Name', 'Phase portrait');
ax = axes(fig); hold(ax, 'on'); box(ax, 'on');
set(ax, 'FontName', fontName, 'FontSize', 11, 'TickDir', 'in', ...
        'TickLength', [0.012 0.025], 'XMinorTick', 'on', 'YMinorTick', 'on', ...
        'Layer', 'bottom');
hSc = scatter(ax, NaN, NaN, 10, NaN, 'filled', 'MarkerFaceAlpha', 0.55);
hL  = plot(ax, NaN, NaN, '-', 'Color', lineColor, 'LineWidth', 1.2);
hHd = plot(ax, NaN, NaN, 'o', 'MarkerSize', 5, 'MarkerFaceColor', lineColor, ...
           'MarkerEdgeColor', 'w');
hAr = quiver(ax, arrR.X, arrR.Y, arrR.U, arrR.V, 0, 'Color', [0.15 0.15 0.15], ...
             'LineWidth', 0.8, 'MaxHeadSize', 1.2, 'Visible', 'off');
hEnv = plot(ax, NaN, NaN, '--', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.2);
hS = plot(ax, NaN, NaN, 'o', 'MarkerSize', 8, ...
          'MarkerFaceColor', [0.10 0.65 0.25], 'MarkerEdgeColor', 'k');
hE = plot(ax, NaN, NaN, 's', 'MarkerSize', 8, ...
          'MarkerFaceColor', [0.85 0.15 0.10], 'MarkerEdgeColor', 'k');
xline(ax, 0, ':', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.0);
yline(ax, 0, ':', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.0);
xlim(ax, [-1.2 1.2]); ylim(ax, [-12 12]);
xlabel(ax, '\theta = Rol - target (deg)', 'FontName', fontName, 'FontSize', 11.5);
ylabel(ax, '\omega = Wy (deg/s)', 'FontName', fontName, 'FontSize', 11.5);
title(ax, sprintf(['Roll phase plane (corner balancing):  |\\theta|_{max} = %.2f°,  ', ...
                   '|\\omega|_{max} = %.1f °/s,  RMS \\theta = %.2f°,  RMS \\omega = %.2f °/s'], ...
                  max(abs(thR)), max(abs(omR)), rms(thR), rms(omR)), ...
      'FontName', fontName, 'FontSize', 11, 'FontWeight', 'bold');
legend(ax, [hS hE hEnv], {'Start (capture)', 'End (STOP)', '1\sigma envelope'}, ...
       'Location', 'northeast', 'FontName', fontName, 'FontSize', 9, 'Box', 'on');
cb = colorbar(ax);
cb.Label.String = 'Time (s)';
cb.Label.FontName = fontName;
colormap(ax, turbo(256));
clim(ax, [0, t(end)]);

upd = @(ph) updPhase(ph, hSc, hL, hHd, hAr, hEnv, hS, hE, ...
                     thR, omR, thRs, omRs, t, earlyP, envR, arrR);
renderFrames(fig, plan, upd, ...
             fullfile(dataDir, 'phase_roll_animated.gif'), ...
             fullfile(dataDir, 'phase_roll_animated.png'), FPS);

% ------------------------------- summary ---------------------------------
fprintf('Balance segment: %.1f s, %d frames\n', t(end), numel(t));
fprintf('Pitch: |theta|max %.2f deg, RMS theta %.2f deg | Roll: %.2f / %.2f deg\n', ...
        max(abs(thP)), rms(thP), max(abs(thR)), rms(thR));

% ======================================================================
function plan = buildPlan(n, Tc, Tp, Th, fps, easeExp)
plan = cell(0, 1);
kk = seqFor(n, Tc, fps, easeExp);
plan = [plan; cellfun(@(k) struct('kind', 'curve', 'k', k), num2cell(kk), 'uni', 0)]; %#ok<AGROW>
plan{end+1} = struct('kind', 'finish');                                 %#ok<SAGROW>
plan = [plan; repmat({struct('kind', 'pause')}, round(Tp * fps), 1)];   %#ok<AGROW>
plan = [plan; repmat({struct('kind', 'pause')}, round(Th * fps), 1)];   %#ok<AGROW>
end

% ======================================================================
function updPhase(ph, hSc, hL, hHd, hAr, hEnv, hS, hE, ...
                  th, om, ths, oms, t, earlyP, env, arr)
%UPDPHASE apply one animation phase to a phase-plane panel.
switch ph.kind
    case 'curve'
        k = ph.k;
        set(hSc, 'XData', th(1:k), 'YData', om(1:k), 'CData', t(1:k));
        ke = min(k, nnz(earlyP));
        set(hL, 'XData', ths(1:ke), 'YData', oms(1:ke));
        set(hHd, 'XData', ths(k), 'YData', oms(k));
    case 'finish'
        set(hSc, 'XData', th, 'YData', om, 'CData', t);
        set(hL, 'XData', ths(earlyP), 'YData', oms(earlyP));
        set(hHd, 'XData', NaN, 'YData', NaN);
        set(hAr, 'Visible', 'on');
        set(hEnv, 'XData', env(1, :), 'YData', env(2, :));
        set(hS, 'XData', ths(1), 'YData', oms(1));
        set(hE, 'XData', ths(end), 'YData', oms(end));
    otherwise
        % pause: nothing changes
end
end

% ======================================================================
function a = arrowSegs(pts)
%ARROWSEGS direction arrows along a path, one per 8 samples.
iA = 1:8:size(pts, 1);
a = struct('X', pts(iA(1:end-1), 1), 'Y', pts(iA(1:end-1), 2), ...
           'U', pts(iA(2:end), 1) - pts(iA(1:end-1), 1), ...
           'V', pts(iA(2:end), 2) - pts(iA(1:end-1), 2));
end

% ======================================================================
function e = envelopePath(th, om)
%ENVELOPEPATH 1-sigma covariance ellipse path (2 x N).
mu = mean([th om], 1);
C  = cov([th om]);
[V, D] = eig(C);
ang = linspace(0, 2*pi, 120);
e = V * sqrt(D) * [cos(ang); sin(ang)] + mu';
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
keys = {'Pit','Rol','RefPit','RefRol','Wx','Wy','Bal'};
fields = {'pit','rol','refpit','refrol','wx','wy','bal'};
M = struct(); M.t = zeros(0,1);
for k = 1:numel(fields), M.(fields{k}) = zeros(0,1); end

framePat = '^([0-9.]+),([-0-9.]+),"(\$TELE,.*)"$';
for i = 1:numel(lines)
    tok = regexp(lines(i), framePat, 'tokens', 'once');
    if isempty(tok), continue; end
    body = tok{3};
    vals = nan(1, numel(keys));
    ok = true;
    for j = 1:numel(keys)
        m = regexp(body, ['(?:^|,)', keys{j}, ':([-0-9.]+)'], 'tokens', 'once');
        if isempty(m), ok = false; break; end
        vals(j) = str2double(m{1});
    end
    if ~ok, continue; end
    M.t(end+1,1) = str2double(tok{2}) / 1000;
    for j = 1:numel(fields), M.(fields{j})(end+1,1) = vals(j); end
end
end
