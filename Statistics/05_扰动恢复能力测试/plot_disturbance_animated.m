%% Disturbance recovery (three manual kicks), animated — one figure per chart.
%  The five charts are produced as independent figures, each with its own
%  animated GIF and final static PNG:
%    kick_timeline_animated - full-run tilt error (Pitch / Roll) + kick markers
%    kick_recovery_animated - the three kick-aligned recovery curves K1-K3,
%                             each recovery marker appearing on completion
%    kick_maxdev_animated   - bar chart, max deviation per kick
%    kick_rectime_animated  - bar chart, recovery time per kick
%    kick_osc_animated      - bar chart, secondary oscillations per kick
%  Curves/bars are drawn one after another with an ease-out profile (fast at
%  the beginning, slowing towards the end).  Academic style: Times New Roman,
%  boxed axes, inward ticks, dashed grid, framed legend.
%
%  Run:  matlab -batch "cd('<this folder>'); plot_disturbance_animated"

clear; clc; close all;

dataDir = fileparts(mfilename('fullpath'));
if isempty(dataDir), dataDir = pwd; end

% ----------------------------- configuration -----------------------------
FPS      = 12;
DT       = 0.04;
T_CURVE  = 1.8;
T_PAUSE  = 0.3;
T_BAR    = 0.45;
T_HOLD   = 1.4;
EASE     = 2.0;

fontName = 'Times New Roman';
evCol = [0.00 0.35 0.75; 0.80 0.30 0.10; 0.20 0.55 0.20];

% ------------------------------ read data --------------------------------
M = readTELE(fullfile(dataDir, 'kick_run1.csv'));
sel = M.bal == 3;
if ~any(sel), error('No Bal=3 (BALANCING) segment in the data'); end

t = M.t(sel); t = t - t(1);
eP = M.pit(sel) - M.refpit(sel);
eR = M.rol(sel) - M.refrol(sel);
tilt = hypot(eP, eR);
W = [M.wx(sel), M.wy(sel), M.wz(sel)];
wn = sqrt(sum(W.^2, 2));
sm = movmean(tilt, 5);

% ---------------------------- event detection ----------------------------
rateThr = max(18, 4 * median(wn));
idx = find(wn >= rateThr);
events = [];
for i = 1:numel(idx)
    if isempty(events) || t(idx(i)) - t(events(end)) > 2.5
        events(end+1) = idx(i); %#ok<AGROW>
    end
end
if numel(events) < 3
    error('Only %d kicks detected, need 3', numel(events));
end
if numel(events) > 3
    pk = zeros(numel(events), 1);
    for i = 1:numel(events)
        w = t >= t(events(i)) - 1.0 & t <= t(events(i)) + 1.0;
        pk(i) = max(wn(w));
    end
    [~, ord] = sort(pk, 'descend');
    events = events(sort(ord(1:3)));
end
tp = t(events);
nEv = numel(tp);

% ------------------------------- metrics ---------------------------------
peakDev = zeros(nEv, 1); recTime = zeros(nEv, 1); oscCount = zeros(nEv, 1);
ratePeak = zeros(nEv, 1);
for k = 1:nEv
    pre = t >= tp(k) - 2.0 & t <= tp(k) - 0.2;
    recThr = mean(sm(pre)) + max(0.25, 2 * std(sm(pre)));
    post = t >= tp(k) & t <= tp(k) + 2.5;
    [peakDev(k), ip0] = max(tilt(post));
    ip = find(post, 1) + ip0 - 1;
    ratePeak(k) = max(wn(t >= tp(k) & t <= tp(k) + 1.0));

    recTime(k) = NaN;
    for j = ip:numel(t)
        seg = t >= t(j) & t <= t(j) + 0.5;
        if nnz(seg) >= 4 && all(sm(seg) < recThr)
            recTime(k) = t(j) - tp(k);
            break;
        end
    end
    if isnan(recTime(k)), recTime(k) = 5.0; end

    oscCount(k) = 0; lastPeak = -inf;
    for j = ip + 2:numel(t) - 1
        if t(j) > tp(k) + recTime(k), break; end
        if sm(j) > sm(j-1) && sm(j) >= sm(j+1) && sm(j) > recThr
            lo = max(1, j - 8); hi = min(numel(t), j + 8);
            if sm(j) - min(sm(lo:hi)) >= 0.08 && t(j) - lastPeak > 0.35
                oscCount(k) = oscCount(k) + 1;
                lastPeak = t(j);
            end
        end
    end
end
dur = t(end);

% ------------------- dense grids for the two time plots ------------------
[tu, iu] = unique(t, 'stable');
tg = (0:DT:dur)';
rs = @(v) interp1(tu, v(iu), tg, 'linear');
yPg = rs(eP); yRg = rs(eR);

tau = (-1.5:DT:5.0)';
Kty = cell(nEv, 1); Kwy = cell(nEv, 1);
for k = 1:nEv
    win = t >= tp(k) - 1.5 & t <= tp(k) + 5.0;
    Kty{k} = interp1(t(win) - tp(k), tilt(win), tau);
end

% ================= panel 1: full-run tilt error ==========================
fig = figure('Color', 'w', 'Position', [80 80 920 500], 'ToolBar', 'none', ...
             'Name', 'Disturbance recovery');
ax = axes(fig); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
styleAx(ax, fontName, 10.5);
hP = plot(ax, NaN, NaN, '-', 'Color', [0.10 0.10 0.10], 'LineWidth', 1.2);
hR = plot(ax, NaN, NaN, '-', 'Color', [0.00 0.35 0.75], 'LineWidth', 1.2);
yline(ax, 0, '-', 'Color', [0.75 0.75 0.75], 'LineWidth', 0.9);
for k = 1:nEv
    xline(ax, tp(k), '--', 'Color', evCol(k, :), 'LineWidth', 1.1);
end
yl1 = [-1.10, 1.10] * max(abs([eP; eR]));
ylim(ax, [yl1(1), yl1(2) + 1.0]);
for k = 1:nEv
    text(ax, tp(k), yl1(2) + 0.5, sprintf('K%d', k), 'Color', evCol(k, :), ...
         'HorizontalAlignment', 'center', 'FontName', fontName, ...
         'FontSize', 9, 'FontWeight', 'bold');
end
xlim(ax, [0 dur]);
xlabel(ax, 'Time from balance onset (s)', 'FontName', fontName, 'FontSize', 11.5);
ylabel(ax, 'Tilt error (deg)', 'FontName', fontName, 'FontSize', 11.5);
title(ax, {'Full-run tilt error with three manual kicks (BAUTO corner balancing)', ...
           sprintf('kicks at %s s, rate peaks %s deg/s, balancing for %.1f s', ...
                   mat2str(round(tp', 1)), mat2str(round(ratePeak', 0)), dur)}, ...
      'FontName', fontName, 'FontSize', 12, 'FontWeight', 'bold');
legend(ax, [hP hR], {'Pitch error', 'Roll error'}, 'Location', 'northeast', ...
       'FontName', fontName, 'FontSize', 9, 'Box', 'on');

D1 = struct('Pit', yPg, 'Rol', yRg);
H1 = struct('Pit', hP, 'Rol', hR);
plan1 = buildKeyPlan({'Pit', 'Rol'}, numel(tg), T_CURVE, T_PAUSE, T_HOLD, FPS, EASE);
upd1 = @(ph) updLines(ph, H1, D1, tg);
renderFrames(fig, plan1, upd1, ...
             fullfile(dataDir, 'kick_timeline_animated.gif'), ...
             fullfile(dataDir, 'kick_timeline_animated.png'), FPS);

% ================= panel 2: kick-aligned recovery ========================
fig = figure('Color', 'w', 'Position', [100 100 860 500], 'ToolBar', 'none', ...
             'Name', 'Disturbance recovery');
ax = axes(fig); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
styleAx(ax, fontName, 10.5);
fill(ax, [-1.5 5 5 -1.5], [0.5 0.5 0.8 0.8], [0.85 0.85 0.85], ...
     'FaceAlpha', 0.25, 'EdgeColor', 'none');
hK = gobjects(1, nEv);
hM = gobjects(1, nEv);
for k = 1:nEv
    hK(k) = plot(ax, NaN, NaN, '-', 'Color', evCol(k, :), 'LineWidth', 1.4);
    hM(k) = plot(ax, NaN, NaN, 'o', 'Color', evCol(k, :), ...
                 'MarkerFaceColor', 'w', 'MarkerSize', 7, 'LineWidth', 1.4);
end
xline(ax, 0, '--', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.0);
xlim(ax, [-1.5 5.0]);
allK = vertcat(Kty{:});
ylim(ax, [min(allK) - 0.08 * range(allK), max(allK) + 0.08 * range(allK)]);
xlabel(ax, 'Time after kick \tau (s)', 'FontName', fontName, 'FontSize', 11.5);
ylabel(ax, 'Deviation magnitude (deg)', 'FontName', fontName, 'FontSize', 11.5);
title(ax, {'Kick-aligned recovery of the tilt deviation', ...
           sprintf(['max deviation %.2f / %.2f / %.2f°,  recovery %.2f / %.2f / %.2f s, ', ...
                    'oscillations %d / %d / %d'], ...
                   peakDev(1), peakDev(2), peakDev(3), ...
                   recTime(1), recTime(2), recTime(3), ...
                   oscCount(1), oscCount(2), oscCount(3))}, ...
      'FontName', fontName, 'FontSize', 12, 'FontWeight', 'bold');
legend(ax, hK, arrayfun(@(k) sprintf('K%d', k), 1:nEv, 'UniformOutput', false), ...
       'Location', 'northeast', 'FontName', fontName, 'FontSize', 9, 'Box', 'on');
text(ax, 4.9, 0.65, 'pre-kick envelope', 'FontName', fontName, 'FontSize', 8, ...
     'HorizontalAlignment', 'right', 'Color', [0.40 0.40 0.40]);

upd2 = @(ph) updRecovery(ph, hK, hM, tau, Kty, recTime);
plan2 = buildPlan(nEv, numel(tau), T_CURVE, T_PAUSE, T_HOLD, FPS, EASE);
renderFrames(fig, plan2, upd2, ...
             fullfile(dataDir, 'kick_recovery_animated.gif'), ...
             fullfile(dataDir, 'kick_recovery_animated.png'), FPS);

% ================= panels 3-5: metric bar charts =========================
metricVal = {peakDev, recTime, oscCount};
metricTag = {'kick_maxdev', 'kick_rectime', 'kick_osc'};
metricYlab = {'Max deviation (deg)', 'Recovery time (s)', 'Oscillation count'};
metricTtl = {'Maximum tilt deviation per kick', ...
             'Recovery time per kick (back to pre-kick envelope)', ...
             'Secondary oscillations per kick'};
metricFmt = {'%.2f', '%.2f', '%d'};
for c = 1:3
    fig = figure('Color', 'w', 'Position', [120 + 20*c 120 + 20*c 620 470], ...
                 'ToolBar', 'none', 'Name', 'Disturbance recovery');
    ax = axes(fig); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    styleAx(ax, fontName, 10);
    vals = metricVal{c};
    hB = gobjects(1, nEv);
    hL = gobjects(1, nEv);
    for j = 1:nEv
        hB(j) = bar(ax, j, 0, 0.55, 'FaceColor', evCol(j, :), ...
                    'EdgeColor', [0.25 0.25 0.25], 'LineWidth', 0.6);
        hL(j) = text(ax, j, 0, '', 'HorizontalAlignment', 'center', ...
                     'FontName', fontName, 'FontSize', 9.5);
    end
    xlim(ax, [0.4 nEv + 0.6]);
    xticks(ax, 1:nEv);
    xticklabels(ax, arrayfun(@(j) sprintf('K%d', j), 1:nEv, 'UniformOutput', false));
    ylim(ax, [0 max(vals) * 1.25 + eps]);
    if c == 3
        ylim(ax, [0 max(oscCount) + 1]);
        yticks(ax, 0:ceil(max(oscCount)));
    end
    xlabel(ax, 'Kick', 'FontName', fontName, 'FontSize', 11);
    ylabel(ax, metricYlab{c}, 'FontName', fontName, 'FontSize', 11);
    title(ax, metricTtl{c}, 'FontName', fontName, 'FontSize', 12, 'FontWeight', 'bold');

    plan3 = buildBarPlan(nEv, T_BAR, T_HOLD, FPS, EASE);
    upd3 = @(ph) updBars(ph, hB, hL, vals, metricFmt{c});
    renderFrames(fig, plan3, upd3, ...
                 fullfile(dataDir, [metricTag{c} '_animated.gif']), ...
                 fullfile(dataDir, [metricTag{c} '_animated.png']), FPS);
end

% ------------------------------- summary ---------------------------------
fprintf('Balance segment %.2f s, %d kicks detected\n', dur, nEv);
for k = 1:nEv
    fprintf('K%d: t=%.2f s, max deviation %.2f deg, recovery %.2f s, oscillations %d\n', ...
            k, tp(k), peakDev(k), recTime(k), oscCount(k));
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
function plan = buildKeyPlan(keys, n, Tc, Tp, Th, fps, easeExp)
%BUILDKEYPLAN sequential reveal plan keyed by curve name.
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
function plan = buildPlan(nCurves, n, Tc, Tp, Th, fps, easeExp)
%BUILDPLAN sequential reveal plan for nCurves indexed curves over n points.
plan = cell(0, 1);
for c = 1:nCurves
    kk = seqFor(n, Tc, fps, easeExp);
    plan = [plan; cellfun(@(k) struct('kind', 'curve', 'idx', c, 'k', k), ...
                          num2cell(kk), 'uni', 0)];                 %#ok<AGROW>
    plan{end+1} = struct('kind', 'finish', 'idx', c);               %#ok<SAGROW>
    if c < nCurves
        plan = [plan; repmat({struct('kind', 'pause')}, round(Tp * fps), 1)]; %#ok<AGROW>
    end
end
plan = [plan; repmat({struct('kind', 'pause')}, round(Th * fps), 1)]; %#ok<AGROW>
end

% ======================================================================
function plan = buildBarPlan(nBars, Tb, Th, fps, easeExp)
%BUILDBARPLAN plan growing nBars one after another.
plan = cell(0, 1);
kBar = seqFor(1000, Tb, fps, easeExp) / 1000;
for j = 1:nBars
    for q = 1:numel(kBar)
        plan{end+1, 1} = struct('kind', 'bar', 'idx', j, 'v', kBar(q)); %#ok<SAGROW>
    end
    plan{end+1, 1} = struct('kind', 'bfinish', 'idx', j);              %#ok<SAGROW>
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
function updRecovery(ph, hK, hM, tau, Kty, recTime)
switch ph.kind
    case 'curve'
        set(hK(ph.idx), 'XData', tau(1:ph.k), 'YData', Kty{ph.idx}(1:ph.k));
    case 'finish'
        set(hK(ph.idx), 'XData', tau, 'YData', Kty{ph.idx});
        set(hM(ph.idx), 'XData', recTime(ph.idx), ...
            'YData', interp1(tau, Kty{ph.idx}, recTime(ph.idx)));
    otherwise
        % pause: nothing changes
end
end

% ======================================================================
function updBars(ph, hB, hL, vals, fmt)
switch ph.kind
    case 'bar'
        set(hB(ph.idx), 'YData', ph.v * vals(ph.idx));
    case 'bfinish'
        set(hB(ph.idx), 'YData', vals(ph.idx));
        set(hL(ph.idx), 'String', sprintf(fmt, vals(ph.idx)), ...
            'Position', [ph.idx, vals(ph.idx) + 0.03 * max(ylim(hB(ph.idx).Parent)), 0]);
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
keys = {'Bal','Pit','Rol','RefPit','RefRol','Wx','Wy','Wz','S1','S2','S3','U1','U2','U3'};
fields = {'bal','pit','rol','refpit','refrol','wx','wy','wz','s1','s2','s3','u1','u2','u3'};
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
