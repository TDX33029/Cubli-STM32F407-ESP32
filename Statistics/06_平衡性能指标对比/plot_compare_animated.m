%% Balancing performance comparison of three LQR gain sets — one figure per chart.
%  The nine charts are produced as independent figures, each with its own
%  animated GIF and final static PNG:
%    compare_tilt_base/low/high_animated - tilt-deviation timeline per scheme
%    compare_rms_animated                - bar chart, RMS tilt
%    compare_maxtilt_animated            - bar chart, max tilt
%    compare_recovery_animated           - bar chart, median recovery time
%    compare_settle_animated             - bar chart, initial settling time
%    compare_wheelpeak_animated          - bar chart, wheel speed peak
%    compare_voltage_animated            - bar chart, voltage peak (+ 6 V limit)
%  Curves/bars are drawn one after another with an ease-out profile (fast at
%  the beginning, slowing towards the end).  Academic style: Times New Roman,
%  boxed axes, inward ticks, dashed grid, framed legend.
%
%  Run:  matlab -batch "cd('<this folder>'); plot_compare_animated"

clear; clc; close all;

dataDir = fileparts(mfilename('fullpath'));
if isempty(dataDir), dataDir = pwd; end

% ----------------------------- configuration -----------------------------
FPS      = 12;
DT       = 0.04;
T_CURVE  = 2.0;
T_PAUSE  = 0.3;
T_BAR    = 0.4;
T_HOLD   = 1.4;
EASE     = 2.0;

fontName = 'Times New Roman';
schemes = {'base', 'low', 'high'};
labels  = {'Baseline 80/5', 'Low gain 60/4', 'High gain 110/7'};
shortL  = {'Base', 'Low', 'High'};
colors  = [0.00 0.35 0.75; 0.78 0.30 0.10; 0.20 0.55 0.20];
nS = numel(schemes);

% --------------------------- metrics per scheme --------------------------
met = struct('rmsTilt', [], 'maxTilt', [], 'recMed', [], 'settle', [], ...
             'wheelPeak', [], 'uPeak', [], 'nevents', []);
D = struct('t', {}, 'y', {});
for s = 1:nS
    M = readTELE(fullfile(dataDir, ['compare_' schemes{s} '.csv']));
    sel = M.bal == 3;
    t = M.t(sel); t = t - t(1);
    eP = M.pit(sel) - M.refpit(sel);
    eR = M.rol(sel) - M.refrol(sel);
    tilt = hypot(eP, eR);
    wn = sqrt(M.wx(sel).^2 + M.wy(sel).^2 + M.wz(sel).^2);
    U = [M.u1(sel), M.u2(sel), M.u3(sel)];
    S = [M.s1(sel), M.s2(sel), M.s3(sel)];
    sm = movmean(tilt, 5);

    met.rmsTilt(s) = sqrt(mean(tilt.^2));
    met.maxTilt(s) = max(tilt);
    met.wheelPeak(s) = max(abs(S), [], 'all');
    met.uPeak(s) = max(abs(U), [], 'all');

    met.settle(s) = NaN;
    for k = 1:numel(t)
        seg = t >= t(k) & t <= t(k) + 1.0;
        if nnz(seg) >= 4 && all(sm(seg) < 0.5)
            met.settle(s) = t(k);
            break;
        end
    end
    if isnan(met.settle(s)), met.settle(s) = t(end); end

    [ev, recs] = detectEvents(t, wn, tilt, sm);
    met.nevents(s) = numel(ev);
    met.recMed(s) = median(recs);

    [tu, iu] = unique(t, 'stable');
    D(s).t = (0:DT:t(end))';
    D(s).y = interp1(tu, tilt(iu), D(s).t, 'linear'); %#ok<SAGROW>
end

% ================= panels 1-3: tilt-deviation timelines ==================
for s = 1:nS
    fig = figure('Color', 'w', 'Position', [80 + 15*s 80 + 15*s 820 450], ...
                 'ToolBar', 'none', 'Name', 'LQR performance comparison');
    ax = axes(fig); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    styleAx(ax, fontName, 10.5);
    hC = plot(ax, NaN, NaN, '-', 'Color', colors(s, :), 'LineWidth', 1.2);
    yline(ax, 0.5, ':', 'Color', [0.65 0.65 0.65], 'LineWidth', 0.8);
    ylim(ax, [0 5.5]);
    xlim(ax, [0 30]);
    xlabel(ax, 'Time from balance onset (s)', 'FontName', fontName, 'FontSize', 11.5);
    ylabel(ax, 'Tilt deviation (deg)', 'FontName', fontName, 'FontSize', 11.5);
    title(ax, sprintf('%s: tilt deviation over 30 s of balancing (RMS %.2f°, max %.2f°)', ...
          labels{s}, met.rmsTilt(s), met.maxTilt(s)), ...
          'FontName', fontName, 'FontSize', 11.5, 'FontWeight', 'bold');

    plan = buildPlan(1, numel(D(s).t), T_CURVE, T_PAUSE, T_HOLD, FPS, EASE);
    upd = @(ph) updSingle(ph, hC, D(s).t, D(s).y);
    renderFrames(fig, plan, upd, ...
                 fullfile(dataDir, sprintf('compare_tilt_%s_animated.gif', schemes{s})), ...
                 fullfile(dataDir, sprintf('compare_tilt_%s_animated.png', schemes{s})), FPS);
end

% ================= panels 4-9: metric bar charts =========================
metricTag = {'compare_rms', 'compare_maxtilt', 'compare_recovery', ...
             'compare_settle', 'compare_wheelpeak', 'compare_voltage'};
metricVal = {met.rmsTilt, met.maxTilt, met.recMed, ...
             met.settle, met.wheelPeak, met.uPeak};
metricYlab = {'RMS tilt (deg)', 'Max tilt (deg)', 'Median recovery time (s)', ...
              'Initial settling time (s)', 'Wheel speed peak (rad/s)', 'Voltage peak (V)'};
metricTtl = {'RMS tilt deviation per LQR gain set', ...
             'Maximum tilt deviation per LQR gain set', ...
             'Median disturbance recovery time per LQR gain set', ...
             'Initial settling time per LQR gain set', ...
             'Wheel speed peak per LQR gain set', ...
             'Control voltage peak per LQR gain set'};
metricFmt = {'%.2f', '%.2f', '%.2f', '%.2f', '%.0f', '%.2f'};
for ci = 1:numel(metricTag)
    isVolt = strcmp(metricTag{ci}, 'compare_voltage');
    fig = figure('Color', 'w', 'Position', [100 + 15*ci 100 + 15*ci 620 470], ...
                 'ToolBar', 'none', 'Name', 'LQR performance comparison');
    ax = axes(fig); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    styleAx(ax, fontName, 10);
    vals = metricVal{ci};
    hB = gobjects(1, nS);
    hL = gobjects(1, nS);
    for s = 1:nS
        hB(s) = bar(ax, s, 0, 0.55, 'FaceColor', colors(s, :), ...
                    'EdgeColor', [0.25 0.25 0.25], 'LineWidth', 0.6);
        hL(s) = text(ax, s, 0, '', 'HorizontalAlignment', 'center', ...
                     'FontName', fontName, 'FontSize', 9.5);
    end
    xlim(ax, [0.4 nS + 0.6]);
    xticks(ax, 1:nS); xticklabels(ax, shortL);
    ylim(ax, [0, max(vals) * 1.28 + eps]);
    if isVolt
        ylim(ax, [0 6.8]);
        yline(ax, 6, '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 0.9);
        text(ax, 2.5, 6.25, 'dashed = 6 V limit', 'FontName', fontName, ...
             'FontSize', 8.5, 'HorizontalAlignment', 'center', 'Color', [0.40 0.40 0.40]);
    end
    xlabel(ax, 'LQR gain set', 'FontName', fontName, 'FontSize', 11);
    ylabel(ax, metricYlab{ci}, 'FontName', fontName, 'FontSize', 11);
    title(ax, metricTtl{ci}, 'FontName', fontName, 'FontSize', 12, 'FontWeight', 'bold');

    plan = buildBarPlan(nS, T_BAR, T_HOLD, FPS, EASE);
    upd = @(ph) updBars(ph, hB, hL, vals, metricFmt{ci});
    renderFrames(fig, plan, upd, ...
                 fullfile(dataDir, [metricTag{ci} '_animated.gif']), ...
                 fullfile(dataDir, [metricTag{ci} '_animated.png']), FPS);
end

% ------------------------------- summary ---------------------------------
for s = 1:nS
    fprintf(['%s: RMS %.2f deg, max %.2f deg, recovery median %.2f s (n=%d), ', ...
             'settling %.2f s, wheel peak %.0f rad/s, U peak %.2f V\n'], ...
            labels{s}, met.rmsTilt(s), met.maxTilt(s), met.recMed(s), ...
            met.nevents(s), met.settle(s), met.wheelPeak(s), met.uPeak(s));
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
function plan = buildPlan(nCurves, n, Tc, Tp, Th, fps, easeExp)
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
function updSingle(ph, hC, tg, y)
switch ph.kind
    case 'curve'
        set(hC, 'XData', tg(1:ph.k), 'YData', y(1:ph.k));
    case 'finish'
        set(hC, 'XData', tg, 'YData', y);
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
function [ev, recs] = detectEvents(t, wn, tilt, sm)
thr = max(18, 4 * median(wn));
idx = find(wn >= thr);
ev = [];
for i = 1:numel(idx)
    if isempty(ev) || t(idx(i)) - t(ev(end)) > 2.5
        ev(end+1) = idx(i); %#ok<AGROW>
    end
end
recs = nan(numel(ev), 1);
for e = 1:numel(ev)
    i0 = ev(e);
    on = i0;
    while on > 1 && t(i0) - t(on) < 0.5 && wn(on) > 10
        on = on - 1;
    end
    ti = t(on);
    pre = t >= ti - 2.0 & t <= ti - 0.2;
    thrEnv = mean(sm(pre)) + max(0.25, 2 * std(sm(pre)));
    post = t >= ti & t <= ti + 2.5;
    ip = find(post, 1) + find(tilt(post) == max(tilt(post)), 1) - 1;
    for k = ip:numel(t)
        seg = t >= t(k) & t <= t(k) + 0.5;
        if nnz(seg) >= 4 && all(sm(seg) < thrEnv)
            recs(e) = t(k) - ti;
            break;
        end
    end
    if isnan(recs(e)), recs(e) = 2.5; end
end
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
