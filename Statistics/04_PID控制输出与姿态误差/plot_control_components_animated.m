%% PID/LQR control output vs. attitude error — one figure per chart.
%  The five charts are produced as independent figures, each with its own
%  animated GIF and final static PNG:
%    ctrl_error_animated   - control error E1 / E2 / E3
%    ctrl_pterm_animated   - P term P1 / P2 / P3
%    ctrl_dterm_animated   - D term D1 / D2 / D3
%    ctrl_wterm_animated   - wheel-feedback term W1 / W2 / W3
%    ctrl_output_animated  - total output U1 / U2 / U3 (+ 6 V limit lines)
%  Curves are drawn one after another with an ease-out profile (fast at the
%  beginning, slowing towards the end).  Academic style: Times New Roman,
%  boxed axes, inward ticks, dashed grid, framed legend.
%
%  Run:  matlab -batch "cd('<this folder>'); plot_control_components_animated"

clear; clc; close all;

dataDir = fileparts(mfilename('fullpath'));
if isempty(dataDir), dataDir = pwd; end

% ----------------------------- configuration -----------------------------
FPS      = 10;
DT       = 0.04;
T_CURVE  = 1.2;
T_PAUSE  = 0.15;
T_HOLD   = 1.4;
EASE     = 2.0;

fontName = 'Times New Roman';
cW = [0.85 0.45 0.10; 0.00 0.35 0.75; 0.20 0.55 0.20];

% ------------------------------ read data --------------------------------
M = readTELE(fullfile(dataDir, 'ctrl_run1.csv'));
sel = M.bal == 3;
if ~any(sel), error('No Bal=3 (BALANCING) segment in the data'); end

t = M.t(sel); t = t - t(1);
E = [M.e1(sel), M.e2(sel), M.e3(sel)];
P = [M.p1(sel), M.p2(sel), M.p3(sel)];
Dm = [M.d1(sel), M.d2(sel), M.d3(sel)];
W = [M.w1(sel), M.w2(sel), M.w3(sel)];
U = [M.u1(sel), M.u2(sel), M.u3(sel)];
vq = M.vq(find(sel, 1));
dur = t(end);
aux = U - (P + Dm + W);

% ------------------------------ metrics ----------------------------------
rmsE = sqrt(mean(E.^2, 1));
eAC = zeros(1, 3); uAC = zeros(1, 3);
for i = 1:3
    eAC(i) = std(E(:, i) - movmean(E(:, i), 9));
    uAC(i) = std(U(:, i) - movmean(U(:, i), 9));
end
rmsP = sqrt(mean(P.^2, 1));
rmsD = sqrt(mean(Dm.^2, 1));
rmsW = sqrt(mean(W.^2, 1));
mP = mean(abs(P), 'all'); mD = mean(abs(Dm), 'all'); mW = mean(abs(W), 'all');
shareP = mP / (mP + mD + mW);
shareD = mD / (mP + mD + mW);
shareW = mW / (mP + mD + mW);
uPeak = max(abs(U), [], 'all');
satPct = 100 * mean(abs(U(:)) >= 0.995 * vq);
auxMax = max(abs(aux), [], 'all');
corrPE = 0; corrDE = 0; corrWE = 0;
for i = 1:3
    c = corrcoef(E(:, i), P(:, i));  corrPE = corrPE + c(1, 2) / 3;
    c = corrcoef(E(:, i), Dm(:, i)); corrDE = corrDE + c(1, 2) / 3;
    c = corrcoef(E(:, i), W(:, i));  corrWE = corrWE + c(1, 2) / 3;
end

% ------------------- dense uniform grid, one entry/curve -----------------
[tu, iu] = unique(t, 'stable');
tg = (0:DT:dur)';
rs = @(v) interp1(tu, v(iu), tg, 'linear');

% panel definitions: {gif/png tag, data matrix, ylabel, title cell}
panelDefs = {
    'ctrl_error',   E,  'Control error e_i (deg)', ...
        {'Control error during corner balancing', ...
         sprintf('RMS %.2f / %.2f / %.2f°,  high-freq noise %.2f / %.2f / %.2f°', ...
                 rmsE(1), rmsE(2), rmsE(3), eAC(1), eAC(2), eAC(3))};
    'ctrl_pterm',   P,  'P term (V)', ...
        {'P term K_\theta·e during corner balancing', ...
         sprintf('RMS %.2f / %.2f / %.2f V,  %.0f%% of output,  corr(e,P)=%.2f (BDIR=-1 inversion)', ...
                 rmsP(1), rmsP(2), rmsP(3), 100*shareP, corrPE)};
    'ctrl_dterm',   Dm, 'D term (V)', ...
        {'D term K_d·rate during corner balancing', ...
         sprintf('RMS %.2f / %.2f / %.2f V,  %.0f%% of output,  corr(e,D)=%.2f', ...
                 rmsD(1), rmsD(2), rmsD(3), 100*shareD, corrDE)};
    'ctrl_wterm',   W,  'Wheel-feedback term (V)', ...
        {'Wheel feedback -K_w·\omega during corner balancing', ...
         sprintf('RMS %.2f / %.2f / %.2f V,  %.0f%% of output,  K_i=0 (no attitude integrator)', ...
                 rmsW(1), rmsW(2), rmsW(3), 100*shareW)};
    'ctrl_output',  U,  'Total output U_i (V)', ...
        {'Total LQR output during corner balancing', ...
         sprintf(['peak %.2f V / limit %.0f V,  saturation %.1f%%,  high-freq noise ', ...
                  '%.2f / %.2f / %.2f V,  residual max %.2f V'], ...
                 uPeak, vq, satPct, uAC(1), uAC(2), uAC(3), auxMax)};
};
legNames = {{'E1','E2','E3'}, {'P1','P2','P3'}, {'D1','D2','D3'}, ...
            {'W1','W2','W3'}, {'U1','U2','U3'}};

for ip = 1:size(panelDefs, 1)
    tag = panelDefs{ip, 1};
    X = panelDefs{ip, 2};
    ylab = panelDefs{ip, 3};
    ttl = panelDefs{ip, 4};

    fig = figure('Color', 'w', 'Position', [80 + 20*ip 80 + 20*ip 900 470], ...
                 'ToolBar', 'none', 'Name', 'Control output vs. attitude error');
    ax = axes(fig); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    styleAx(ax, fontName, 10.5);
    h = gobjects(1, 3);
    for i = 1:3
        h(i) = plot(ax, NaN, NaN, '-', 'Color', cW(i, :), 'LineWidth', 1.2);
    end
    yline(ax, 0, '-', 'Color', [0.75 0.75 0.75], 'LineWidth', 0.9);
    if strcmp(tag, 'ctrl_output')
        yline(ax, vq, '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 0.9);
        yline(ax, -vq, '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 0.9);
        ylim(ax, [-1.15, 1.15] * vq);
    else
        ylim(ax, [min(X(:)) - 0.10 * range(X(:)), max(X(:)) + 0.10 * range(X(:))]);
    end
    xlabel(ax, 'Time from balance onset (s)', 'FontName', fontName, 'FontSize', 11.5);
    ylabel(ax, ylab, 'FontName', fontName, 'FontSize', 11.5);
    title(ax, ttl, 'FontName', fontName, 'FontSize', 11.5, 'FontWeight', 'bold');
    legend(ax, h, legNames{ip}, 'Location', 'northeast', ...
           'FontName', fontName, 'FontSize', 8.5, 'Box', 'on');
    xlim(ax, [0 dur]);

    keys = {'C1', 'C2', 'C3'};   % internal field names only
    D = struct(keys{1}, rs(X(:, 1)), keys{2}, rs(X(:, 2)), keys{3}, rs(X(:, 3)));
    H = struct(keys{1}, h(1), keys{2}, h(2), keys{3}, h(3));

    plan = buildPlan(keys, numel(tg), T_CURVE, T_PAUSE, T_HOLD, FPS, EASE);
    upd = @(ph) updLines(ph, H, D, tg);
    renderFrames(fig, plan, upd, ...
                 fullfile(dataDir, [tag '_animated.gif']), ...
                 fullfile(dataDir, [tag '_animated.png']), FPS);
end

% ------------------------------- summary ---------------------------------
fprintf('Balance segment: %.2f s, %d frames\n', dur, sum(sel));
fprintf('Component shares |P|:|D|:|W| = %.0f%% : %.0f%% : %.0f%%\n', ...
        100*shareP, 100*shareD, 100*shareW);
fprintf('Total output peak %.2f V, limit %.1f V, saturation %.1f%%\n', uPeak, vq, satPct);
fprintf('Reconstruction residual (U-P-D-W): max %.3f V\n', auxMax);
fprintf('Same-point correlation: corr(e,P)=%.2f, corr(e,D)=%.2f, corr(e,W)=%.2f\n', ...
        corrPE, corrDE, corrWE);

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
keys = {'Bal','Vq','Pit','Rol','RefPit','RefRol','Wx','Wy','Wz', ...
        'S1','S2','S3','U1','U2','U3', ...
        'E1','E2','E3','P1','P2','P3','D1','D2','D3','W1','W2','W3'};
fields = {'bal','vq','pit','rol','refpit','refrol','wx','wy','wz', ...
          's1','s2','s3','u1','u2','u3', ...
          'e1','e2','e3','p1','p2','p3','d1','d2','d3','w1','w2','w3'};
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
