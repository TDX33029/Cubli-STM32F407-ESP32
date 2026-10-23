%% 三种 LQR 参数的平衡性能对比 (图 6)
% 数据: compare_base.csv / compare_low.csv / compare_high.csv
%   每组 30 s 连续平衡; 运行中含重复外部扰动 (检测角速度峰值)
% 指标: RMS 倾角、最大倾角、恢复时间中位数、初始稳定时间、飞轮峰值、电压峰值
%
% 运行: 在 MATLAB 中直接执行本脚本, 输出 PNG 到本目录。

clear; clc; close all;

dataDir = fileparts(mfilename('fullpath'));
if isempty(dataDir), dataDir = pwd; end
fontName = 'Microsoft YaHei';

schemes = {'base', 'low', 'high'};
labels  = {'基线 80/5', '低增益 60/4', '高增益 110/7'};
shortL  = {'基线', '低增益', '高增益'};
colors  = [0.00 0.45 0.74; 0.85 0.33 0.10; 0.40 0.65 0.20];
nS = numel(schemes);

met = struct('rmsTilt', [], 'maxTilt', [], 'recMed', [], 'recMin', [], ...
             'recMax', [], 'settle', [], 'wheelPeak', [], 'uPeak', [], ...
             'uRms', [], 'ratePeak', [], 'nevents', [], 'oscMed', []);
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
    met.uRms(s) = sqrt(mean(U(:).^2));

    % 初始稳定时间: 之后连续 1 s 在 ±0.5° 内
    met.settle(s) = NaN;
    for k = 1:numel(t)
        seg = t >= t(k) & t <= t(k) + 1.0;
        if nnz(seg) >= 4 && all(sm(seg) < 0.5)
            met.settle(s) = t(k);
            break;
        end
    end
    if isnan(met.settle(s)), met.settle(s) = t(end); end

    % 冲击检测 + 每次冲击的恢复时间
    [ev, recs, oscs, ratePeaks] = detectEvents(t, wn, tilt, sm);
    met.nevents(s) = numel(ev);
    met.recMed(s) = median(recs);
    met.recMin(s) = min(recs);
    met.recMax(s) = max(recs);
    met.oscMed(s) = median(oscs);
    met.ratePeak(s) = max(ratePeaks);

    met.t{s} = t; met.tilt{s} = tilt; met.ev{s} = ev; %#ok<SAGROW>
end

% ---------- 图 ----------
fig = figure('Name', '平衡性能对比', 'Color', 'w', 'Position', [40 30 1150 1020]);
tl = tiledlayout(3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, {'三种 LQR 参数的平衡性能对比 (每组 30 s 连续平衡)', ...
           '恢复时间取各组检测到的扰动中位数；电压虚线为 6 V 限幅'}, ...
      'FontName', fontName, 'FontSize', 13.5, 'FontWeight', 'bold');

% 第一行: 三组姿态偏差时序
for s = 1:nS
    ax = nexttile; hold(ax, 'on'); grid(ax, 'on'); box(ax, 'off');
    plot(ax, met.t{s}, met.tilt{s}, '-', 'Color', colors(s, :), 'LineWidth', 1.0);
    yline(ax, 0.5, ':', 'Color', [0.65 0.65 0.65], 'LineWidth', 0.8);
    ylim(ax, [0 5.5]);
    xlim(ax, [0 30]);
    title(ax, sprintf('%s: RMS %.2f°, 最大 %.2f°', ...
          labels{s}, met.rmsTilt(s), met.maxTilt(s)), ...
          'FontName', fontName, 'FontSize', 9.5, 'FontWeight', 'bold');
    if s == 1
        ylabel(ax, '姿态偏差模长 (°)', 'FontName', fontName, 'FontSize', 9.5);
    end
    xlabel(ax, '时间 (s)', 'FontName', fontName, 'FontSize', 9);
    set(ax, 'FontName', fontName, 'FontSize', 9);
end

% 第二/三行: 指标柱状图
bars = {met.rmsTilt, met.maxTilt, met.recMed; ...
        met.settle, met.wheelPeak, met.uPeak};
ylabs = {'RMS 倾角 (°)', '最大倾角 (°)', '恢复时间中位数 (s)'; ...
         '初始稳定时间 (s)', '飞轮转速峰值 (rad/s)', '电压峰值 (V)'};
fmts  = {'%.2f', '%.2f', '%.2f'; '%.2f', '%.0f', '%.2f'};
for r = 1:2
    for c = 1:3
        ax = nexttile; hold(ax, 'on'); grid(ax, 'on'); box(ax, 'off');
        vals = bars{r, c};
        for s = 1:nS
            bar(ax, s, vals(s), 0.55, 'FaceColor', colors(s, :), ...
                'EdgeColor', [0.25 0.25 0.25], 'LineWidth', 0.6);
        end
        xlim(ax, [0.4 nS + 0.6]);
        xticks(ax, 1:nS); xticklabels(ax, shortL);
        ylim(ax, [0, max(vals) * 1.28 + eps]);
        ylabel(ax, ylabs{r, c}, 'FontName', fontName, 'FontSize', 9.5);
        set(ax, 'FontName', fontName, 'FontSize', 9);
        for s = 1:nS
            text(ax, s, vals(s) + 0.03 * max(ylim(ax)), sprintf(fmts{r, c}, vals(s)), ...
                 'HorizontalAlignment', 'center', 'FontName', fontName, 'FontSize', 8.5);
        end
    end
end

% 电压峰值面板标出 6 V 限幅
axLast = gca;
yline(axLast, 6, '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 0.9);
ylim(axLast, [0 6.8]);
title(axLast, '虚线 = 6 V 限幅', 'FontName', fontName, 'FontSize', 8.5, 'FontWeight', 'normal');

exportgraphics(fig, fullfile(dataDir, 'performance_compare.png'), 'Resolution', 160);

% ---------- 控制台输出 ----------
fprintf('\n===== 三种 LQR 参数性能对比 =====\n');
for s = 1:nS
    fprintf(['%s: RMS %.2f°, 最大 %.2f°, 恢复中位数 %.2f s (%.2f–%.2f, n=%d), ', ...
             '初始稳定 %.2f s, 轮速峰值 %.0f rad/s, U峰值 %.2f V, U RMS %.2f V\n'], ...
            labels{s}, met.rmsTilt(s), met.maxTilt(s), met.recMed(s), ...
            met.recMin(s), met.recMax(s), met.nevents(s), met.settle(s), ...
            met.wheelPeak(s), met.uPeak(s), met.uRms(s));
end
fprintf('图片已保存: %s\n', fullfile(dataDir, 'performance_compare.png'));

% ======================================================================
function [ev, recs, oscs, ratePeaks] = detectEvents(t, wn, tilt, sm)
thr = max(18, 4 * median(wn));
idx = find(wn >= thr);
ev = [];
for i = 1:numel(idx)
    if isempty(ev) || t(idx(i)) - t(ev(end)) > 2.5
        ev(end+1) = idx(i); %#ok<AGROW>
    end
end
recs = nan(numel(ev), 1); oscs = zeros(numel(ev), 1); ratePeaks = zeros(numel(ev), 1);
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
    ratePeaks(e) = max(wn(t >= ti & t <= ti + 1.0));
    for k = ip:numel(t)
        seg = t >= t(k) & t <= t(k) + 0.5;
        if nnz(seg) >= 4 && all(sm(seg) < thrEnv)
            recs(e) = t(k) - ti;
            break;
        end
    end
    if isnan(recs(e)), recs(e) = 2.5; end
    lastPeak = -inf;
    for k = ip + 2:numel(t) - 1
        if t(k) > ti + recs(e), break; end
        if sm(k) > sm(k-1) && sm(k) >= sm(k+1) && sm(k) > thrEnv
            lo = max(1, k - 8); hi = min(numel(t), k + 8);
            if sm(k) - min(sm(lo:hi)) >= 0.08 && t(k) - lastPeak > 0.35
                oscs(e) = oscs(e) + 1;
                lastPeak = t(k);
            end
        end
    end
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
