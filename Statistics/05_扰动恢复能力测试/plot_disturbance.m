%% 扰动恢复能力 (三次人工轻敲)
% 数据: kick_run1.csv — BAUTO 单点平衡, 30.2 s, 三次轻敲约 2.6/7.9/12.8 s
% 冲击时刻由陀螺角速度模长自动检测 (阈值 20 deg/s, 3 s 不应期)
% 指标: 最大偏移(原始峰值)、恢复时间(回到冲击前包络并保持 0.5 s)、二次振荡次数
%
% 运行: 在 MATLAB 中直接执行本脚本, 输出 PNG 到本目录。

clear; clc; close all;

dataDir = fileparts(mfilename('fullpath'));
if isempty(dataDir), dataDir = pwd; end
fontName = 'Microsoft YaHei';

M = readTELE(fullfile(dataDir, 'kick_run1.csv'));
sel = M.bal == 3;
if ~any(sel), error('数据中没有 Bal=3 (BALANCING) 段'); end

t = M.t(sel); t = t - t(1);
eP = M.pit(sel) - M.refpit(sel);
eR = M.rol(sel) - M.refrol(sel);
tilt = hypot(eP, eR);
W = [M.wx(sel), M.wy(sel), M.wz(sel)];
wn = sqrt(sum(W.^2, 2));
U = [M.u1(sel), M.u2(sel), M.u3(sel)];
S = [M.s1(sel), M.s2(sel), M.s3(sel)];

% 平滑 (0.25 s) 用于恢复判据, 原始值用于最大偏移
sm = movmean(tilt, 5);

% ---------- 冲击检测 ----------
% 取首次越过阈值的时刻作为冲击起点; 2.5 s 不应期避免把响应中的二次
% 角速度峰当成新的冲击 (三次敲击间隔约 5 s)。
rateThr = max(18, 4 * median(wn));
idx = find(wn >= rateThr);
events = [];
for i = 1:numel(idx)
    if isempty(events) || t(idx(i)) - t(events(end)) > 2.5
        events(end+1) = idx(i); %#ok<AGROW>
    end
end
if numel(events) < 3
    error('只检测到 %d 次冲击, 需要 3 次', numel(events));
end
if numel(events) > 3
    % 超过三次时保留角速度峰值最大的三次 (在各自 ±1 s 窗口内)
    pk = zeros(numel(events), 1);
    for i = 1:numel(events)
        w = t >= t(events(i)) - 1.0 & t <= t(events(i)) + 1.0;
        pk(i) = max(wn(w));
    end
    [~, ord] = sort(pk, 'descend');
    events = events(sort(ord(1:3)));
end
tp = t(events);                      % 三次冲击的起点时刻
nEv = numel(tp);

% ---------- 指标 ----------
peakDev = zeros(nEv, 1); peakTime = zeros(nEv, 1);
recTime = zeros(nEv, 1); recThr = zeros(nEv, 1);
oscCount = zeros(nEv, 1); baseMean = zeros(nEv, 1);
ratePeak = zeros(nEv, 1); uPeak = zeros(nEv, 1);
for k = 1:nEv
    pre = t >= tp(k) - 2.0 & t <= tp(k) - 0.2;
    baseMean(k) = mean(sm(pre));
    recThr(k) = baseMean(k) + max(0.25, 2 * std(sm(pre)));
    post = t >= tp(k) & t <= tp(k) + 2.5;
    [peakDev(k), ip] = max(tilt(post));
    ip = find(post, 1) + ip - 1;
    peakTime(k) = t(ip) - tp(k);
    ratePeak(k) = max(wn(t >= tp(k) & t <= tp(k) + 1.0));
    uWin = t >= tp(k) - 0.3 & t <= tp(k) + 2.0;
    uPeak(k) = max(abs(U(uWin, :)), [], 'all');

    recTime(k) = NaN;
    for j = ip:numel(t)
        seg = t >= t(j) & t <= t(j) + 0.5;
        if nnz(seg) >= 4 && all(sm(seg) < recThr(k))
            recTime(k) = t(j) - tp(k);
            break;
        end
    end
    if isnan(recTime(k)), recTime(k) = 5.0; end

    oscCount(k) = 0; lastPeak = -inf;
    for j = ip + 2:numel(t) - 1
        if t(j) > tp(k) + recTime(k), break; end
        if sm(j) > sm(j-1) && sm(j) >= sm(j+1) && sm(j) > recThr(k)
            lo = max(1, j - 8); hi = min(numel(t), j + 8);
            if sm(j) - min(sm(lo:hi)) >= 0.08 && t(j) - lastPeak > 0.35
                oscCount(k) = oscCount(k) + 1;
                lastPeak = t(j);
            end
        end
    end
end
dur = t(end);

% ---------- 图 ----------
evCol = [0.00 0.45 0.74; 0.85 0.33 0.10; 0.40 0.65 0.20];
lw = 1.4;

fig = figure('Name', '扰动恢复能力', 'Color', 'w', 'Position', [50 30 1100 1080]);
tl = tiledlayout(3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, '扰动恢复能力: 三次人工轻敲 (BAUTO 单点平衡)', ...
      'FontName', fontName, 'FontSize', 14, 'FontWeight', 'bold');

% (a) 全程姿态偏差 + 冲击标记
ax1 = nexttile([1 3]); hold(ax1, 'on'); grid(ax1, 'on'); box(ax1, 'off');
hP = plot(ax1, t, eP, '-', 'Color', [0.10 0.10 0.10], 'LineWidth', 1.2);
hR = plot(ax1, t, eR, '-', 'Color', [0.00 0.35 0.75], 'LineWidth', 1.2);
yline(ax1, 0, '-', 'Color', [0.75 0.75 0.75], 'LineWidth', 0.9);
for k = 1:nEv
    xline(ax1, tp(k), '--', 'Color', evCol(k, :), 'LineWidth', 1.1);
end
yl = ylim(ax1);
ylim(ax1, [yl(1), yl(2) + 1.0]);
for k = 1:nEv
    text(ax1, tp(k), yl(2) + 0.5, sprintf('K%d', k), 'Color', evCol(k, :), ...
         'HorizontalAlignment', 'center', 'FontName', fontName, ...
         'FontSize', 9, 'FontWeight', 'bold');
end
xlim(ax1, [0 dur]);
ylabel(ax1, '姿态偏差 (°)', 'FontName', fontName, 'FontSize', 10);
title(ax1, sprintf(['全程姿态偏差: 冲击 %s s, 角速度峰值 %s°/s, ', ...
                    '平衡保持 %.1f s'], mat2str(round(tp', 1)), ...
                    mat2str(round(ratePeak', 0)), dur), ...
      'FontName', fontName, 'FontSize', 10.5, 'FontWeight', 'bold');
legend(ax1, [hP hR], {'Pitch 偏差', 'Roll 偏差'}, 'Location', 'northeast', ...
       'FontName', fontName, 'FontSize', 8.5);

% (b) 三次恢复曲线对齐
ax2 = nexttile([1 3]); hold(ax2, 'on'); grid(ax2, 'on'); box(ax2, 'off');
fill(ax2, [-1.5 5 5 -1.5], [0.5 0.5 0.8 0.8], [0.85 0.85 0.85], ...
     'FaceAlpha', 0.25, 'EdgeColor', 'none');
h = gobjects(1, nEv);
for k = 1:nEv
    win = t >= tp(k) - 1.5 & t <= tp(k) + 5.0;
    h(k) = plot(ax2, t(win) - tp(k), tilt(win), '-', ...
                'Color', evCol(k, :), 'LineWidth', lw);
    plot(ax2, recTime(k), recThr(k), 'o', 'Color', evCol(k, :), ...
         'MarkerFaceColor', 'w', 'MarkerSize', 7, 'LineWidth', 1.4);
end
xline(ax2, 0, '--', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.0);
xlim(ax2, [-1.5 5.0]);
xlabel(ax2, '冲击后时间 \tau (s)', 'FontName', fontName, 'FontSize', 10);
ylabel(ax2, '偏差模长 (°)', 'FontName', fontName, 'FontSize', 10);
title(ax2, sprintf(['三次恢复对齐: 最大偏移 %.2f / %.2f / %.2f°,  ', ...
                    '恢复 %.2f / %.2f / %.2f s, 振荡 %d / %d / %d 次'], ...
                    peakDev(1), peakDev(2), peakDev(3), ...
                    recTime(1), recTime(2), recTime(3), ...
                    oscCount(1), oscCount(2), oscCount(3)), ...
      'FontName', fontName, 'FontSize', 10.5, 'FontWeight', 'bold');
legend(ax2, h, arrayfun(@(k) sprintf('K%d', k), 1:nEv, 'UniformOutput', false), ...
       'Location', 'northeast', 'FontName', fontName, 'FontSize', 9);
text(ax2, 4.9, 0.65, '冲击前包络', 'FontName', fontName, 'FontSize', 8, ...
     'HorizontalAlignment', 'right', 'Color', [0.40 0.40 0.40]);

% (c)-(e) 指标柱状图
metricVal = {peakDev, recTime, oscCount};
metricTitle = {'最大偏移 (°)', '恢复时间 (s)', '振荡次数'};
metricFmt = {'%.2f', '%.2f', '%d'};
for k = 1:3
    ax = nexttile; hold(ax, 'on'); grid(ax, 'on'); box(ax, 'off');
    for j = 1:nEv
        bar(ax, j, metricVal{k}(j), 0.55, 'FaceColor', evCol(j, :), ...
            'EdgeColor', [0.25 0.25 0.25], 'LineWidth', 0.6);
    end
    xlim(ax, [0.4 nEv + 0.6]);
    xticks(ax, 1:nEv);
    xticklabels(ax, arrayfun(@(j) sprintf('K%d', j), 1:nEv, 'UniformOutput', false));
    ylabel(ax, metricTitle{k}, 'FontName', fontName, 'FontSize', 10);
    set(ax, 'FontName', fontName, 'FontSize', 9.5);
    if k == 3
        ylim(ax, [0 max(oscCount) + 1]);
        yticks(ax, 0:ceil(max(oscCount)));
    else
        ylim(ax, [0 max(metricVal{k}) * 1.25]);
    end
    for j = 1:nEv
        text(ax, j, metricVal{k}(j) + 0.03 * max(ylim(ax)), ...
             sprintf(metricFmt{k}, metricVal{k}(j)), ...
             'HorizontalAlignment', 'center', 'FontName', fontName, 'FontSize', 9);
    end
end

set([ax1 ax2], 'FontName', fontName, 'FontSize', 10, 'GridAlpha', 0.12);
exportgraphics(fig, fullfile(dataDir, 'disturbance_recovery.png'), 'Resolution', 160);

% ---------- 控制台输出 ----------
fprintf('\n===== 扰动恢复指标 =====\n');
fprintf('平衡段 %.2f s, 检测到 %d 次冲击\n', dur, nEv);
for k = 1:nEv
    fprintf(['K%d: t=%.2f s, 角速度峰值 %.1f°/s, 最大偏移 %.2f° (%.2f s), ', ...
             '恢复 %.2f s (阈值 %.2f°), 二次振荡 %d 次, U峰值 %.2f V\n'], ...
            k, tp(k), ratePeak(k), peakDev(k), peakTime(k), ...
            recTime(k), recThr(k), oscCount(k), uPeak(k));
end
fprintf('图片已保存: %s\n', fullfile(dataDir, 'disturbance_recovery.png'));

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
