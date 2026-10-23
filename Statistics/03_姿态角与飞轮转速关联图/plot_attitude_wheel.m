%% 姿态角与飞轮转速关联图 (Cubli 单点平衡)
% 数据: corner_run1.csv — BAUTO 单点(corner)平衡, Bal=3 段约 25.4 s
% 固件: BMODE 4, BAXIS 2 3 1, BDIR -1 -1 -1, LQR 80/5/-0.03/0,
%       U=BU=6 V, L=330, BCAP=0.5°, BFULL=250 rad/s
%
% 三子图共享时间轴:
%   (a) 姿态角偏差  Pit-RefPit / Rol-RefRol
%   (b) 三飞轮实际转速 S1/S2/S3
%   (c) LQR 控制电压 U1/U2/U3 (单点平衡没有转速目标, 电压即等效指令)
% 另计算: 电压->轮速互相关峰值与滞后, 姿态偏差->轮速变化率相关性。
%
% 运行: 在 MATLAB 中直接执行本脚本, 输出 PNG 到本目录。

clear; clc; close all;

dataDir = fileparts(mfilename('fullpath'));
if isempty(dataDir), dataDir = pwd; end
fontName = 'Microsoft YaHei';

M = readTELE(fullfile(dataDir, 'corner_run1.csv'));
sel = M.bal == 3;
if ~any(sel), error('数据中没有 Bal=3 (BALANCING) 段'); end

t = M.t(sel);
t = t - t(1);
errP = (M.pit - M.refpit); errP = errP(sel);
errR = (M.rol - M.refrol); errR = errR(sel);
S = [M.s1(sel), M.s2(sel), M.s3(sel)];
U = [M.u1(sel), M.u2(sel), M.u3(sel)];
dur = t(end);

% ---------- 指标 ----------
rmsP = rms(errP);      rmsR = rms(errR);
p2pP = max(errP) - min(errP);
p2pR = max(errR) - min(errR);
sStart = S(1, :);      sEnd = S(end, :);
sPeak = max(abs(S), [], 1);
slope = zeros(1, 3);
for i = 1:3
    p = polyfit(t, S(:, i), 1);
    slope(i) = p(1);
end
uMean = mean(U, 1);    uMin = min(U, [], 'all');    uMax = max(U, [], 'all');

% 轮速变化率 (对姿态偏差的即时响应)
dS = zeros(size(S));
for i = 1:3, dS(:, i) = gradient(S(:, i), t); end

% 电压 -> 轮速 互相关 (均匀重采样 50 ms, 搜索 ±2 s)
dt = 0.05;
tu = (0:dt:dur)';
peakUS = zeros(1, 3); lagUS = zeros(1, 3);
peakUdS = zeros(1, 3); lagUdS = zeros(1, 3);
for i = 1:3
    uu = interp1(t, U(:, i), tu, 'linear');
    ss = interp1(t, S(:, i), tu, 'linear');
    dd = interp1(t, dS(:, i), tu, 'linear');
    [peakUS(i), lagUS(i)] = corrLag(uu, ss, dt, 2.0);
    [peakUdS(i), lagUdS(i)] = corrLag(uu, dd, dt, 2.0);
end

% 姿态偏差 -> 轮速变化率 同点相关
corrPdS = zeros(1, 3); corrRdS = zeros(1, 3);
for i = 1:3
    c = corrcoef(errP, dS(:, i)); corrPdS(i) = c(1, 2);
    c = corrcoef(errR, dS(:, i)); corrRdS(i) = c(1, 2);
end

% ---------- 图 ----------
cP = [0.10 0.10 0.10];
cR = [0.00 0.35 0.75];
cW = [0.85 0.33 0.10; 0.00 0.45 0.74; 0.40 0.65 0.20];
lw = 1.4;

fig = figure('Name', '姿态角与飞轮转速关联', 'Color', 'w', ...
             'Position', [50 40 1100 940]);
tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, '姿态角与飞轮转速关联 (BAUTO 单点平衡)', ...
      'FontName', fontName, 'FontSize', 14, 'FontWeight', 'bold');

% (a) 姿态角偏差
ax1 = nexttile; hold(ax1, 'on'); grid(ax1, 'on'); box(ax1, 'off');
hP = plot(ax1, t, errP, '-', 'Color', cP, 'LineWidth', lw);
hR = plot(ax1, t, errR, '-', 'Color', cR, 'LineWidth', lw);
yline(ax1, 0, '-', 'Color', [0.75 0.75 0.75], 'LineWidth', 0.9);
ylabel(ax1, '偏差 \theta-\theta_{ref} (°)', 'FontName', fontName, 'FontSize', 10);
title(ax1, sprintf(['姿态角偏差: RMS  Pitch %.2f° / Roll %.2f°,  ', ...
                    '峰峰 %.2f° / %.2f°'], rmsP, rmsR, p2pP, p2pR), ...
      'FontName', fontName, 'FontSize', 10.5, 'FontWeight', 'bold');
legend(ax1, [hP hR], {'Pitch 偏差', 'Roll 偏差'}, 'Location', 'northeast', ...
       'FontName', fontName, 'FontSize', 8.5);
ylim(ax1, [-1.15 1.15]);

% (b) 三飞轮实际转速
ax2 = nexttile; hold(ax2, 'on'); grid(ax2, 'on'); box(ax2, 'off');
hS = gobjects(1, 3);
for i = 1:3
    hS(i) = plot(ax2, t, S(:, i), '-', 'Color', cW(i, :), 'LineWidth', lw);
end
yline(ax2, 0, '-', 'Color', [0.75 0.75 0.75], 'LineWidth', 0.9);
ylabel(ax2, '飞轮转速 (rad/s)', 'FontName', fontName, 'FontSize', 10);
title(ax2, sprintf(['飞轮转速: 25.4 s 内漂移至 %.0f / %.0f / %.0f rad/s,  ', ...
                    '平均斜率 %.2f / %.2f / %.2f rad/s^2 (持续反作用力矩)'], ...
                    sEnd(1), sEnd(2), sEnd(3), slope(1), slope(2), slope(3)), ...
      'FontName', fontName, 'FontSize', 10.5, 'FontWeight', 'bold');
legend(ax2, hS, {'S1', 'S2', 'S3'}, 'Location', 'northeast', ...
       'FontName', fontName, 'FontSize', 8.5);
xlim(ax2, [0 dur]);

% (c) LQR 控制电压 (单点平衡无转速目标, 电压为等效指令)
ax3 = nexttile; hold(ax3, 'on'); grid(ax3, 'on'); box(ax3, 'off');
hU = gobjects(1, 3);
for i = 1:3
    hU(i) = plot(ax3, t, U(:, i), '-', 'Color', cW(i, :), 'LineWidth', lw);
end
yline(ax3, 0, '-', 'Color', [0.75 0.75 0.75], 'LineWidth', 0.9);
xlabel(ax3, '时间 (s, 以自动接管时刻为 0)', 'FontName', fontName, 'FontSize', 10);
ylabel(ax3, '控制电压 (V)', 'FontName', fontName, 'FontSize', 10);
title(ax3, sprintf(['控制电压: 均值 %.2f / %.2f / %.2f V;  ', ...
                    '电压->轮速互相关 %.2f / %.2f / %.2f, 滞后 %.2f / %.2f / %.2f s'], ...
                    uMean(1), uMean(2), uMean(3), ...
                    peakUS(1), peakUS(2), peakUS(3), ...
                    lagUS(1), lagUS(2), lagUS(3)), ...
      'FontName', fontName, 'FontSize', 10.5, 'FontWeight', 'bold');
legend(ax3, hU, {'U1', 'U2', 'U3'}, 'Location', 'northeast', ...
       'FontName', fontName, 'FontSize', 8.5);
xlim(ax3, [0 dur]);

linkaxes([ax1 ax2 ax3], 'x');
xlim(ax1, [0 dur]);
set([ax1 ax2 ax3], 'FontName', fontName, 'FontSize', 10, 'GridAlpha', 0.12);

exportgraphics(fig, fullfile(dataDir, 'attitude_wheel.png'), 'Resolution', 160);

% ---------- 控制台输出 ----------
fprintf('\n===== 姿态角与飞轮转速关联指标 =====\n');
fprintf('平衡段: %.2f s, %d 帧, 平均采样 %.0f ms\n', dur, sum(sel), 1000 * mean(diff(t)));
fprintf('姿态偏差 RMS: Pitch %.2f°, Roll %.2f°; 峰峰 %.2f° / %.2f°\n', rmsP, rmsR, p2pP, p2pR);
fprintf('飞轮转速: 起始 %.1f / %.1f / %.1f -> 结束 %.1f / %.1f / %.1f rad/s\n', ...
        sStart(1), sStart(2), sStart(3), sEnd(1), sEnd(2), sEnd(3));
fprintf('飞轮峰值 |S|: %.0f / %.0f / %.0f rad/s (BFULL=250, 余量 %.0f%%)\n', ...
        sPeak(1), sPeak(2), sPeak(3), 100 * (1 - max(sPeak) / 250));
fprintf('轮速趋势斜率: %.2f / %.2f / %.2f rad/s^2\n', slope(1), slope(2), slope(3));
fprintf('控制电压: 均值 %.2f / %.2f / %.2f V, 范围 [%.2f, %.2f] V\n', ...
        uMean(1), uMean(2), uMean(3), uMin, uMax);
for i = 1:3
    fprintf(['M%d: U->S 互相关峰值 %.2f, 滞后 %.2f s; ' ...
             'U->dS 峰值 %.2f, 滞后 %.2f s; corr(errP,dS)=%.2f, corr(errR,dS)=%.2f\n'], ...
            i, peakUS(i), lagUS(i), peakUdS(i), lagUdS(i), corrPdS(i), corrRdS(i));
end
fprintf('图片已保存: %s\n', fullfile(dataDir, 'attitude_wheel.png'));

% ======================================================================
function [peak, lag] = corrLag(u, s, dt, maxLag)
% 互相关峰值与滞后; 正 lag 表示 s 滞后于 u。
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
