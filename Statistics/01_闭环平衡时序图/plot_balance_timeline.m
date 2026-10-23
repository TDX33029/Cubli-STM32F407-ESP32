%% 单点闭环平衡过程时序图 (Cubli 反作用飞轮)
% 数据: corner_run1.csv — BAUTO 单点(corner)平衡, 接管后约 25.4 s
% 固件: Cubli-ST-Statistics, BMODE 4, BAXIS 2 3 1, BDIR -1 -1 -1,
%       LQR 80/5/-0.03/0, U=BU=6 V, L=330, BCAP=0.5°, BFULL=250
%
% 三子图共享时间轴: 姿态角(含目标) / 三轴角速度 / 三飞轮转速
% 运行: 在 MATLAB 中直接执行本脚本, 输出 PNG 到本目录。

clear; clc; close all;

dataDir = fileparts(mfilename('fullpath'));
if isempty(dataDir), dataDir = pwd; end
fontName = 'Microsoft YaHei';

M = readTELE(fullfile(dataDir, 'corner_run1.csv'));
iBal = find(M.bal == 3, 1);
if isempty(iBal), error('数据中没有 Bal=3 (BALANCING) 段'); end

t0 = M.t(iBal);
x  = M.t - t0;
win = x >= -2.0 & x <= 26.0;

% ---------- 指标 ----------
errP = M.pit - M.refpit;
errR = M.rol - M.refrol;

% 稳定时间: 两轴误差连续 1 s 进入 ±1° 带
settleT = NaN;
for i = iBal:numel(M.t)
    seg = i:min(i+4, numel(M.t));
    if numel(seg) >= 3 && all(abs(errP(seg)) < 1) && all(abs(errR(seg)) < 1)
        settleT = M.t(i) - t0;
        break;
    end
end

% 稳态(接管 1 s 后)振荡
steady = find(M.t >= t0 + 1.0 & M.bal == 3);
rmsP = rms(errP(steady));  rmsR = rms(errR(steady));
p2pP = max(errP(steady)) - min(errP(steady));
p2pR = max(errR(steady)) - min(errR(steady));
dur  = M.t(steady(end)) - M.t(steady(1));
wheelPeak = max(abs([M.s1(steady); M.s2(steady); M.s3(steady)]));

% ---------- 图 ----------
fig = figure('Name', '单点闭环平衡过程时序', 'Color', 'w', ...
             'Position', [50 40 1100 940]);
tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, '单点闭环平衡过程时序 (BAUTO 接管)', ...
      'FontName', fontName, 'FontSize', 14, 'FontWeight', 'bold');

% (1) 三轴姿态角 + 目标
ax1 = nexttile; hold(ax1, 'on'); grid(ax1, 'on'); box(ax1, 'off');
yyaxis(ax1, 'left');
hP = plot(ax1, x(win), M.pit(win), 'k-', 'LineWidth', 1.5);
hR = plot(ax1, x(win), M.rol(win), '-', 'Color', [0.00 0.35 0.75], 'LineWidth', 1.5);
hRefP = yline(ax1, M.refpit(iBal), 'k--', 'LineWidth', 1.0);
hRefR = yline(ax1, M.refrol(iBal), '--', 'Color', [0.00 0.35 0.75], 'LineWidth', 1.0);
xline(ax1, 0, '--', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.0);
ylabel(ax1, '倾角 (°)', 'FontName', fontName, 'FontSize', 10);
ylim(ax1, [20 55]);
yyaxis(ax1, 'right');
hY = plot(ax1, x(win), M.yaw(win), '-', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.0);
ylabel(ax1, 'Yaw (°)', 'FontName', fontName, 'FontSize', 10);
set(ax1, 'FontName', fontName, 'FontSize', 10);
ax1.YAxis(1).FontName = fontName;
ax1.YAxis(2).FontName = fontName;
title(ax1, sprintf(['姿态角: 稳定时间 %.1f s (±1°), 稳态 RMS  Pitch %.2f° / Roll %.2f°,  ', ...
                    '峰峰 %.2f° / %.2f°'], settleT, rmsP, rmsR, p2pP, p2pR), ...
      'FontName', fontName, 'FontSize', 10.5, 'FontWeight', 'bold');
legend(ax1, [hP hR hY hRefP hRefR], ...
       {'Pitch', 'Roll', 'Yaw (右轴)', 'Pitch 目标', 'Roll 目标'}, ...
       'Location', 'northeast', 'FontName', fontName, 'FontSize', 8.5);
xlim(ax1, [-2 26]);

% (2) 三轴角速度
ax2 = nexttile; hold(ax2, 'on'); grid(ax2, 'on'); box(ax2, 'off');
plot(ax2, x(win), M.wx(win), '-', 'Color', [0.85 0.20 0.15], 'LineWidth', 1.3);
plot(ax2, x(win), M.wy(win), '-', 'Color', [0.00 0.45 0.20], 'LineWidth', 1.3);
plot(ax2, x(win), M.wz(win), '-', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.3);
yline(ax2, 0, '-', 'Color', [0.85 0.85 0.85], 'LineWidth', 0.8);
xline(ax2, 0, '--', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.0);
xlim(ax2, [-2 26]);
ylabel(ax2, '角速度 (°/s)', 'FontName', fontName, 'FontSize', 10);
set(ax2, 'FontName', fontName, 'FontSize', 10);
title(ax2, '三轴角速度 (校准后, 已扣除零偏)', ...
      'FontName', fontName, 'FontSize', 10.5, 'FontWeight', 'bold');
legend(ax2, {'\omega_x', '\omega_y', '\omega_z'}, 'Location', 'northeast', ...
       'FontName', fontName, 'FontSize', 8.5);

% (3) 三飞轮转速 + 控制电压
ax3 = nexttile; hold(ax3, 'on'); grid(ax3, 'on'); box(ax3, 'off');
h1 = plot(ax3, x(win), M.s1(win), '-', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.3);
h2 = plot(ax3, x(win), M.s2(win), '-', 'Color', [0.00 0.45 0.74], 'LineWidth', 1.3);
h3 = plot(ax3, x(win), M.s3(win), '-', 'Color', [0.40 0.65 0.20], 'LineWidth', 1.3);
yline(ax3, 0, '-', 'Color', [0.85 0.85 0.85], 'LineWidth', 0.8);
xline(ax3, 0, '--', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.0);
xlim(ax3, [-2 26]);
xlabel(ax3, '时间 (s, 以自动接管时刻为 0)', 'FontName', fontName, 'FontSize', 10);
ylabel(ax3, '飞轮转速 (rad/s)', 'FontName', fontName, 'FontSize', 10);
set(ax3, 'FontName', fontName, 'FontSize', 10);
title(ax3, sprintf('三飞轮转速: 稳态峰值 %.0f rad/s, 平衡持续 %.1f s', wheelPeak, dur), ...
      'FontName', fontName, 'FontSize', 10.5, 'FontWeight', 'bold');
legend(ax3, [h1 h2 h3], {'S1', 'S2', 'S3'}, 'Location', 'northeast', ...
       'FontName', fontName, 'FontSize', 8.5);

exportgraphics(fig, fullfile(dataDir, 'balance_timeline.png'), 'Resolution', 160);

fprintf('\n===== 单点平衡时序指标 =====\n');
fprintf('接管时刻: t=%.1f s (相对采集起点)\n', t0);
fprintf('平衡持续: %.1f s\n', dur);
fprintf('稳定时间(±1°): %.1f s\n', settleT);
fprintf('稳态 RMS: Pitch %.2f°, Roll %.2f°\n', rmsP, rmsR);
fprintf('稳态峰峰: Pitch %.2f°, Roll %.2f°\n', p2pP, p2pR);
fprintf('飞轮峰值: %.0f rad/s\n', wheelPeak);
fprintf('图片已保存: %s\n', fullfile(dataDir, 'balance_timeline.png'));

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
        m = regexp(body, [keys{j} ':([-0-9.]+)'], 'tokens', 'once');
        if isempty(m), ok = false; break; end
        vals(j) = str2double(m{1});
    end
    if ~ok, continue; end
    M.t(end+1,1) = str2double(tok{2}) / 1000;
    M.bal(end+1,1) = str2double(bal{1});
    for j = 1:numel(fields), M.(fields{j})(end+1,1) = vals(j); end
end
end
