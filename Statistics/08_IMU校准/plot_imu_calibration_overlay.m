%% IMU 校准前后叠加对比 (两段独立的复位后 0-30 s 数据)
% 实验流程:
%   段 A: ST-Link 复位 -> 未校准直接采集 0-30 s  ($TELE, Cal=0)
%   段 B: ST-Link 复位 -> CAL_IMU 校准 -> 采集 0-30 s ($TELE, Cal=1)
% 两段时间轴均以各自采集起点为 0, Yaw 以各自首帧为参考做归零, 叠加显示。
%
% 运行方式: 在 MATLAB 中直接执行本脚本; 图窗弹出并输出 PNG 到本目录。

clear; clc; close all;

dataDir = fileparts(mfilename('fullpath'));
if isempty(dataDir), dataDir = pwd; end
fontName = 'Microsoft YaHei';

% ---------- 读取两段数据 ----------
U = readTelemetry(fullfile(dataDir, 'uncal_30s.csv'));   % 未校准
C = readTelemetry(fullfile(dataDir, 'cal_30s.csv'));     % 校准后

% 未校准段: Cal=0 且 t>=0;  校准段: Cal=1 且 t>=0
selU = U.t >= 0 & U.cal == 0;
selC = C.t >= 0 & C.cal == 1;
tU = U.t(selU); yU = U.yaw(selU);
tC = C.t(selC); yC = C.yaw(selC);

if numel(tU) < 50 || numel(tC) < 50
    error('有效数据不足: 未校准 %d 帧, 校准 %d 帧', numel(tU), numel(tC));
end

% 各自以首帧为时间/角度参考
xU = tU - tU(1); yU = yU - yU(1);
xC = tC - tC(1); yC = yC - yC(1);

% 漂移率: 线性拟合斜率
pU = polyfit(xU, yU, 1);
pC = polyfit(xC, yC, 1);
driftU = pU(1);  driftC = pC(1);
ratio  = abs(driftU) / max(abs(driftC), 1e-9);

% ---------- 叠加图 ----------
fig = figure('Name', 'IMU 校准前后 Yaw 漂移叠加', 'Color', 'w', ...
             'Position', [100 100 1080 520]);
ax = axes(fig); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'off');

hU = plot(ax, xU, yU, '-', 'Color', [0.85 0.20 0.15], 'LineWidth', 2.0);
hC = plot(ax, xC, yC, '-', 'Color', [0.10 0.60 0.25], 'LineWidth', 2.0);
yline(ax, 0, '-', 'Color', [0.75 0.75 0.75], 'LineWidth', 0.8);

xlim(ax, [0 30]);
ylim(ax, [-1 11]);
set(ax, 'FontName', fontName, 'FontSize', 11);
xlabel(ax, '时间 (s, 复位后从 0 开始)', 'FontName', fontName, 'FontSize', 11);
ylabel(ax, 'Yaw 漂移 (°, 相对首帧归零)', 'FontName', fontName, 'FontSize', 11);
title(ax, {'IMU 校准前后 0-30 s 叠加对比', ...
           sprintf(['未校准 %.2f °/s (%.1f °/min)  \\rightarrow  校准后 %.3f °/s (%.1f °/min),  ', ...
                    '漂移抑制 %.0f 倍'], driftU, driftU*60, driftC, driftC*60, ratio)}, ...
      'FontName', fontName, 'FontSize', 13, 'FontWeight', 'bold');
legend(ax, [hU hC], {'未校准 (Cal=0)', '校准后 (Cal=1)'}, ...
       'Location', 'northwest', 'FontName', fontName, 'FontSize', 10);

% 30 s 端点漂移标注
text(ax, 30, yU(end), sprintf('  %.1f°', yU(end)), 'FontName', fontName, ...
     'FontSize', 10, 'Color', [0.85 0.20 0.15], 'VerticalAlignment', 'bottom');
text(ax, 30, yC(end), sprintf('  %.1f°', yC(end)), 'FontName', fontName, ...
     'FontSize', 10, 'Color', [0.10 0.60 0.25], 'VerticalAlignment', 'bottom');

exportgraphics(fig, fullfile(dataDir, 'imu_calibration_overlay.png'), 'Resolution', 160);

% ---------- 命令行摘要 ----------
fprintf('\n===== IMU 校准前后 0-30 s 对比 =====\n');
fprintf('未校准段: %d 帧, %.1f s, Yaw 漂移 %.1f° (%.2f °/s, %.1f °/min)\n', ...
        numel(tU), xU(end), yU(end), driftU, driftU*60);
fprintf('校准后段: %d 帧, %.1f s, Yaw 漂移 %.1f° (%.3f °/s, %.1f °/min)\n', ...
        numel(tC), xC(end), yC(end), driftC, driftC*60);
fprintf('漂移抑制: %.0f 倍\n', ratio);
fprintf('校准估计零偏 Bx/By/Bz: %+.2f / %+.2f / %+.2f °/s\n', ...
        C.bx(end), C.by(end), C.bz(end));
fprintf('图片已保存: %s\n', fullfile(dataDir, 'imu_calibration_overlay.png'));

% ======================================================================
function M = readTelemetry(fname)
%READTELEMETRY 解析 imu_capture.py 采集的 $TELE CSV。
%   行格式: t_local_ms,t_rel_ms,"$TELE,...,Yaw:..,Bx:..,By:..,Bz:..,Cal:..,...#"
lines = readlines(fname, 'Encoding', 'UTF-8');
M.t = zeros(0,1); M.yaw = zeros(0,1); M.pit = zeros(0,1); M.rol = zeros(0,1);
M.bx = zeros(0,1); M.by = zeros(0,1); M.bz = zeros(0,1); M.cal = zeros(0,1);

framePat = '^([0-9.]+),([-0-9.]+),"(\$TELE,.*)"$';

for i = 1:numel(lines)
    tok = regexp(lines(i), framePat, 'tokens', 'once');
    if isempty(tok), continue; end
    body = tok{3};
    yaw = regexp(body, 'Yaw:([-0-9.]+)', 'tokens', 'once');
    pit = regexp(body, 'Pit:([-0-9.]+)', 'tokens', 'once');
    rol = regexp(body, 'Rol:([-0-9.]+)', 'tokens', 'once');
    bx  = regexp(body, 'Bx:([-0-9.]+)', 'tokens', 'once');
    by  = regexp(body, 'By:([-0-9.]+)', 'tokens', 'once');
    bz  = regexp(body, 'Bz:([-0-9.]+)', 'tokens', 'once');
    cal = regexp(body, 'Cal:([0-9]+)', 'tokens', 'once');
    if isempty(yaw) || isempty(cal), continue; end

    M.t(end+1,1)   = str2double(tok{2}) / 1000;
    M.yaw(end+1,1) = str2double(yaw{1});
    M.cal(end+1,1) = str2double(cal{1});
    if ~isempty(pit), M.pit(end+1,1) = str2double(pit{1}); end
    if ~isempty(rol), M.rol(end+1,1) = str2double(rol{1}); end
    if ~isempty(bx),  M.bx(end+1,1)  = str2double(bx{1});  end
    if ~isempty(by),  M.by(end+1,1)  = str2double(by{1});  end
    if ~isempty(bz),  M.bz(end+1,1)  = str2double(bz{1});  end
end
end
