%% 相平面图 (Phase Portrait) — 单点平衡, Pitch / Roll 两轴
% 数据: corner_run1.csv (BAUTO 单点平衡, 平衡段约 25.4 s)
% 横轴: 倾角误差 θ = 实际角 - 目标角 (°)
% 纵轴: 角速度 ω = 校准后角速度 (°/s)
% 曲线样式参考 img/02: 彩色时间轨迹 + 方向箭头, 无网格, 细边框
%
% 运行: 在 MATLAB 中直接执行本脚本, 输出 PNG 到本目录。

clear; clc; close all;

dataDir = fileparts(mfilename('fullpath'));
if isempty(dataDir), dataDir = pwd; end
fontName = 'Microsoft YaHei';

M = readTELE(fullfile(dataDir, 'corner_run1.csv'));
iBal = find(M.bal == 3, 1);
if isempty(iBal), error('数据中没有 Bal=3 (BALANCING) 段'); end
nb  = find(M.bal == 3, 1, 'last');
sel = iBal:nb;
t   = M.t(sel) - M.t(iBal);

thP = M.pit(sel) - M.refpit(sel);   % Pitch 误差 (°)
omP = M.wx(sel);                    % Pitch 角速度 (°/s)
thR = M.rol(sel) - M.refrol(sel);   % Roll 误差 (°)
omR = M.wy(sel);                    % Roll 角速度 (°/s)

% 轻度平滑 (3 点, 0.6 s), 仅用于轨迹线与箭头, 散点保留原始数据
sm = @(v) movmean(v, 5);    % 短时平滑, 用于显示轨迹形态
thPs = sm(thP); omPs = sm(omP);
thRs = sm(thR); omRs = sm(omR);

fig = figure('Name', '相平面图', 'Color', 'w', 'Position', [80 60 980 900]);
tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, '单点平衡相平面图 (θ = 实际角 − 目标角, ω = 校准后角速度)', ...
      'FontName', fontName, 'FontSize', 14, 'FontWeight', 'bold');

plotPhase(nexttile, t, thP, omP, thPs, omPs, ...
          'Pitch', 'Pit', 'Wx', fontName, [0.05 0.05 0.05]);
plotPhase(nexttile, t, thR, omR, thRs, omRs, ...
          'Roll', 'Rol', 'Wy', fontName, [0.05 0.05 0.05]);

exportgraphics(fig, fullfile(dataDir, 'phase_portrait.png'), 'Resolution', 160);

fprintf('\n===== 相平面统计 =====\n');
fprintf('平衡段: %.1f s, %d 帧\n', t(end), numel(t));
fprintf('Pitch: |θ|max %.2f°, |ω|max %.1f°/s, RMS θ %.2f°, RMS ω %.2f°/s\n', ...
        max(abs(thP)), max(abs(omP)), rms(thP), rms(omP));
fprintf('Roll : |θ|max %.2f°, |ω|max %.1f°/s, RMS θ %.2f°, RMS ω %.2f°/s\n', ...
        max(abs(thR)), max(abs(omR)), rms(thR), rms(omR));
fprintf('图片已保存: %s\n', fullfile(dataDir, 'phase_portrait.png'));

% ======================================================================
function plotPhase(ax, t, th, om, ths, oms, name, thKey, omKey, fontName, lineColor)
hold(ax, 'on'); box(ax, 'on'); grid(ax, 'off');

% 时间着色散点 (原始数据)
scatter(ax, th, om, 10, t, 'filled', 'MarkerFaceAlpha', 0.55);
% 1 sigma 稳定包络椭圆
mu = mean([th om], 1);
C  = cov([th om]);
[V, D] = eig(C);
ang = linspace(0, 2*pi, 120);
ell = V * sqrt(D) * [cos(ang); sin(ang)] + mu';
hEnv = plot(ax, ell(1,:), ell(2,:), '--', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.2);

% 前 8 s 的平滑轨迹 (避免 25 s 全段缠绕)
early = t <= 8;
te = t(early); thE = ths(early); omE = oms(early);
plot(ax, thE, omE, '-', 'Color', lineColor, 'LineWidth', 1.2);
step = 8;
for k = 1:step:numel(thE)-step
    quiver(ax, thE(k), omE(k), thE(k+1)-thE(k), omE(k+1)-omE(k), 0, ...
           'Color', [0.15 0.15 0.15], 'LineWidth', 0.8, 'MaxHeadSize', 1.2);
end
% 原点十字与零线
xline(ax, 0, ':', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.0);
yline(ax, 0, ':', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.0);
% 起点/终点
hS = plot(ax, ths(1), oms(1), 'o', 'MarkerSize', 8, ...
          'MarkerFaceColor', [0.10 0.65 0.25], 'MarkerEdgeColor', 'k');
hE = plot(ax, ths(end), oms(end), 's', 'MarkerSize', 8, ...
          'MarkerFaceColor', [0.85 0.15 0.10], 'MarkerEdgeColor', 'k');

xlim(ax, [-1.2 1.2]);
ylim(ax, [-12 12]);
set(ax, 'FontName', fontName, 'FontSize', 11);
xlabel(ax, ['\theta = ' thKey ' - 目标角 (°)'], 'FontName', fontName, 'FontSize', 11);
ylabel(ax, ['\omega = ' omKey ' (°/s)'], 'FontName', fontName, 'FontSize', 11);
title(ax, sprintf(['%s 相平面:  |\\theta|_{max} = %.2f°,  |\\omega|_{max} = %.1f °/s,  ', ...
                   'RMS \\theta = %.2f°,  RMS \\omega = %.2f °/s'], ...
                  name, max(abs(th)), max(abs(om)), rms(th), rms(om)), ...
      'FontName', fontName, 'FontSize', 10.5, 'FontWeight', 'bold');
legend(ax, [hS hE hEnv], {'起点 (接管)', '终点 (STOP)', '1σ 包络'}, 'Location', 'northeast', ...
       'FontName', fontName, 'FontSize', 9);
cb = colorbar(ax);
cb.Label.String = '时间 (s)';
cb.Label.FontName = fontName;
colormap(ax, turbo(256));
end

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
        m = regexp(body, [keys{j} ':([-0-9.]+)'], 'tokens', 'once');
        if isempty(m), ok = false; break; end
        vals(j) = str2double(m{1});
    end
    if ~ok, continue; end
    M.t(end+1,1) = str2double(tok{2}) / 1000;
    for j = 1:numel(fields), M.(fields{j})(end+1,1) = vals(j); end
end
end
