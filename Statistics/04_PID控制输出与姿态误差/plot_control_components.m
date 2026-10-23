%% PID/LQR 控制输出与姿态误差 (Cubli 单点平衡)
% 数据: ctrl_run1.csv — 含固件新增的 E/P/D/W 控制分量字段
%   E_i  控制误差 (deg, 绕各自 body 轴的参考误差)
%   P_i  K_theta * e          (V)
%   D_i  K_dtheta * rate      (V)
%   W_i  轮速反馈 -K_w*omega  (V, K_i=0 时无积分项)
%   U_i  LQR 总输出电压 (V, 已限幅)
%
% 五子图共享时间轴: 误差 / P / D / W / 总输出+限幅
% 运行: 在 MATLAB 中直接执行本脚本, 输出 PNG 到本目录。

clear; clc; close all;

dataDir = fileparts(mfilename('fullpath'));
if isempty(dataDir), dataDir = pwd; end
fontName = 'Microsoft YaHei';

M = readTELE(fullfile(dataDir, 'ctrl_run1.csv'));
sel = M.bal == 3;
if ~any(sel), error('数据中没有 Bal=3 (BALANCING) 段'); end

t = M.t(sel); t = t - t(1);
E = [M.e1(sel), M.e2(sel), M.e3(sel)];
P = [M.p1(sel), M.p2(sel), M.p3(sel)];
D = [M.d1(sel), M.d2(sel), M.d3(sel)];
W = [M.w1(sel), M.w2(sel), M.w3(sel)];
U = [M.u1(sel), M.u2(sel), M.u3(sel)];
vq = M.vq(find(sel, 1));
dur = t(end);

% 未饱和时的总输出 = P+D+W+残差(偏航阻尼/配平); 用 U 反推残差
aux = U - (P + D + W);

% ---------- 指标 ----------
rmsE = sqrt(mean(E.^2, 1));
peakE = max(abs(E), [], 1);
meanE = mean(E, 1);
% 高频噪声: 去掉 0.45 s 滑动平均后的标准差
eAC = zeros(1, 3); uAC = zeros(1, 3);
for i = 1:3
    eAC(i) = std(E(:, i) - movmean(E(:, i), 9));
    uAC(i) = std(U(:, i) - movmean(U(:, i), 9));
end
rmsP = sqrt(mean(P.^2, 1));
rmsD = sqrt(mean(D.^2, 1));
rmsW = sqrt(mean(W.^2, 1));
shareP = mean(abs(P), 'all') / (mean(abs(P), 'all') + mean(abs(D), 'all') + mean(abs(W), 'all'));
shareD = mean(abs(D), 'all') / (mean(abs(P), 'all') + mean(abs(D), 'all') + mean(abs(W), 'all'));
shareW = mean(abs(W), 'all') / (mean(abs(P), 'all') + mean(abs(D), 'all') + mean(abs(W), 'all'));
uPeak = max(abs(U), [], 'all');
satPct = 100 * mean(abs(U(:)) >= 0.995 * vq);
auxRms = sqrt(mean(aux.^2, 'all'));
auxMax = max(abs(aux), [], 'all');
% 误差与各分量同点相关 (只作一致性检查)
corrPE = 0; corrDE = 0; corrWE = 0;
for i = 1:3
    c = corrcoef(E(:, i), P(:, i)); corrPE = corrPE + c(1, 2) / 3;
    c = corrcoef(E(:, i), D(:, i)); corrDE = corrDE + c(1, 2) / 3;
    c = corrcoef(E(:, i), W(:, i)); corrWE = corrWE + c(1, 2) / 3;
end

% ---------- 图 ----------
cW = [0.85 0.33 0.10; 0.00 0.45 0.74; 0.40 0.65 0.20];
lw = 1.2;

fig = figure('Name', '控制输出与姿态误差', 'Color', 'w', ...
             'Position', [50 30 1100 1180]);
tl = tiledlayout(5, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, '控制输出与姿态误差 (BAUTO 单点平衡)', ...
      'FontName', fontName, 'FontSize', 14, 'FontWeight', 'bold');

% (a) 控制误差
ax1 = nexttile; hold(ax1, 'on'); grid(ax1, 'on'); box(ax1, 'off');
h = gobjects(1, 3);
for i = 1:3, h(i) = plot(ax1, t, E(:, i), '-', 'Color', cW(i, :), 'LineWidth', lw); end
yline(ax1, 0, '-', 'Color', [0.75 0.75 0.75], 'LineWidth', 0.9);
ylabel(ax1, '控制误差 e_i (°)', 'FontName', fontName, 'FontSize', 10);
title(ax1, sprintf(['控制误差: RMS %.2f / %.2f / %.2f°,  峰峰 %.2f / %.2f / %.2f°,  ', ...
                    '高频噪声 %.2f / %.2f / %.2f°'], ...
                    rmsE(1), rmsE(2), rmsE(3), ...
                    max(E(:,1))-min(E(:,1)), max(E(:,2))-min(E(:,2)), max(E(:,3))-min(E(:,3)), ...
                    eAC(1), eAC(2), eAC(3)), ...
      'FontName', fontName, 'FontSize', 10, 'FontWeight', 'bold');
legend(ax1, h, {'e1', 'e2', 'e3'}, 'Location', 'northeast', ...
       'FontName', fontName, 'FontSize', 8.5);
xlim(ax1, [0 dur]);

% (b) P 项
ax2 = nexttile; hold(ax2, 'on'); grid(ax2, 'on'); box(ax2, 'off');
h = gobjects(1, 3);
for i = 1:3, h(i) = plot(ax2, t, P(:, i), '-', 'Color', cW(i, :), 'LineWidth', lw); end
yline(ax2, 0, '-', 'Color', [0.75 0.75 0.75], 'LineWidth', 0.9);
ylabel(ax2, 'P 项 (V)', 'FontName', fontName, 'FontSize', 10);
title(ax2, sprintf(['P 项 K_\\theta·e: RMS %.2f / %.2f / %.2f V,  占总输出 %.0f%%,  ', ...
                    'corr(e,P)=%.2f (BDIR=-1 反向)'], ...
                    rmsP(1), rmsP(2), rmsP(3), 100*shareP, corrPE), ...
      'FontName', fontName, 'FontSize', 10, 'FontWeight', 'bold');
legend(ax2, h, {'P1', 'P2', 'P3'}, 'Location', 'northeast', ...
       'FontName', fontName, 'FontSize', 8.5);
xlim(ax2, [0 dur]);

% (c) D 项
ax3 = nexttile; hold(ax3, 'on'); grid(ax3, 'on'); box(ax3, 'off');
h = gobjects(1, 3);
for i = 1:3, h(i) = plot(ax3, t, D(:, i), '-', 'Color', cW(i, :), 'LineWidth', lw); end
yline(ax3, 0, '-', 'Color', [0.75 0.75 0.75], 'LineWidth', 0.9);
ylabel(ax3, 'D 项 (V)', 'FontName', fontName, 'FontSize', 10);
title(ax3, sprintf(['D 项 K_d·rate: RMS %.2f / %.2f / %.2f V,  占总输出 %.0f%%,  corr(e,D)=%.2f'], ...
                    rmsD(1), rmsD(2), rmsD(3), 100*shareD, corrDE), ...
      'FontName', fontName, 'FontSize', 10, 'FontWeight', 'bold');
legend(ax3, h, {'D1', 'D2', 'D3'}, 'Location', 'northeast', ...
       'FontName', fontName, 'FontSize', 8.5);
xlim(ax3, [0 dur]);

% (d) 轮速反馈项
ax4 = nexttile; hold(ax4, 'on'); grid(ax4, 'on'); box(ax4, 'off');
h = gobjects(1, 3);
for i = 1:3, h(i) = plot(ax4, t, W(:, i), '-', 'Color', cW(i, :), 'LineWidth', lw); end
yline(ax4, 0, '-', 'Color', [0.75 0.75 0.75], 'LineWidth', 0.9);
ylabel(ax4, '轮速反馈项 (V)', 'FontName', fontName, 'FontSize', 10);
title(ax4, sprintf(['轮速反馈 -K_w·\\omega: RMS %.2f / %.2f / %.2f V,  占总输出 %.0f%%,  ', ...
                    'K_i=0 (无姿态积分)'], rmsW(1), rmsW(2), rmsW(3), 100*shareW), ...
      'FontName', fontName, 'FontSize', 10, 'FontWeight', 'bold');
legend(ax4, h, {'W1', 'W2', 'W3'}, 'Location', 'northeast', ...
       'FontName', fontName, 'FontSize', 8.5);
xlim(ax4, [0 dur]);

% (e) 总输出 + 限幅
ax5 = nexttile; hold(ax5, 'on'); grid(ax5, 'on'); box(ax5, 'off');
h = gobjects(1, 3);
for i = 1:3, h(i) = plot(ax5, t, U(:, i), '-', 'Color', cW(i, :), 'LineWidth', lw); end
yline(ax5, vq, '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 0.9);
yline(ax5, -vq, '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 0.9);
yline(ax5, 0, '-', 'Color', [0.75 0.75 0.75], 'LineWidth', 0.9);
xlabel(ax5, '时间 (s, 以自动接管时刻为 0)', 'FontName', fontName, 'FontSize', 10);
ylabel(ax5, '总输出 U_i (V)', 'FontName', fontName, 'FontSize', 10);
title(ax5, sprintf(['总输出: 峰值 %.2f V / 限幅 %.0f V,  饱和 %.1f%%,  高频噪声 %.2f / %.2f / %.2f V,  ', ...
                    '残差最大 %.2f V'], uPeak, vq, satPct, uAC(1), uAC(2), uAC(3), auxMax), ...
      'FontName', fontName, 'FontSize', 10, 'FontWeight', 'bold');
legend(ax5, h, {'U1', 'U2', 'U3'}, 'Location', 'northeast', ...
       'FontName', fontName, 'FontSize', 8.5);
xlim(ax5, [0 dur]);

linkaxes([ax1 ax2 ax3 ax4 ax5], 'x');
set([ax1 ax2 ax3 ax4 ax5], 'FontName', fontName, 'FontSize', 9.5, 'GridAlpha', 0.12);

exportgraphics(fig, fullfile(dataDir, 'control_components.png'), 'Resolution', 160);

% ---------- 控制台输出 ----------
fprintf('\n===== 控制输出与姿态误差指标 =====\n');
fprintf('平衡段: %.2f s, %d 帧\n', dur, sum(sel));
for i = 1:3
    fprintf(['M%d: e RMS %.2f° (均值 %+.2f°, 峰值 %.2f°, 高频噪声 %.2f°); ', ...
             'P RMS %.2f V; D RMS %.2f V; W RMS %.2f V; U RMS %.2f V\n'], ...
            i, rmsE(i), meanE(i), peakE(i), eAC(i), rmsP(i), rmsD(i), rmsW(i), rms(U(:, i)));
end
fprintf('分量占比 |P|:|D|:|W| = %.0f%% : %.0f%% : %.0f%%\n', 100*shareP, 100*shareD, 100*shareW);
fprintf('总输出峰值 %.2f V, 限幅 %.1f V, 饱和占比 %.1f%%\n', uPeak, vq, satPct);
fprintf('重构残差 (U-P-D-W): RMS %.3f V, 最大 %.3f V\n', auxRms, auxMax);
fprintf('同点相关: corr(e,P)=%.2f, corr(e,D)=%.2f, corr(e,W)=%.2f\n', corrPE, corrDE, corrWE);
fprintf('图片已保存: %s\n', fullfile(dataDir, 'control_components.png'));

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
