%% 三电机多级转速阶跃响应 (0% -> 30% -> 60% -> 100% -> 60% -> 30% -> 0%)
% 数据来源: Cubli-F407ZGT6 ST 固件 $TELE 遥测 (COM10, 115200 8N1)
% 采集脚本: scope_client.py ($TELE 标称 40 ms/帧, 实测约 50 ms/帧, 含目标转速 M# 与实测滤波转速 S#)
% 每级台阶保持 3.5~4 s, 满速 = 180 rad/s (固件 velocity_limit)
%
% 运行方式: 在 MATLAB 中直接执行本脚本; 图窗弹出并输出 PNG 到本目录。

clear; clc; close all;

dataDir = fileparts(mfilename('fullpath'));
if isempty(dataDir), dataDir = pwd; end

files  = {'m1_stairs.csv', 'm2_stairs.csv', 'm3_stairs.csv'};
names  = {'M1', 'M2', 'M3'};
colors = [0.00 0.45 0.74;   % 蓝
          0.85 0.33 0.10;   % 橙
          0.40 0.65 0.20];  % 绿
FS     = 180;                 % rad/s, "满速" = 100%
RPM    = 60/(2*pi);           % rad/s -> rpm
preS   = 3.3;                 % 第一级台阶前的显示窗口 (s)
fontName = 'Microsoft YaHei';

% ---------- 读取数据 ----------
D = struct([]);
for k = 1:numel(files)
    [t, tgt, act] = readTelemetry(fullfile(dataDir, files{k}), k);
    if numel(t) < 50
        error('文件 %s 有效遥测行过少 (%d 行)', files{k}, numel(t));
    end
    idxStart = find(tgt > 0.01*FS, 1);        % 第一级台阶(30%)起点
    t0 = t(idxStart);
    D(k).t = t - t0;                          % 时间零点 = 第一次阶跃时刻
    D(k).tgt = tgt;
    D(k).act = act;
end

% ---------- 图 1: 三个电机, 每轴一个子图 ----------
fig1 = figure('Name', '三电机多级转速阶跃响应', 'Color', 'w', ...
              'Position', [60 40 1060 960]);
tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, '三电机多级转速阶跃响应 (0% \rightarrow 30% \rightarrow 60% \rightarrow 100% \rightarrow 60% \rightarrow 30% \rightarrow 0%)', ...
      'FontName', fontName, 'FontSize', 14, 'FontWeight', 'bold');

yl = [0 54 108 180];                 % 各级台阶 (rad/s)
ylab = {'0', '30%', '60%', '100%'};

for k = 1:numel(files)
    x  = D(k).t;
    in = x >= -preS & x <= 22.5;

    ax = nexttile;
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'off');

    % 各级台阶参考线
    for v = yl
        yline(ax, v, '-', 'Color', [0.88 0.88 0.88], 'LineWidth', 0.7);
    end
    % 目标转速阶梯
    stairs(ax, x(in), D(k).tgt(in), '--', 'Color', [0.42 0.42 0.42], 'LineWidth', 1.2);
    % 实测转速
    plot(ax, x(in), D(k).act(in), '-', 'Color', colors(k,:), 'LineWidth', 1.8);

    xlim(ax, [-preS 22.5]);
    ylim(ax, [-10 200]);
    yticks(ax, yl);
    yticklabels(ax, ylab);
    xlabel(ax, '时间 (s)', 'FontName', fontName, 'FontSize', 10);
    ylabel(ax, '转速 (rad/s)', 'FontName', fontName, 'FontSize', 10);
    set(ax, 'FontName', fontName, 'FontSize', 10);

    % 指标: 100% 台阶峰值/超调, 全程稳态最大偏差
    [vPeak, overshoot, maxSteadyErr] = stairMetrics(D(k).tgt, D(k).act, FS);

    title(ax, sprintf(['%s:  100%% 台阶峰值 %.1f rad/s (超调 %.1f%%)   |   ', ...
                       '各级稳态最大偏差 %.1f%%'], ...
                      names{k}, vPeak, overshoot, maxSteadyErr), ...
          'FontName', fontName, 'FontSize', 10.5, 'FontWeight', 'bold');

    if k == 1
        legend(ax, {'目标转速', '实测转速'}, 'Location', 'southeast', ...
               'FontName', fontName, 'FontSize', 9);
    end
end

% ---------- 图 2: 三轴实测转速对比 ----------
fig2 = figure('Name', '三电机多级阶跃对比', 'Color', 'w', ...
              'Position', [90 90 1060 480]);
ax2 = axes(fig2); hold(ax2, 'on'); grid(ax2, 'on'); box(ax2, 'off');

for v = yl
    yline(ax2, v, '-', 'Color', [0.88 0.88 0.88], 'LineWidth', 0.7);
end
x = D(1).t;
in = x >= -preS & x <= 22.5;
stairs(ax2, x(in), D(1).tgt(in), '--', 'Color', [0.42 0.42 0.42], 'LineWidth', 1.2);
for k = 1:numel(files)
    plot(ax2, D(k).t(in), D(k).act(in), '-', 'Color', colors(k,:), 'LineWidth', 1.7);
end

xlim(ax2, [-preS 22.5]); ylim(ax2, [-10 200]);
yticks(ax2, yl); yticklabels(ax2, ylab);
set(ax2, 'FontName', fontName, 'FontSize', 11);
xlabel(ax2, '时间 (s)', 'FontName', fontName, 'FontSize', 11);
ylabel(ax2, '转速 (rad/s)', 'FontName', fontName, 'FontSize', 11);
title(ax2, '三电机实测转速对比 (满速 180 rad/s = 1719 rpm)', ...
      'FontName', fontName, 'FontSize', 13, 'FontWeight', 'bold');
legend(ax2, [names, {'目标转速'}], 'Location', 'northeast', ...
       'FontName', fontName, 'FontSize', 10);

% ---------- 命令行指标表 ----------
fprintf('\n===== 多级阶跃各级稳态 (最后 1 s 均值) =====\n');
fprintf('%-4s %8s %14s %14s %10s\n', '轴', '台阶', '目标rad/s', '实测均值', '误差');
for k = 1:numel(files)
    t = D(k).t; tgt = D(k).tgt; act = D(k).act;
    idx = find(diff([-1; tgt]) ~= 0);        % 目标变化点
    idx(end+1) = numel(tgt) + 1;
    for j = 1:numel(idx)-1
        seg = idx(j):idx(j+1)-1;
        if numel(seg) < 3, continue; end
        tv = tgt(seg(end));
        tail = t >= t(seg(end)) - 1.0 & t <= t(seg(end));
        m = mean(act(tail));
        fprintf('%-4s %7.0f%% %14.0f %14.1f %9.1f%%\n', ...
                names{k}, tv/FS*100, tv, m, (m-tv)/max(1,tv)*100);
    end
end
fprintf('满速参考: %.0f rad/s = %.0f rpm\n\n', FS, FS*RPM);

% ---------- 保存 PNG ----------
exportgraphics(fig1, fullfile(dataDir, 'staircase_response_3motors.png'), 'Resolution', 150);
exportgraphics(fig2, fullfile(dataDir, 'staircase_response_compare.png'), 'Resolution', 150);
fprintf('图片已保存:\n  %s\n  %s\n', ...
        fullfile(dataDir, 'staircase_response_3motors.png'), ...
        fullfile(dataDir, 'staircase_response_compare.png'));

% ======================================================================
function [vPeak, overshoot, maxSteadyErr] = stairMetrics(tgt, act, FS)
%STAIRMETRICS 100% 台阶的峰值/超调, 以及各级稳态(最后 1s)最大相对偏差。
idx = find(diff([-1; tgt]) ~= 0);
idx(end+1) = numel(tgt) + 1;

vPeak = -inf; overshoot = 0; maxSteadyErr = 0;
for j = 1:numel(idx)-1
    seg = idx(j):idx(j+1)-1;
    if numel(seg) < 3, continue; end
    target = tgt(seg(end));
    % 只在"上升到 100%"的区间统计超调
    if abs(target - FS) < 1
        pk = max(act(seg));
        vPeak = pk;
        overshoot = max(0, (pk - FS)/FS*100);
    end
    % 稳态偏差
    tail = seg(max(1, numel(seg)-5):end);   % 最后 5 帧 (约 0.25 s @ 50 ms/帧)
    m = mean(act(tail));
    maxSteadyErr = max(maxSteadyErr, abs(m - target)/max(1,target)*100);
end
end

function [t, tgt, act] = readTelemetry(fname, motorIdx)
%READTELEMETRY 解析 scope_client.py 采集的 $TELE 遥测 CSV, 提取指定电机通道。
%   行格式: t_local_ms,t_rel_ms,"$TELE,M1:..,M2:..,M3:..,...,S1:..,S2:..,S3:..,...,Bal:0#"
lines = readlines(fname, 'Encoding', 'UTF-8');
t = zeros(0,1); tgt = zeros(0,1); act = zeros(0,1);

framePat = '^([0-9.]+),([0-9.]+),"(\$TELE,.*)"$';
tgtPat = sprintf('M%d:([-0-9.]+)', motorIdx);
actPat = sprintf('S%d:([-0-9.]+)', motorIdx);

for i = 1:numel(lines)
    tok = regexp(lines(i), framePat, 'tokens', 'once');
    if isempty(tok), continue; end
    mt = regexp(tok{3}, tgtPat, 'tokens', 'once');
    mv = regexp(tok{3}, actPat, 'tokens', 'once');
    if isempty(mt) || isempty(mv), continue; end
    t(end+1,1)   = str2double(tok{2}) / 1000;   % t_rel_ms -> s
    tgt(end+1,1) = str2double(mt{1});
    act(end+1,1) = str2double(mv{1});
end
end
