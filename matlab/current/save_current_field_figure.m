function save_current_field_figure(current_field, MAX_X, MAX_Y, MAX_Z, output_dir)
%SAVE_CURRENT_FIELD_FIGURE Export a sparse 3D current vector plot.

if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

step = 4;
[X, Y, Z] = meshgrid(1:step:MAX_Y, 1:step:MAX_X, 1:step:MAX_Z);
U = zeros(size(X));
V = zeros(size(X));
W = zeros(size(X));

for idx = 1:numel(X)
    vec = sample_current_3d(current_field, Y(idx), X(idx), Z(idx));
    U(idx) = vec(2);
    V(idx) = vec(1);
    W(idx) = vec(3);
end

fig = figure('Visible', 'off', 'Position', [100, 100, 900, 700]);
quiver3(X, Y, Z, U, V, W, 1.2, 'Color', [0.1 0.45 0.9], 'LineWidth', 1.0);
grid on;
axis([0 MAX_Y 0 MAX_X 0 MAX_Z]);
view(42, 24);
xlabel('Y坐标 / 栅格');
ylabel('X坐标 / 栅格');
zlabel('Z坐标 / 栅格');
title('三维分层剪切急流洋流场');
legend({'洋流速度方向'}, 'Location', 'northeastoutside');
print(fig, fullfile(output_dir, 'fig-current-field-3d.png'), '-dpng', '-r300');
close(fig);
end

