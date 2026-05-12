function vec = sample_current_3d(current_field, x, y, z)
%SAMPLE_CURRENT_3D Return nearest-neighbor current vector [u; v; w].

[MAX_X, MAX_Y, MAX_Z] = size(current_field.U);
xi = max(1, min(MAX_X, round(x)));
yi = max(1, min(MAX_Y, round(y)));
zi = max(1, min(MAX_Z, round(z)));

vec = [
    current_field.U(xi, yi, zi)
    current_field.V(xi, yi, zi)
    current_field.W(xi, yi, zi)
];
end
