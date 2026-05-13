function current_field = make_current_field_3d(MAX_X, MAX_Y, MAX_Z, base_speed, vertical_speed)
%MAKE_CURRENT_FIELD_3D Build a layered shear-jet current field for 3D tests.
% The field combines depth-varying shear flow, a central jet, and weak
% vertical disturbance. It is designed to influence route choice without
% overwhelming the GBSOM activity field.

if nargin < 5
    vertical_speed = 0.08 * base_speed;
end

U = zeros(MAX_X, MAX_Y, MAX_Z);
V = zeros(MAX_X, MAX_Y, MAX_Z);
W = zeros(MAX_X, MAX_Y, MAX_Z);

cy = (MAX_Y + 1) / 2;
jet_width = max(2.5, MAX_Y / 7);

for x = 1:MAX_X
    for y = 1:MAX_Y
        jet_strength = exp(-((y - cy)^2) / (2 * jet_width^2));
        cross_shear = sin(2 * pi * y / MAX_Y);

        for z = 1:MAX_Z
            depth_phase = (z - 1) / max(1, MAX_Z - 1);
            depth_shear = cos(pi * depth_phase);
            upper_layer_bias = 0.5 + 0.5 * depth_shear;

            U(x, y, z) = base_speed * (0.45 + 0.55 * jet_strength) * depth_shear;
            V(x, y, z) = base_speed * 0.28 * cross_shear * (0.5 + upper_layer_bias);
            W(x, y, z) = vertical_speed * sin(pi * x / MAX_X) * ...
                sin(2 * pi * y / MAX_Y) * cos(pi * depth_phase);
        end
    end
end

current_field.U = U;
current_field.V = V;
current_field.W = W;
current_field.base_speed = base_speed;
current_field.vertical_speed = vertical_speed;
end
