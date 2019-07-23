function delta = steering_sfb(lateral_state_vec)

%Full state feedback
Ksteering = [0.0551    0.0193    0.8222    0.1252];
delta = -Ksteering*lateral_state_vec;

% Look-ahead lateral position error feedback
% y = e1 + ds * e2, where ds = look=ahead distance
% y = lateral_state_vec(1) + ds * lateral_state_vec(3);
% Use y as the feedback variable and your own controller
% What to do if a dynamic controller, or additional states are needed?