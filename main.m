close all;
clear all;
clc;

[leg, bodies] = leg_model;
figure; show(leg);

theta = zeros(length(bodies), 1); % angles of each joint
p = [0; 1; -1];                   % position of **ankle**
r = [0; -1; 0] - p;
A = 3;
B = 3;
C = norm(r);
theta(4) = acos(real((C^2 - A^2 - B^2) / 2 / A / B));
theta(5) = -atan2(r(1), sign(r(3)) * sqrt(r(2) ^ 2 + r(3) ^ 2)) ...
	- asin((A / C) * sin(pi - theta(4)));
theta(6) = atan2(r(2), r(3));
if theta(6) > pi / 2
	theta(6) = theta(6) - pi;
elseif theta(6) < -pi / 2
	theta(6) = theta(6) + pi;
end
R = [1, 0, 0; ...
	0, cos(theta(6)), sin(theta(6)); ...
	0, -sin(theta(6)), cos(theta(6))] ...
	* [cos(theta(4)+theta(5)), 0, -sin(theta(4)+theta(5)); ...
	0, 1, 0; ...
	sin(theta(4)+theta(5)), 0, cos(theta(4)+theta(5))];
theta(1) = atan2(-R(1,2), R(2,2));
theta(2) = atan2(R(3,2), R(2,2) * cos(theta(1)) - R(1,2) * sin(theta(1)));
theta(3) = atan2(-R(3,1), R(3,3));

for k = 1:length(bodies)
	body = getBody(leg, bodies{k});
	joint = copy(body.Joint);
	joint.HomePosition = theta(k);
	replaceJoint(leg, bodies{k}, joint);
end
figure; show(leg);
