clear all;

% inputs:
% x, y, z relative displacements into T matrix
x = 0;
y = 0;
z = 0;
h = 2;
T = [x;y;z+h];
% phi, theta, psi
phi = 0;
theta = 0;
psi = 0;

% constants:
% b[] distances to each leg, array of 3x1 location vectors
% distance to each leg is 1
b1 = [1;0;0];
beta = [0,10,120,130,240,250];
% b = [b1, rotz(10)*b1, rotz(120)*b1, rotz(130)*b1, rotz(240)*b1,
% rotz(250)*b1]; OBSOLETE
b = zeros(3,6);
for i = 1:6
b(:,i) = rotz(beta(:,i))*b1;
end

% p[] distances from origin of platform to the joints
p = b;

% compute rotation matrix based on input angles
R = rotz(phi)*roty(theta)*rotx(psi);

% compute l_i of each leg
l = (T + R*p) - b;

% take 2norm of each leg and find unit vectors 
% unit vectors aren't actually useful :(
d = size(l);
l_2norm = zeros(1,d(1,2));
l_unit = zeros(d);
for i = 1:d(1,2)
    l_2norm(i) = norm(l(:,i));
    l_unit(:,i) = l(:,i) ./ l_2norm(i);
end


% calculate P_base, end of each joint in base reference frame
P_base = b + l;
P_base(:,end+1) = P_base(:,end-1);

% plot legs
% figure;
% hold on
plot3(P_base(1,:),P_base(2,:),P_base(3,:),'-')
hold on
plot3(b(1,:),b(2,:),b(3,:),'-')
axis equal
% for i = 1:d(1,2)
%     plot3(P_base(1,i),P_base(2,i),P_base(3,i),'-')
% end


% calculate joint angles

%% servo motors
s = 0.9; % length of rod
a = 0.1; % length of servo arm

% define home position


% calculate servo angles
alpha = zeros(1,6);
for i = 1:6
    alpha(i) = servoCalc(p(:,i),b(:,i),l_2norm(i),s,a,beta(i));
    servoArms = b(:,i) + a * rotz(beta(i)) * roty(alpha(i));

end

function [alpha] = servoCalc(P,B,l,s,a,beta)
L = l^2 - (s^2 - a^2);
M = 2*a*(P(3)-B(3));
N = 2*a*(cos(beta)*(P(1)-B(1)) + sin(beta)*(P(2)-B(2)));
alpha = asin(L / (sqrt(M^2 + N^2))) - atan(N/M);

end
