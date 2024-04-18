function h0 = findh0(platformParams)

armLegRatio = platformParams.armlegratio;
platformBaseRatio = platformParams.platformbaseratio;
platformRadius = platformParams.radius;
legRestingLength = platformParams.restingleglength;
angleBetweenLegPairs = platformParams.angleBetweenLegPairs;


% leg parameters
s = sqrt(legRestingLength^2 / (1+armLegRatio)); % length of rod
a = s * armLegRatio; % length of servo arm
b1 = [platformRadius;0;0]; % ADD a name for length
angleFromAxis = angleBetweenLegPairs / 2;
legAngles = [-angleFromAxis,angleFromAxis,120-angleFromAxis,120+angleFromAxis,240-angleFromAxis,240+angleFromAxis] + 120; %% IMPORTANT: added 120 to match up axes with fwd kinematics
legAngles = circshift(legAngles,[0 1]);
b = zeros(3,6);
for i = 1:6
    b(:,i) = rotz(legAngles(i))*b1;
end

% p[] distances from origin of platform to the joints
p = b*platformBaseRatio; % adjust platform size
defaultPlatformRotation = 60;
for i = 1:6
    p(:,i) = rotz(defaultPlatformRotation) * p(:,i); % rotate top platform
end
p = circshift(p,[0 1]); % shift matrix to shift which legs connect to which joints

l0_squared = s^2+a^2; %default leg length using pythagoras theorem bc right angles
h0 = sqrt(l0_squared - (p(1,1) - b(1,1))^2 - (p(2,1) - b(2,1))^2) - p(3,1);
if isreal(h0) ~= true
        error("Legs too short given platform dimensions")
end