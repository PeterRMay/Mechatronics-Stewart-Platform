function [BP, correctionAngles] = baseToStaticPlatformPosition(D,Gamma,T,Phi,h0)
% calculate angle offset from inertial frame
gammaX = Gamma(1) + Phi(1);
gammaY = Gamma(2) + Phi(2);
gammaZ = Gamma(3) + Phi(3);

OP = T+[0;0;h0]; % vector from origin of inertial frame (O) to origin of platform (P)

R = rotz(gammaZ)*roty(gammaY)*rotx(gammaX); % 3x3 rotation matrix

BP = R'*(OP - D); % BP is the base origin to platform origin vector
correctionAngles = -Gamma;