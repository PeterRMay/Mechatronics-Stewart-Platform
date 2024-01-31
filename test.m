clc
T = [-0.0040;0;0];
Phi =[0;0;90];
ArmLegRatio = 1/10;
PlatformBaseRatio = 0.8;
RestingLegLength = 100;
PlatformRadius = 1;

StewartPlatformEqs(T,Phi, ArmLegRatio, PlatformBaseRatio,RestingLegLength, PlatformRadius, true, true);