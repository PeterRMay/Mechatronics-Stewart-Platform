clear all; close all; clc;

%% initialize standard values
stdPlatformBaseRatio = 1;
stdArmLegRatio = 0.1495;
stdRestingLegLength = 11/12;

platformParams = struct;
platformParams.armlegratio = stdArmLegRatio;
platformParams.platformbaseratio = stdPlatformBaseRatio;
platformParams.radius = 1/2;
platformParams.restingleglength = stdRestingLegLength;
platformParams.servorange = 180;
platformParams.defaultHeight = 0.7541;

%%
[Trange, Phirange] = findROM_disturbance(platformParams, true, linspace(-40,40,6500));
% [Trange, Phirange] = findROM_disturbance(platformParams, true, linspace(-1.5,1.5,15));
