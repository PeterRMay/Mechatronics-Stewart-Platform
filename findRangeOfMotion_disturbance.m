clear all; close all; clc;

%% initialize standard values
stdPlatformBaseRatio = 0.5;
stdArmLegRatio = 0.1495;
stdRestingLegLength = 11/12;

platformParams = struct;
platformParams.armlegratio = stdArmLegRatio;
platformParams.platformbaseratio = stdPlatformBaseRatio;
platformParams.radius = 1;
platformParams.restingleglength = stdRestingLegLength;
platformParams.servorange = 180;
platformParams.defaultHeight = 0.3743;

%%
[Trange, Phirange] = findROM_disturbance(platformParams, true, linspace(-200,200,10000));
