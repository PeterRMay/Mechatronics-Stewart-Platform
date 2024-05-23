clear all; close all; clc;

data = load("StewartData.mat");

gamma = [data.gammaX, data.gammaY, data.gammaZ];
T = [data.TX, data.TY, data.TZ];
Phi = [data.PhiX, data.PhiY, data.PhiZ];
legLengths = [data.legLengths1, data.legLengths2, data.legLengths3, data.legLengths4, data.legLengths5, data.legLengths6];
alpha = [data.alpha1, data.alpha2, data.alpha3, data.alpha4, data.alpha5, data.alpha6];
outOfRange = data.outOfRange;

L = length(outOfRange);
radius = 23;

figure
hold on
for i = 1:L
    if outOfRange(i) == 1
        pointRadius = sqrt(gamma(i,1)^2 + gamma(i,2)^2);
        if pointRadius < radius
            plot(gamma(i,1), gamma(i,2), 'marker', 'o', 'MarkerSize',5)
        end
    end
end
xlabel('Pitch')
ylabel('Roll')
title('Range of Motion')

figure
for i = 1:3
    subplot(3,1,i)
    plot(gamma(:,i))
end