clear all; clc; close all;

tau = 0.2;
lowpass = tf(1, [tau 1]);
step(lowpass)
TS = 20*10^-3;
lowPassD = c2d(lowpass,TS,'tustin');
lowPassD
step(lowPassD)

alpha = TS / (2*tau + TS);
-(1-2*alpha)
2*alpha-1