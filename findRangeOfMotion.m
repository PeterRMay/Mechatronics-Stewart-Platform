clear all; close all; clc;

TRange = zeros(3,2);
PhiRange = zeros(3,2);

testRange = linspace(-40,40,10000);

for q = 1:6
    if q <4
        currentError = true;
        previousError = true;
        T = zeros(3,1);
        Phi = zeros(3,1);
        for i = 1:length(testRange)
            T(q) = testRange(i);
            currentError = StewartPlatformEqs(T, Phi);
            if previousError == true && currentError == false % transition from out of range to in range
                
                minValue = testRange(i);
            end
            if previousError == false && currentError == true % transition into range
              
                maxValue = testRange(i);
            end
            previousError = currentError;
        end
        TRange(q,:) = [minValue maxValue];


    else
        currentError = true;
        previousError = true;
        T = zeros(3,1);
        Phi = zeros(3,1);
        for i = 1:length(testRange)
            Phi(q-3) = testRange(i);
            currentError = StewartPlatformEqs(T, Phi);
            if previousError == true && currentError == false % transition from out of range to in range
                minValue = testRange(i);
            end
            if previousError == false && currentError == true % transition into range
                maxValue = testRange(i);
            end
            previousError = currentError;
        end
       PhiRange(q-3,:) = [minValue maxValue];
    end
    
end
TRange
PhiRange