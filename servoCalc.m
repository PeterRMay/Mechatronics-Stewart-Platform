function [alpha] = servoCalc(P,B,l,s,a,beta)
    L = l^2 - (s^2 - a^2);
    M = 2*a*(P(3)-B(3));
    N = 2*a*(cosd(beta)*(P(1)-B(1)) + sind(beta)*(P(2)-B(2)));
    alpha = asind(L / (sqrt(M^2 + N^2))) - atand(N/M);
    if isnan(alpha)
        alpha = 0;
    end
    alpha = real(alpha);
end