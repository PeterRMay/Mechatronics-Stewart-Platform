function [alpha] = servoCalc(P_base_i,B_i,l,s,a,beta,maxAlphaRange,minAlphaRange)
    
    L = l^2 - (s^2 - a^2);
    M = 2*a*(P_base_i(3)-B_i(3));
    N = 2*a*(cosd(beta)*(P_base_i(1)-B_i(1)) + sind(beta)*(P_base_i(2)-B_i(2)));

    foo = (L / (sqrt(M^2 + N^2)));

    if foo > 1
        foo = 1;
        % disp("limits exceeded")
    elseif foo < -1
        foo = -1;
        % disp("limits exceeded")
    end
    
    alpha = asind(foo) - atand(N/M);
    if isnan(alpha)
        alpha = 0;
    end
    alpha = real(alpha);

    if alpha > maxAlphaRange
        alpha = maxAlphaRange;
    elseif alpha < minAlphaRange
        alpha = minAlphaRange;
    end
end


