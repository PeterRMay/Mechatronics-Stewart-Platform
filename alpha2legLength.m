function [L, A_base] = alpha2legLength(alpha, beta, B_base, a, s, q_previous_base, A_previous_base)
    % find end of servo arm in base frame, A
    % take servo arm [a;0;0] then rotate by beta and then by alpha
    A_base = zeros(3,6);
    A_B = zeros(3,6);
    for i = 1:6
        A_B(:,i) = a * rotz(beta(i)) * roty(-alpha(i)) * [1; 0; 0];
        A_base(:,i) = A_B(:,i) + B_base(:,i);
    end

    % add previous s vector
    previousS_B = q_previous_base - A_previous_base;
    S_B_length = norm(previousS_B);
    scalar = s/S_B_length;
    previousS_B = scalar*previousS_B;
    l = previousS_B + A_B;

    % take norm of vector to find length
    L = zeros(6,1);
    for i = 1:6
        L(i) = norm(l(:,i));
    end
end



