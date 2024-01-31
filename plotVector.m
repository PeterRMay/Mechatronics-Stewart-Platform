function plotVector(origin, vector, color)
    plot3([origin(1) vector(1)], [origin(2) vector(2)], [origin(3) vector(3)],'Color',color)
    hold on
end