function plotComponents(vector, color)
    zeros2 = zeros(1,2);
    plot3([0 vector(1)],zeros2,zeros2, 'Color',color) %plot x
    hold on
    plot3(zeros2,[0 vector(2)],zeros2, 'Color',color) %plot y
    plot3(zeros2,zeros2,[0 vector(3)], 'Color',color) %plot z
end