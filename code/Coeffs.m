function [a, b, c] = Coeffs(x,y)
    coefficients = polyfit(x, y, 2);
    a = coefficients(1);
    b = coefficients(2);
    c = coefficients(3);
end

