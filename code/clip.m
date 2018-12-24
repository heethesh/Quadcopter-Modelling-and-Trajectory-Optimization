function ret = clip(x, min, max)
x(x > max) = max;
x(x < min) = min;
ret = x;
end