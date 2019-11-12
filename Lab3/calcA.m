function A = calcA(a, r, d, theta)
    A = [cos(theta), -sin(theta)*cos(r), sin(theta)*sin(r), a*cos(theta);
    sin(theta), cos(theta)*cos(r), -cos(theta)*sin(r), a*sin(theta);
    0, sin(r), cos(r), d;
    0, 0, 0, 1];
end
