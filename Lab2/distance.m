function [distance] = distance(q1, q2)
    distance = sqrt((q1(1)-q2(1))^2 + (q1(2)-q2(2))^2 + (q1(3)-q2(3))^2 +(q1(4)-q2(4))^2);
end