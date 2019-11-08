syms a;
syms alpha;
syms d;
syms theta;

syms d1;
syms a2;
syms a3;
syms d4;
syms d5;
syms d6;

syms q1;
syms q2;
syms q3;
syms q4;
syms q5;
syms q6;

A(a, alpha, d, theta) = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
    sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
    0, sin(alpha), cos(alpha), d;
    0, 0, 0, 1];

A1 = simplify(A(0, -pi/2, d1, q1))
A2 = simplify(A(a2, 0, 0, q2-pi/2));
A3 = simplify(A(a3, 0, 0, q3+pi/2));
A4 = simplify(A(0, -pi/2, 0, q4-pi/2));
A5 = simplify(A(0, pi/2, d5, q5+pi/2));
A6 = simplify(A(0, 0, d6, 0));

T01 = A1
T02 = simplify(A1*A2)
T03 = simplify(A1*A2*A3)
T04 = simplify(A1*A2*A3*A4)
T05 = simplify(A1*A2*A3*A4*A5)

T06 = simplify(A1*A2*A3*A4*A5*A6)
d06 = T06(1:3, 4);

Jv = simplify([diff(d06, q1) diff(d06, q2) diff(d06, q3) diff(d06, q4) diff(d06, q5) diff(d06, q6)])

Jw = [[0; 0; 1] T01(1:3,3) T02(1:3,3) T03(1:3,3) T04(1:3,3) T05(1:3,3)]