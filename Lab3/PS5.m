syms a;
syms alpha;
syms d;
syms theta;

syms d1;
syms d2;
syms d3;
syms d4;
syms d5;

syms theta1;
syms theta2;
syms theta3;
syms theta4;
syms theta5;

syms c1 c2 c3 c4 c5 s1 s2 s3 s4 s5 

%calculate A1, A2, A3
A(a, alpha, d, theta) = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
    sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
    0, sin(alpha), cos(alpha), d;
    0, 0, 0, 1];

A1_1 = A(0, -pi/2, d1, theta1);
A2_1 = A(d2, 0, 0, theta2-pi/2);
A3_1 = A(d3, 0, 0, theta3+pi/2);

%simplified A1, A2, A3
A1 = [cos(theta1), 0, -sin(theta1), 0; sin(theta1), 0, cos(theta1), 0; 0, -1, 0, d1; 0, 0, 0, 1];
A2 = [sin(theta2) cos(theta2), 0, d2*sin(theta2); -cos(theta2), sin(theta2), 0, -d2*cos(theta2); 0, 0, 1, 0; 0, 0, 0, 1];
A3 = [-sin(theta3), -cos(theta3), 0, -d3*sin(theta3); cos(theta3), -sin(theta3), 0, d3*cos(theta3); 0, 0, 1, 0; 0, 0, 0, 1];
T = A1*A2*A3;

%simplified d03 from T
d03 = [cos(theta1)*d2*sin(theta2) - cos(theta1)*d3*sin(theta2)*sin(theta3) + cos(theta1)*cos(theta2)*cos(theta3)*d3;...
    d2*sin(theta1)*sin(theta2) - d3*sin(theta1)*sin(theta2)*sin(theta3) + cos(theta2)*cos(theta3)*d3*sin(theta1);...
    d1 + cos(theta2)*d2 - cos(theta2)*d3*sin(theta3) - cos(theta3)*d3*sin(theta2)];

%question 1
d03_diff_theta1 = diff(d03,theta1);
d03_diff_theta2 = diff(d03,theta2);
d03_diff_theta3 = diff(d03,theta3);
d03_diff = [d03_diff_theta1, d03_diff_theta2, d03_diff_theta3];
d03_diff = simplify(d03_diff);

%question 2
Jv1 = cross([0;0;1],(d03-[0;0;0]));
Jv2 = cross(A1(1:3,1:3)*[0;0;1],(d03-A1(1:3,4)));
A12 = A1*A2;
Jv3 = cross(A12(1:3, 3),(d03-A12(1:3,4)));
Jv = simplify([Jv1, Jv2, Jv3]);


