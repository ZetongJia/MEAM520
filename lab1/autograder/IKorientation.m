syms theta1;
syms theta2;
syms theta3;
syms theta4;
syms theta5;
syms d1;
syms d2;
syms d3;
syms d4;
syms d5;
syms r1;
syms r2;
syms r3;

d1 = 76.20;
d2 = 146.05;
d3 = 187.325;
d5 = 85.725-9.525;
    
X = T0e(1,4);
Y = T0e(2,4);
Z = T0e(3,4);
r1 = sqrt(X^2, Y^2);
r2 = d1-Z;
r3 = sqrt(r1^2+r2^2);
theta1 = atan2(Y,X);
theta2 = acos((d3^2-d2^2-r3^2)/(-2d2*r3));
theta3 = acos((r3^2-d2^2-d3^2)/(-2d2*d3));


a1 = [cos(theta1) 0 -sin(theta1) 0; sin(theta1) 0 cos(theta1) 0; 0 -1 0 d1; 0 0 0 1];
a2 = [sin(theta2) cos(theta2) 0 d2*sin(theta2); -cos(theta2) sin(theta2) 0 -d2*cos(theta2); 0 0 1 0; 0 0 0 1];
a3 = [-sin(theta3) -cos(theta3) 0 -d3*sin(theta3); cos(theta3) -sin(theta3) 0 d3*cos(theta3); 0 0 1 0; 0 0 0 1];

T03 = a1*a2*a3;
R03 = T03(1:3,1:3);
R45 = R03'*T0e(1:3,1:3);

theta5 = -acos(R45(3,2));
theta4 = asin(R45(2,3));

