%% Control to facet controller implementation
% Formal Methods in Robotics Fall 2019
% HW 3
%% Author : Mohammed Tousif Zaman

clc;
close all;
clearvars;
syms u1 u2 u3

% System matrices
A = [-1 -1; -2 1];
B = [-2; -2];
a = [3; 1];

% Vertices of the polytope
v1 = [-1; 0];
v2 = [1; 1];
v3 = [1; -1];

% Normal Vectors to the facets
n1 = [1; 0];
n2 = [-1/sqrt(5); -2/sqrt(5)];
n3 = [-1/sqrt(5); 2/sqrt(5)];

%% Plot the polytope

figure

% plot the line segments joining the vertices
plot([v1(1), v2(1)], [v1(2), v2(2)], '.-b', 'MarkerSize', 10)
text(-1.35,0.5,'Facet F3')
text(-1.5,-0.1,'(-1,0) v1')
hold on

plot([v2(1), v3(1)], [v2(2), v3(2)], '.-b', 'MarkerSize', 10)
text(1.2,0.8,'Facet F1')
text(1,1,'(1,1) v2')
hold on

plot([v1(1), v3(1)], [v1(2), v3(2)], '.-b', 'MarkerSize', 10)
text(0.15,-1.35,'Facet F2')
text(1,-1,'(1,-1) v3')
hold on

% plot the normal vectors
% quiver(n1(1), n1(2), n1(1)*1.5, n1(2)*1.5)
% hold on

% divide the line into equal parts
n=1;
t = linspace(0,1,n+2);  
t = t(2:(end-1));       
v = v3 - v1;
x = v1(1) + t*v(1);
y = v1(2) + t*v(2);
v = v / norm(v);
line([x(1)+v(2), x(1)],[y(1)-v(1), y(1)],'Color','#A2142F');  
text(2,0,'N1')

hold on

v = v2 - v3;
x = v3(1) + t*v(1);
y = v3(2) + t*v(2);
v = v / norm(v);
line([x(1)+v(2), x(1)],[y(1)-v(1), y(1)],'Color','#A2142F');
text(-0.6,-1.5,'N2')

hold on

v = v1 - v2;
x = v2(1) + t*v(1);
y = v2(2) + t*v(2);
v = v / norm(v);
line([x(1)+v(2), x(1)],[y(1)-v(1), y(1)],'Color','#A2142F');
text(-0.6,1.5,'N3')

% define the axes for the plot
xlim([-3 3])
ylim([-3 3])
xlabel('x1')
ylabel('x2')
title('Control to facet problem on a polytope')
set(gcf, 'Position',  [100, 100, 750, 750])

%% Compute the Necessary Conditions for affine control feedback to exist

% The inequality (<= or >=) is determined by the sign on u1/u2/u3 and
% checked using vpa(ineq_u1_1) for example, if sign on u1/u2/u3 is negative
% then the inequality reverses.

fprintf('Inequalities for u1\n')

ineq_u1_1 = n2'*B*u1 + n2'*(A*v1 + a);
fprintf('u1 <= %s\n', solve(ineq_u1_1))

ineq_u1_2 = n3'*B*u1 + n3'*(A*v1 + a);
fprintf('u1 >= %s\n', solve(ineq_u1_2))

ineq_u1_3 = n2'*B*u1 + n3'*B*u1 + n2'*(A*v1 + a) + n3'*(A*v1 + a);
fprintf('u1 < %s\n', solve(ineq_u1_3))

fprintf('Inequalities for u2\n')

ineq_u2_1 = n1'*B*u2 + n1'*(A*v2 + a);
fprintf('u2 < %s\n', solve(ineq_u2_1))

ineq_u2_2 = n3'*B*u2 + n3'*(A*v2 + a);
fprintf('u2 >= %s\n', solve(ineq_u2_2))

fprintf('Inequalities for u3\n')

ineq_u3_1 = n1'*B*u3 + n1'*(A*v3 + a);
fprintf('u3 < %s\n', solve(ineq_u3_1))

ineq_u3_2 = n2'*B*u3 + n2'*(A*v3 + a);
fprintf('u3 <= %s\n', solve(ineq_u3_2))

%% Implement the controller

% Solve for F and g by solving the below equation 8 from the paper, 

syms f1 f2 g

A = [-1 0 1; 1 1 1; 1 -1 1];
X = [f1; f2; g];
b = [1; 0; -1/6];

% problem is now to solve the linear equation Ax = b

X = A\b;
fprintf('Solving for F and g below \n')
fprintf('f1 = %f\n', X(1))
fprintf('f2 = %f\n', X(2))
fprintf('g = %f\n', X(3))

%% Run ODE45 to simulate the system state trajectory
% Choose different initial conditions
[t,y] = ode45(@odefun,[0 20],[0; 0], 'r');
p1 = plot(y(:,1), y(:,2), 'r')

[t,y] = ode45(@odefun,[0 20],[0.35; -0.45], 'g');
p2 = plot(y(:,1), y(:,2), 'g')

[t,y] = ode45(@odefun,[0 20],[-1; 0]);
p3 = plot(y(:,1), y(:,2), 'black')

legend([p1 p2 p3], {'Path 1','Path 2','Path 3'})

function dydt = odefun(t,y)
% odefun:  Evaluate the system trajectory for exiting the polytope

    % System matrices
    A = [-1 -1; -2 1];
    B = [-2; -2];
    a = [3; 1];
    %  F = [f1 f2] calculated above
    F = [-0.5417 0.0833];
    % g calculated above as well
    g = 0.4583;
    % System equation
    dydt = (A + B*F)*y + (B*g + a);
end
