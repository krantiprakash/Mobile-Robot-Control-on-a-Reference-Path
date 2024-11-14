% Project Code
% Kranti Prakash (M23IRM006)

clc
clear all
close all

global k1 k2 r vd;

%.............Initialize Variables

xi = 20;        % x initial
yi = 20;        % y initial
vi = 0;        % v initial
thetai = 0;    % theta initial
variablei = [xi ; yi ; vi ; thetai];

% .................For circle having radius(r) and control inputs k1 and k2
% vd velocity delay

k1 = 1;     % gain K1
k2 = 1;     % gain K2
r = 3;        % radius r
vd =15;       % desired velocity

% ................. For having equal distribution of the 2*pi
phi = linspace(0, 2*pi , 1000);



%..............Time Span
time = 0:0.01:20;

% ..............ode45 for solving the statespace of differential equations

[t,variable]=ode45(@diffeq,time,variablei);

% ..............Storing the calculated data from the ode45 in the column
% matrix
for i= 1:length(t)

    x = variable(i,1);
    y = variable(i,2);
    theta = variable(i,3);
    v = variable(i,4);

    w(i) = (1/r)*(v + (k1*(x*cos(theta) + y*sin(theta))));
    u(i) = -k2*(v - vd);
    
end

%..............Plotting the reference cicle and the trajectory of the agent
figure(1)
plot(variable(:,1),variable(:,2), r*cos(phi), r*sin(phi),2,2,'o',0,0,'*')
grid on 
axis equal


%..............Plottitng the graph between 'u' linear speed control
figure(2)
plot(time,u)
grid on 
xlabel('time');
ylabel('Linear speed');
title('Linear speed vs time');

% ............Plotting the graph between 'w' angular speed control
figure(3)
plot(time,w)
grid on 
xlabel('time');
ylabel('Angular speed');
title('Angular speed vs time')

% ..............ode45 function having the state space of the differential
% equations and storing the values in the matrix.
function dvariable = diffeq(time, variable)

    global k1 k2 r vd;

    x = variable(1,1);
    y = variable(2,1);
    theta = variable(3,1);
    v = variable(4,1);


    w = (1/r)*(v +( k1*(x*cos(theta) + y*sin(theta))));
    u = -k2*(v - vd);

    dvariable(1,1) = v*cos(theta);
    dvariable(2,1) = v*sin(theta);
    dvariable(3,1) = w;
    dvariable(4,1) = u;

end