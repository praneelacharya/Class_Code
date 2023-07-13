
% Writeen by : Praneel Acharya 2018
%The goal of this program is to get the end effector transformation matrix
%Using DH paramters. Pos Extract & Jack

% clc
% close all
% clear all

syms theta1 theta2 theta3 theta4 theta5 theta6 theta7  real
% [a,d,alpha,theta]

% For Joint 1
a  = 0; %a
d  = 0.333; %d
alpha  = 0; %alpha
theta  = theta1;

A1 = compute_dh_matrix(a,alpha,d,theta);

% For Joint 2
a  = 0; %a
d  = 0; %d
alpha  = -pi/2; %alpha
theta  = theta2;

A2 = compute_dh_matrix(a,alpha,d,theta);

% For Joint 3
a  = 0; %a
d  = 0.316; %d
alpha  = pi/2; %alpha
theta  = theta3;

A3 = compute_dh_matrix(a,alpha,d,theta);

% For Joint 4
a  = 0.0825; %a
d  = 0; %d
alpha  = pi/2; %alpha
theta  = theta4;

A4 = compute_dh_matrix(a,alpha,d,theta);

% For Joint 5
a  = -0.0825; %a
d  = 0.384; %d
alpha  = -pi/2; %alpha
theta  = theta5;

A5 = compute_dh_matrix(a,alpha,d,theta);

% For Joint 6
a  = 0; %a
d  = 0; %d
alpha  = pi/2; %alpha
theta  = theta6;

A6 = compute_dh_matrix(a,alpha,d,theta);

% For Joint 7
a  = 0.088; %a
d  = 0.0; %d
alpha  = pi/2; %alpha
theta  = theta7;

A7 = compute_dh_matrix(a,alpha,d,theta);

% For Flange
a  = 0; %a
d  = 0.107; %d
alpha  = 0; %alpha
theta  = 0;

flange = compute_dh_matrix(a,alpha,d,theta);

% For Hand
a  = 0; %a
d  = 0.101; %d
alpha  = 0; %alpha
theta  = 0;

hand = compute_dh_matrix(a,alpha,d,theta);

%%%% Total Transformation is %%%%%%

A = A1*A2*A3*A4*A5*A6*A7*flange*hand;


%%% Let's get the three functions of x,y,z in terms of q it f(q) %%%
pos_x = A(1,4);
pos_y = A(2,4);
pos_z = A(3,4);
pos = [pos_x;pos_y;pos_z];

Jacob = jacobian(pos,[theta1 theta2 theta3 theta4 theta5 theta6 theta7]);
orientation = A(1:3,1:3); %This is the Rotation Matrix
%%%%%%%%%%%%  END %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



