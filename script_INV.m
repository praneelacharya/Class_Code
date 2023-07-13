
%Writeen in 2018 by Praneel 

%This is an Inverse Kinematics Problem" We want to go to desired state;

%Inputs 
clc
close all
clear all

% desired_position = [0.658474;0;0.231877]

desired_position = [ .74, 0.305, .060]';
% theta1 = 0.014;
% theta2 = -0.758;
% theta3 = 0.01934;
% theta4 = -2.34;
% theta5 = 0.02335;
% theta6 = 1.539;
% theta7 = 0.0754;


% theta1 = -0.02610;
% theta2 = -0.475603;
% theta3 = 0.0125458;
% theta4 = -2.76215;
% theta5 = 0.027318;
% theta6 = 2.255;
% theta7 = 0.6914;

theta1 = .0891011;
theta2 = -.370138;
theta3 = -.00125879;
theta4 = -2.16146;
theta5 = 0.00448061;
theta6 = 1.75424;
theta7 = 0.829817;


current_theta = [theta1,theta2,theta3,theta4,theta5,theta6,theta7];
[start_position] = FWK(current_theta)

plot3(start_position(1),start_position(2),start_position(3),'g-o','MarkerSize',10)
xlabel('X'); ylabel('Y'); zlabel('Z');
text(start_position(1),start_position(2),start_position(3),'start')
hold on
grid on

plot3(desired_position(1),desired_position(2),desired_position(3),'p','MarkerSize',10)
text(desired_position(1),desired_position(2),desired_position(3),'desired')

error = 100;

while error>(10^-4)
    
[q_new,intermidiate_position,error] = INVK(desired_position,current_theta);
current_theta = q_new;
plot3(intermidiate_position(1),intermidiate_position(2),intermidiate_position(3),'*','MarkerSize',14)
end

