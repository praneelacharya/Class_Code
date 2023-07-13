%This is forward Kimenatics funtion
%Input Franka 7 joint Values
%Output Franka Position

function position = FWK(current_theta)

%%%%%GETTING READY%%%%
franka_emika_FWK;
%%%%% I am Ready %%%%%

theta1 = current_theta(1);
theta2 = current_theta(2);
theta3 = current_theta(3);
theta4 = current_theta(4);
theta5 = current_theta(5);
theta6 = current_theta(6);
theta7 = current_theta(7);

bTe = eval(A);  %This matrix gives us the postion & orientation of end effector when measured from base frame
position = bTe(1:3,4);   %Get the postion from Homogenous Matrix

end