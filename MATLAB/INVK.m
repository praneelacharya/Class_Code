
function [q_new,final_position,error_norm] = INVK(desired_position,current_theta)

franka_emika_FWK

theta1 = current_theta(1);
theta2 = current_theta(2);
theta3 = current_theta(3);
theta4 = current_theta(4);
theta5 = current_theta(5);
theta6 = current_theta(6);
theta7 = current_theta(7);

bTe = eval(A);  %This matrix gives us the postion & orientation of end effector when measured from base frame
init_position = bTe(1:3,4)   %Get the postion from Homogenous Matrix
jacob = eval(Jacob);

%%%%%% CALACULATION FOR LINEARIZATION %%%%%%
inv_jacob = pinv(jacob);  %Calculate the inverse jacobian
deltaX = desired_position - init_position;  %Give the required value of deltaX
deltaQ = inv_jacob*deltaX;   % q_new = q_old + delataQ

%%%%%% q_new = q_old + deltaQ %%%%%%
q_delta1 = theta1 + deltaQ(1);
q_delta2 = theta2 + deltaQ(2);
q_delta3 = theta3 + deltaQ(3);
q_delta4 = theta4 + deltaQ(4);
q_delta5 = theta5 + deltaQ(5);
q_delta6 = theta6 + deltaQ(6);
q_delta7 = theta7 + deltaQ(7);

q_new = [q_delta1,q_delta2,q_delta3,q_delta4,q_delta5,q_delta6,q_delta7]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%% Cross_check we pass this to FWK and see what orientation we get  %%%%
theta1 = q_new(1);
theta2 = q_new(2);
theta3 = q_new(3);
theta4 = q_new(4);
theta5 = q_new(5);
theta6 = q_new(6);
theta7 = q_new(7);

bTe = eval(A);
final_position = bTe(1:3,4)
abs_error_xyz = abs([final_position - (init_position+deltaX)]);
error_norm = norm(abs_error_xyz)
end
