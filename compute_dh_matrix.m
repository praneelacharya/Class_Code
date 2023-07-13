

function A = compute_dh_matrix(r, alpha, d, theta)

    A = eye(4);
    
    Rotz_theta = [cos(theta),-sin(theta) 0 0;
                  sin(theta) cos(theta) 0 0;
                  0             0        1 0;
                  0             0       0  1];
              
    Transz_distanced = [1 0 0 0; 0 1 0 0; 0 0 1 d; 0 0 0 1];
    
    Transx_distancer = [1 0 0 r; 0 1 0 0; 0 0 1 0; 0 0 0 1];
    
    Rotx_alpha = [1,    0                0              0;
                  0     cos(alpha)      -sin(alpha)     0;
                  0     sin(alpha)       cos(alpha)     0;
                  0             0        0              1];
              
    
%      A = Rotz_theta * Transz_distanced * Transx_distancer * Rotx_alpha;

     A = Rotx_alpha * Transx_distancer * Transz_distanced * Rotz_theta;
                      
end

