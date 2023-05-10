% dynamics introduction
close all; 
clear all; 

L_1 = 0.6;          % m
L_2 = 0.5;          % 
% m
m_1 = 1.0;          % kg
m_2 = 1.0;          % kg

% gravity defined as a constant
g = 9.81;           % m/s^2


% initial values for theta
theta_1    = 0; 
theta_2    = deg2rad(0); 
vec_theta  = [theta_1; theta_2];

% initial values for d theta
d_theta_1  = 0; 
d_theta_2  = 0; 
vec_d_theta = [d_theta_1; d_theta_2];

% initial values for dd theta
dd_theta_1 = 0; 
dd_theta_2 = 0;
vec_dd_theta = [dd_theta_1; dd_theta_2];


deltaT = 0.01; 

d_theta_iter_save = zeros(0,2);
 theta_iter_save  = zeros(0,2);
p_1_iter_save     = zeros(0,2); 
p_2_iter_save     = zeros(0,2);


for t=0:deltaT:5

    % implement here the mass matrix
    M_11 = m_1*L_1^2 + m_2*(L_1^2 + 2*L_1*L_2*cos(theta_2) + L_2^2); 
    M_12 = m_2*(L_1*L_2*cos(theta_2) + L_2^2);
    M_21 = m_2*(L_1*L_2*cos(theta_2) + L_2^2);
    M_22 = m_2*L_2^2;
    
    M = [M_11 M_12; 
         M_21 M_22];
    
    % implement here the coriolis vector
    c_1 = -m_2*L_1*L_2*sin(theta_2)*(2*d_theta_1*d_theta_2 + d_theta_2^2); 
    c_2 =  m_2*L_1*L_2*d_theta_1*sin(theta_2);
    
    c = [c_1; c_2]
        
    % implement here the gravity vector
    g_1 = (m_1+m_2)*L_1*g*cos(theta_1) + m_2*L_2*g*cos(theta_1+theta_2);
    g_2 = m_2      *L_2*g*cos(theta_1+theta_2)
    
    g_vec = [g_1; g_2]; 




   
        

      
    % inverse problem 
    tau = [0; 0];

    if (t > 2)
        tau = [4; 0]
    end

    if (t > 3)
        tau = [0; 0]
    end
    
    vec_dd_theta =  inv(M)*(tau - c - g_vec);





    % integrate acceleration to get velocity
    vec_d_theta = vec_d_theta + deltaT * vec_dd_theta;

    % model friction
    friction_decrease = 0.01;
    vec_d_theta = (1 - friction_decrease)*vec_d_theta;

    % integrate velocity to get position
    vec_theta = vec_theta + deltaT * vec_d_theta;

    % implement forward kinematics to get cartesian point of the two joints
    x_1 = L_1 * cos(vec_theta(1)); 
    y_1 = L_1 * sin(vec_theta(1));

    x_2 = L_1 * cos(vec_theta(1)) + L_2*cos(vec_theta(1) + vec_theta(2));
    y_2 = L_1 * sin(vec_theta(1)) + L_2*sin(vec_theta(1) + vec_theta(2));

    p_1_iter_save       = [p_1_iter_save; [x_1, y_1]];
    p_2_iter_save       = [p_2_iter_save; [x_2, y_2]];
    
    theta_iter_save     = [theta_iter_save; vec_theta'];
    d_theta_iter_save   = [d_theta_iter_save; vec_d_theta'];

    theta_1 = vec_theta(1); 
    theta_2 = vec_theta(2); 

    d_theta_1 = vec_d_theta(1);
    d_theta_2 = vec_d_theta(2);

end






n = length(p_1_iter_save(:,1)); 
f = cell(n,1) ; 

figure(3);
for i=1:n
    i
    
    plot(p_1_iter_save(i,1), p_1_iter_save(i,2), 'rx')
    hold on; 
    plot(p_2_iter_save(i,1), p_2_iter_save(i,2), 'rx')

    line([0 p_1_iter_save(i,1)], [0 p_1_iter_save(i,2)]);
    line([p_1_iter_save(i,1) p_2_iter_save(i,1)], [p_1_iter_save(i,2), p_2_iter_save(i,2)]);

    hold off; 
    axis([-1.2 1.2 -1.2 1.2]);
    grid on; 
    axis square; 
    drawnow; 
    %pause(0.001);

    f{i} = getframe(gcf) ;
end

obj = VideoWriter('curve.mpeg', 'MPEG-4');
obj.Quality = 50;
obj.FrameRate = 60;
open(obj);
for i = 1:n
    writeVideo(obj, f{i}) ;
end
obj.close();
