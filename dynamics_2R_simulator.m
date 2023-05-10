% dynamics introduction
close all; 
clear all; 


% parameters of the robot manipulator
% length
L_1 = 0.6;          % m
L_2 = 0.5;          % m

% weight
m_1 = 1.0;          % kg
m_2 = 1.0;          % kg

% gravity defined as a constant
g = 9.81;           % m/s^2

% initial values for theta
theta_1    = deg2rad(0); 
theta_2    = deg2rad(90); 
vec_theta  = [theta_1; theta_2];
tau        = [0; 0]

% initial values for d theta
d_theta_1  = 0; 
d_theta_2  = 0; 
vec_d_theta = [d_theta_1; d_theta_2];

% initial values for dd theta
dd_theta_1 = 0; 
dd_theta_2 = 0;
vec_dd_theta = [dd_theta_1; dd_theta_2];

deltaT = 0.01; 


% NOTE: use the following 
theta_iter_save   = zeros(0,2);     % joint angles over time for plot
p_1_iter_save     = zeros(0,2);     % p1 position over time for plot
p_2_iter_save     = zeros(0,2);     % p2 position over time for plot


for t=0:deltaT:5

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    % implement here the mass matrix
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  

    
    % !! remove the following line if you finished your implementation
    M = eye(2); 

        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    % implement here the coriolis vector
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  


    % !! remove the following line if you finished your implementation
    c = [0; 0]

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    % implement here the gravity vector
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  

    % !! remove the following line if you finished your implementation
    g_vec = [0; 0]



        

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    % implement the inverse dynamics equation 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    
    % !! remove the following line if you finished your implementation
    vec_dd_theta = [0; 0]


    % integrate acceleration to get velocity
    vec_d_theta = vec_d_theta + deltaT * vec_dd_theta;


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    % OPTIONAL: implement model friction
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  


    % integrate velocity to get position
    vec_theta = vec_theta + deltaT * vec_d_theta;

    % forward kinematics to get cartesian point of the two joints
    x_1 = L_1 * cos(vec_theta(1)); 
    y_1 = L_1 * sin(vec_theta(1));

    x_2 = L_1 * cos(vec_theta(1)) + L_2*cos(vec_theta(1) + vec_theta(2));
    y_2 = L_1 * sin(vec_theta(1)) + L_2*sin(vec_theta(1) + vec_theta(2));

    p_1_iter_save       = [p_1_iter_save; [x_1, y_1]];
    p_2_iter_save       = [p_2_iter_save; [x_2, y_2]];
    theta_iter_save     = [theta_iter_save; vec_theta'];


    % update all variables for the next iteration
    theta_1   = vec_theta(1); 
    theta_2   = vec_theta(2); 
    d_theta_1 = vec_d_theta(1);
    d_theta_2 = vec_d_theta(2);
end






% plot everything
figure(1);
n = length(p_1_iter_save(:,1))
for i=1:n    
    % mark the joint weight in the plot
    plot(p_1_iter_save(i,1), p_1_iter_save(i,2), 'rx')
    hold on; 
    plot(p_2_iter_save(i,1), p_2_iter_save(i,2), 'rx')

    % draw two lines for each joint
    line([0 p_1_iter_save(i,1)], [0 p_1_iter_save(i,2)]);
    line([p_1_iter_save(i,1) p_2_iter_save(i,1)], [p_1_iter_save(i,2), p_2_iter_save(i,2)]);

    hold off; 
    axis([-1.2 1.2 -1.2 1.2]);
    grid on; 
    axis square; 
    drawnow; 
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% implement a visualization of the velocity of each joint
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  

