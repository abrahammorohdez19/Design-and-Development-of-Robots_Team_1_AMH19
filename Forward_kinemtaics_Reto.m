%Carlos Matanzo Hermoso A01736696
% MATLAB Implementation of Generalized Forward Kinematics using DH Parameters
%In this code a DH table (with assumed measurements added to the table) and
%positions will be inputted into a function that will first change some
%values depending on the position, and then turn them into a matrix so that
%then through a chain of operations we can obtain the forward kinematics in
%a transformation matrix. There is another function called from the other
%function which compute sthe transformation matrix for each row of the
%table.

% Generalized function to compute forward kinematics, important notes to
% have in mind about the variables:
% DH_table: DH parameter table
% joint_types: Vector indicating if joint is prismatic (1), revolute (0) or
% exertor (2)
% joint_values: Current values of joint variables, position or angle


% Define missing link lengths from Cylindrical Robot (these are assumed as
% they were not in the problem)
l1 = 160; 
l2 = 200.05; 
l3 = 195.60; 
l4 = 45.95; 
l5 = 149.89;
l6 = 31.35;

% Define DH parameter table
DH_ProblemSituation = [
    0, 0, l1, 0;
    0, -pi/2, l6, -pi/2;
    l2, 0, 0, 0;
    0, -pi/2, l3, 0;
    0, -pi/2, l4, 0;
    0, pi/2, l5, 0
];

% Define joint type vectors (0 = Revolute, 1 = Prismatic, 2=Exertor)
% (This tells us what type of joint it was, and so which DH parameter could
% change with time and could be inputted to the function):
joint_types_ProblemSituation = [0, 0, 0, 0, 0, 2];


% Define joint values for each robot 
% (These are the values that can change from the DH parameters, basically
% we are saying at which distance or which angle certain part of the robot
% moved from the diagram):

joint_values_ProblemSituation = [0, 1.28, 0.25, 0, 1.54, 0]; %1:Cambiando signo, 2: el mismo pero cambiando signo, 3: manda=este-pi/2-a2, 4:negativo de lo que salga, 5: manda=este en el signo contrario-pi/2


% Compute FK for each robot (We call the aforemention function)
T_ProblemSituation = forward_kinematics_general(DH_ProblemSituation, joint_types_ProblemSituation, joint_values_ProblemSituation)
%The results are logical and shown in the command window


%Functions


% Function to compute transformation matrix using DH parameters
function T = DH_transform(a, alpha, d, theta)
%The parameters will be introduced into the matrix. Which will end up being
%a transformation matrix. This is for modified convention.
    T = [
        cos(theta), -sin(theta), 0, a;
        sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha);
        sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), d*cos(alpha);
        0, 0, 0, 1;
    ];%The output is a transformation matrix that represents the row of the DH table.
end

function T_final = forward_kinematics_general(DH_table, joint_types, joint_values)%Inputs are  Dh table of 4 columns, and two vectors which joint types means: 1-Prismatic, 0-Revolute, and 2 exertor, and joint values which basically gives the angle or distance change of this joint.
%This function takes a DH table, the change the variable parameter has
%suffered, and the type of joints. In turn the function will give back a
%transformation matrix that represents the forward kinematics of the robot.
    T_final = eye(4);
    for i = 1:size(DH_table, 1)%A for is created to evaluate each row of the table
        a = DH_table(i, 1);
        alpha = DH_table(i, 2);
        d = DH_table(i, 3);
        theta = DH_table(i, 4);%The values from each row of the table is being put into variables
        
        if joint_types(i) == 1  % Prismatic joint
            d = d+joint_values(i);%Changes variable d, since is the one that can change
        elseif joint_types(i) == 0  % Revolute joint
            theta = theta+joint_values(i);%Changes variable theta, since is the one that can change
        end
        
        T_i = DH_transform(a, alpha, d, theta);%The transformation matrix of the row is created, by calling other function
        T_final = T_final * T_i;%The final transformation matrix (forward kinematics) is being created until the last row is evaluated, this is the output.
    end
end
