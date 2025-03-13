%Code to obtain the joint angles using inverse kinematics
%Final transformation matrix for the robot
T_Final = [0 0, 1, 350;
        0, -1, 0, -14.6;
        1, 0, 0, 38;
        0, 0, 0, 1];

%We call the inverse kinematic function, using the matrix as input and
%recieving angles as output:
ang = Reto_Inverse_Kinematic(T_Final)

%Inverse kinematic function which recieves matrix as input:
function a = Reto_Inverse_Kinematic(T6)
    %First the links' lengths are defined.
    l1 = 160; 
    l2 = 200.05; 
    l3 = 195.60; 
    l4 = 45.95; 
    l5 = 149.89;
    l6 = 31.35;
    %Then the rotation matrix and position vector are extracted from the
    %transformation matrix:
    R6 = extr_rot_from_transf(T6);
    P6 = extr_pos_from_transf(T6);
    %Then the position of the wrist of the robot is found:
    g = [0; l4; l5];
    Pc = P6 - R6*g;
    %The coordenates of the wrist position are saved in different
    %variables:
    Pcx = Pc(1);
    Pcy = Pc(2);
    Pcz = Pc(3);

    %Starting at a top view:
    %Alpha1 is found first which will in turn help to find a1:
    sen_alpha_1 = l6/(sqrt(Pcx^2+Pcy^2));
    cos_alpha_1 = sqrt(1-sen_alpha_1^2);%Can have a negative or positive value, it can be changed depending on the needed position and rotation.
    alpha_1 = atan2(sen_alpha_1,cos_alpha_1);
    %Then phi1 is found which will also help to find a1:
    phi_1 = atan2(Pcy,Pcx);
    %a1 is found by the angle relation between a1, phi 1 and alpha1.
    a1 = phi_1 - alpha_1; %left shoulder configuration.
    %d1 is obtained since it is a distance that will help to obtain
    %following angles:
    d1 = sqrt(Pcx^2+Pcy^2-l6^2);

    %Continouing now with the side view
    %We obtain c_squared, ci being the distance from the origin to Pc
    c_cuadrado = d1^2 + (Pcz - l1)^2;
    %We obtain the beta3 angle which will help to obtain a2, it is obtained
    %geometrically
    cos_beta3 = (c_cuadrado - (l2^2 + l3^2))/(2*l2*l3);
    sen_beta3 = sqrt(1-cos_beta3^2);%Can have a negative or positive value, it can be changed depending on the needed position and rotation.
    beta3 = atan2(sen_beta3,cos_beta3);
    %Now a2 is obtained by using known and previously determined values:
    a2 = atan2(d1,Pcz-l1) - atan2(l3*sen_beta3,l2+l3*cos_beta3);
    %a3 is obtained with already obtained values:
    sen_a3 = (l2*cos(a2)-(Pcz-l1));
    cos_a3 = d1-l2*sin(a2);
    a3 = atan2(sen_a3,cos_a3)-a2;%Para que nat ponga atencion
    %Determine DH parameters until position 4 for the three angles and 
    %obtain the 0T4 matrix:
    DH_0R4 = [
    0, 0, l1, 0;
    0, -pi/2, l6, -pi/2;
    l2, 0, 0, 0;
    0, -pi/2, l3, 0;
    ];
    joint_types_0R4 = [0, 0, 0, 0];%Joint type which is cylindrical
    joint_values_0R4 = [a1, a2, a3, 0];%Joint angle which are the already obtained
    %Obtain the transformation and rotation matrix at 4:
    T_0R4 = forward_kinematics_general(DH_0R4, joint_types_0R4, joint_values_0R4);
    R_0R4 = extr_rot_from_transf(T_0R4);
    %Obtain the rotation matrix from point 4 to 6:
    T_4R6 = R_0R4'*R6;
    %Obtain the values of the angles obtaining the cosine and sine from the
    %4R6 matrix, using as reference a ZY rotation:
    cos_a5 = T_4R6(3,3);
    sen_a5 = -T_4R6(3,1);
    if abs(cos_a5) < 1e-6
        if sen_a5<0
            a5 = -pi/2;
        else
            a5 = pi/2;
        end
    else
        a5 = atan2(sen_a5,cos_a5);
    end
    cos_a4 = T_4R6(2,2);
    sen_a4 = -T_4R6(1,2);
    if abs(cos_a4) < 1e-6
        if sen_a4<0
            a4 = -pi/2;
        else
            a4 = pi/2;
        end
    else
        a4 = atan2(sen_a4, cos_a4);
    end
    %Save all the angles in a vector:
    a = [a1; a2; a3; a4; a5];
end


%Functions:

function R = extr_rot_from_transf(T) %The input is the homogenous transformation matrix which is 4x4
%This functions obtains the rotation matrix from the homogenous
%transformation matrix. This is done by extracting the elements that belong
%to it, which are the ones in rows 1 to 3 and columns 1 to 3.
    if size(T,1) ~= 4 || size(T,2) ~= 4 %This evaluates if the matrix is 4x4
        error('Input must be a 4x4 transformation matrix.');
    end
    
    % Extract the rotation matrix (upper-left 3x3 submatrix)
    R = T(1:3, 1:3);%The output is the extracted rotation matrix.
end

function p = extr_pos_from_transf(T)%The input is the homogenous transformation matrix which is 4x4
%This functions obtains the position vector from the homogenous
%transformation matrix. This is done by extracting the elements that belong
%to it, which are the ones in rows 1 through 3 in the fourth column.
    if size(T,1) ~= 4 || size(T,2) ~= 4 %This evaluates if the matrix is 4x4
        error('Input must be a 4x4 transformation matrix.');
    end
    
    % Extract the position vector (last column, first three rows)
    p = T(1:3, 4);%The output is the extracted position vector.
end

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
