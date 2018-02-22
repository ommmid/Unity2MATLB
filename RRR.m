clear; clc;

%% MATLAB Simulation
L1 = Revolute('d',0.2,'a',0,'alpha',pi/2);
L2 = Revolute('d',0.2,'a',0,'alpha',pi/2);
L3 = Revolute('d',0.2,'a',0,'alpha',pi/2);
robot = SerialLink([L1 L2 L3]);
% reference configuration:
joints = [0,pi/2,0];
robot.plot(joints) 
robot.teach

% get the transformation in reference configuration (look at the DH.pptx for details)
F2 = robot.A(1:1,joints); % frame 2 expressed in frame 1
F3 = robot.A(1:2,joints); % frame 3 expressed in frame 1
Fe = robot.A(1:3,joints); % frame ee expressed in frame 1

L12 = robot.A(1,joints); % frame 2 expressed in frame 1
T12 = [L12.R, L12.t; 0, 0, 0, 1];
L23 = robot.A(2,joints); % frame 3 expressed in frame 2
T23 = [L23.R, L23.t; 0, 0, 0, 1];
L3e = robot.A(3,joints); % frame ee expressed in frame 3
T3e = [L3e.R, L3e.t; 0, 0, 0, 1];

% from right-handed coordinates to left-handed coordinates
T12_Unity = conM2U_H(T12) 
T23_Unity = conM2U_H(T23)
T3e_Unity = conM2U_H(T3e)

% working with quaternin is easier in Unity for more complicated
% transformations, very handy
[b12, v12, theta12] = conM2U_Q(T12) 
[b23, v23, theta23] = conM2U_Q(T23)
[b3e, v3e, theta3e] = conM2U_Q(T3e)

%% MATLAB as a Server to listen to Unity
clear; close all; format compact; clc;

L1 = Revolute('d',0.2,'a',0,'alpha',pi/2);
L2 = Revolute('d',0.2,'a',0,'alpha',pi/2);
L3 = Revolute('d',0.2,'a',0,'alpha',pi/2);
robot = SerialLink([L1 L2 L3]);
% reference configuration:
joints = [0,pi/2,0];
robot.plot(joints) 
robot.teach

tcpipServer = tcpip('0.0.0.0', 55000,'NetworkRole','Server', 'InputBufferSize', 100);
j=0; J1=0; J2=0; J3=0;
fopen(tcpipServer); 
while(1)
    j = j+1
    data = fread(tcpipServer, 45, 'char');
    dataChar = char(data);
    Ac = dataChar == 'A'; indxA = find(Ac); Ax = indxA(1);
    Bc = dataChar == 'B'; indxB = find(Bc); Bxx =  indxB(indxB > Ax); Bx = Bxx(1);
    Cc = dataChar == 'C'; indxC = find(Cc); Cxx =  indxC(indxC > Bx); Cx = Cxx(1);
    Dc = dataChar == 'D'; indxD = find(Dc); Dxx =  indxD(indxD > Cx); Dx = Dxx(1);
    
    Jv1(j) = str2double(dataChar(Ax+1:Bx-1));
    Jv2(j) = str2double(dataChar(Bx+1:Cx-1));
    Jv3(j) = str2double(dataChar(Cx+1:Dx-1));
    
    if (j>1 && (Jv1(j) ~= Jv1(j-1) || Jv2(j) ~= Jv2(j-1) || Jv3(j) ~= Jv3(j-1)))
        if ( Jv1(j) ~= Jv1(j-1) )
            J1 = Jv1(j);
        end;
        if ( Jv2(j) ~= Jv2(j-1) )
            J2 = Jv2(j);
        end;
        if ( Jv3(j) ~= Jv3(j-1) )
            J3 = Jv3(j);
        end;
        robot.plot(joints + [J1*pi/180, J2*pi/180, J3*pi/180]) ;
    end;
    [J1, J2, J3]
end;
fclose(tcpipServer);


 
