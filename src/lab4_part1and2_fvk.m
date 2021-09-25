clear
clear java
clear classes;

vid = hex2dec('16c0');
pid = hex2dec('0486');

disp (vid);
disp (pid);

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

% Create a PacketProcessor object to send data to the nucleo firmware
pp = Robot(myHIDSimplePacketComs); 

%% CALCULATION
%DH parameters
syms theta1 theta2 theta3;

%Link Lengths
L0 = 55; %mm
L1 = 40; %mm
L2 = 100; %mm
L3 = 100; %mm

%Declare our DH_Table
dhTable = [0             L0 0    0;
           theta1        L1 0   -90;
           (theta2-90)   0  L2  0;
           (theta3+90)   0  L3  90];

%Intermediate Transform Matrices
T01 = pp.dh2mat(dhTable(1, :));
T02 = T01 * pp.dh2mat(dhTable(2, :));
T03 = T02 * pp.dh2mat(dhTable(2, :));
T04 = T03 * pp.dh2mat(dhTable(2, :));

%Position Vector of Final Transform Matrix
P = T04(1:3, 4);

%Calculate position jacobian
Jp1 = diff(P, theta1); %Column 1 = derivative of position vector w.r.t. theta 1
Jp2 = diff(P, theta2);%Column 2 = derivative of position vector w.r.t. theta 2
Jp3 = diff(P, theta3); %Column 3 = derivative of position vector w.r.t. theta 3
Jp = [Jp1 Jp2 Jp3];

%Calculate orientation jacobian
%all joints are resolute => use z component of orientation matrix for each
%joint
z1 = T01(1:3, 3); %Column 1
z2 = T02(1:3, 3); %Column 2
z3 = T03(1:3, 3); %Column 3
Jo = [z1 z2 z3];

%Calculate Final Jacobian
J = [Jp; Jo]












