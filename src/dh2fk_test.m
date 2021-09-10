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

try
    %length in mm
    L0 = 55;
    L1 = 40;
    L2 = 100;
    L3 = 100;
    syms theta1S;
    syms theta2S;
    syms theta3S;
    
    DH0 = [0 L0 0 0];
    DH1 = [theta1S L1 0 -90];
    DH2 = [theta2S-90 0 L2 0];
    DH3 = [theta3S+90 0 L3 90];
    M = [DH0; DH1; DH2; DH3];
        
    pp.dh2fk([DH0; DH1; DH2])

%     T01 = [1, 0, 0, 0;
%            0, 1, 0, 0;
%            0, 0, 1, 55;
%            0, 0, 0, 1];
   
%     T02 = [cos((pi*theta1S)/180),  0, -sin((pi*theta1S)/180),  0;
%            sin((pi*theta1S)/180),  0,  cos((pi*theta1S)/180),  0;
%                                0, -1,                      0, 95;
%                                0,  0,                      0,  1];
%     
%     
%       T03 = [cos((pi*theta1S)/180)*cos((pi*(theta2S - 90))/180), -cos((pi*theta1S)/180)*sin((pi*(theta2S - 90))/180), -sin((pi*theta1S)/180), 100*cos((pi*theta1S)/180)*cos((pi*(theta2S - 90))/180);
%              sin((pi*theta1S)/180)*cos((pi*(theta2S - 90))/180), -sin((pi*theta1S)/180)*sin((pi*(theta2S - 90))/180),  cos((pi*theta1S)/180), 100*sin((pi*theta1S)/180)*cos((pi*(theta2S - 90))/180);
%                                   -sin((pi*(theta2S - 90))/180),                       -cos((pi*(theta2S - 90))/180),                      0,                  95 - 100*sin((pi*(theta2S - 90))/180);
%                                                               0,                                                   0,                      0,                                                      1];

%     Complete Homogeneous Transformation matrix for arm
%     T04 = [ cos((pi*theta1S)/180)*cos((pi*(theta2S - 90))/180)*cos((pi*(theta3S + 90))/180) - cos((pi*theta1S)/180)*sin((pi*(theta2S - 90))/180)*sin((pi*(theta3S + 90))/180), -sin((pi*theta1S)/180), cos((pi*theta1S)/180)*cos((pi*(theta2S - 90))/180)*sin((pi*(theta3S + 90))/180) + cos((pi*theta1S)/180)*cos((pi*(theta3S + 90))/180)*sin((pi*(theta2S - 90))/180), 100*cos((pi*theta1S)/180)*cos((pi*(theta2S - 90))/180) + 100*cos((pi*theta1S)/180)*cos((pi*(theta2S - 90))/180)*cos((pi*(theta3S + 90))/180) - 100*cos((pi*theta1S)/180)*sin((pi*(theta2S - 90))/180)*sin((pi*(theta3S + 90))/180);
%  sin((pi*theta1S)/180)*cos((pi*(theta2S - 90))/180)*cos((pi*(theta3S + 90))/180) - sin((pi*theta1S)/180)*sin((pi*(theta2S - 90))/180)*sin((pi*(theta3S + 90))/180),  cos((pi*theta1S)/180), sin((pi*theta1S)/180)*cos((pi*(theta2S - 90))/180)*sin((pi*(theta3S + 90))/180) + sin((pi*theta1S)/180)*cos((pi*(theta3S + 90))/180)*sin((pi*(theta2S - 90))/180), 100*sin((pi*theta1S)/180)*cos((pi*(theta2S - 90))/180) + 100*sin((pi*theta1S)/180)*cos((pi*(theta2S - 90))/180)*cos((pi*(theta3S + 90))/180) - 100*sin((pi*theta1S)/180)*sin((pi*(theta2S - 90))/180)*sin((pi*(theta3S + 90))/180);
%                                            - cos((pi*(theta2S - 90))/180)*sin((pi*(theta3S + 90))/180) - cos((pi*(theta3S + 90))/180)*sin((pi*(theta2S - 90))/180),                      0,                                             cos((pi*(theta2S - 90))/180)*cos((pi*(theta3S + 90))/180) - sin((pi*(theta2S - 90))/180)*sin((pi*(theta3S + 90))/180),                                                              95 - 100*cos((pi*(theta2S - 90))/180)*sin((pi*(theta3S + 90))/180) - 100*cos((pi*(theta3S + 90))/180)*sin((pi*(theta2S - 90))/180) - 100*sin((pi*(theta2S - 90))/180);
%                                                                                                                                                                  0,                      0,                                                                                                                                                                 0,                                                                                                                                                                                                                                  ];
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end


% Clear up memory upon termination
pp.shutdown()