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

% % Creates Frame3D object
% frame = Frame3D();

% mod = Model(pp, frame);
close all;

pp.closeGripper();

p1 = [45; 54; 24];
p2 = [100; 0; 195];
p3 = [41; -113; 111];
p = [p1 p2 p3 p1];
q = zeros(4,3);



try     
    
    threeAngs = readmatrix('CubicMotion_3JointAngs_Trajectory.csv');
    jointSpace = readmatrix('CubicMotion_JointSpaceAngs_Trajectory.csv');
    taskSpace = readmatrix('CubicMotion_TaskSpaceCoord_Trajectory.csv');
    
    plot3(threeAngs(2:end, 1), threeAngs(2:end, 2), threeAngs(2:end, 3))
    hold on
    plot3(jointSpace(2:end, 1), jointSpace(2:end, 2), jointSpace(2:end, 3))
    plot3(taskSpace(2:end, 1), taskSpace(2:end, 2), taskSpace(2:end, 3))
    legend('Only Sending Joint Angles', 'Trajectory Planning in Joint Space', 'Trajectory Planning in Task Space')
    title('Comparison of EE Paths')
    xlabel('X [mm]')
    ylabel('Y [mm]')
    zlabel('Z [mm]')
    
    
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()