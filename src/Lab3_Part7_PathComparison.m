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
    
    CubicTaskSpace = readmatrix('CubicMotion_TaskSpaceCoord_Trajectory.csv');
    QuinticTaskSpace = readmatrix('QuinticMotion_TaskSpaceCoord_Trajectory.csv');
    
    plot3(CubicTaskSpace(2:end, 1), CubicTaskSpace(2:end, 2), CubicTaskSpace(2:end, 3))
    hold on
    plot3(QuinticTaskSpace(2:end, 1), QuinticTaskSpace(2:end, 2), QuinticTaskSpace(2:end, 3))
    legend('Cubic Trajectory Planning in Task Space', 'Quintic Trajectory Planning in Task Space')
    title('Comparison of Cubic and Quintic Paths')
    xlabel('X [mm]')
    ylabel('Y [mm]')
    zlabel('Z [mm]')
    
    
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()