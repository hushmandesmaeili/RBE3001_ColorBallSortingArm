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

% Creates Frame3D object
frame = Frame3D();

mod = Model(pp, frame);

graph = graphing("motionPlanning.csv", pp);

try 
    pp.closeGripper();
    close all;
    q1 = [0 50 17];
    q2 = [0 24 -17];
    q3 = [0 4 27.5];
    pp.servo_jp(q1);
    pause(1);
    tic
    c = 1;
    %use lab1_recordjoinval to get values
    while (toc < 5)
        if floor(toc) > c
            c = c + 1;
        end
        
        %change joint values to recorded positions
%         if c == 1
% %             pp.interpolate_jp([45 45 45], 900);
%             
        if c == 2
            pp.servo_jp(q2);
        elseif c == 3
            pp.servo_jp(q3);
        elseif c == 4
            pp.servo_jp(q1);
        end
        %continuously record joint angles and tip position into csv
        graph.record();
        disp(toc)
        
        
                
    end
    graph.plotMotion(q1, q2, q3);
    
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()