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

graph = graphing("motionPlanning.csv", pp)

try 
    pp.closeGripper();
    close all;
    %use lab1_recordjoinval to get values
    while (toc < 7)
        if floor(toc) > c
            c = c + 1;
        end
        
        %change joint values to recorded positions
        if c == 1
            pp.interpolate_jp([45 45 45], 1000);
        elseif c == 2
            pp.interpolate_jp([0 0 0], 1000);
        elseif c == 3
            pp.interpolate_jp([0 -10 0], 1000);
        end
        %continuously record joint angles and tip position into csv
        graph.record();
        
    end
    
    graph.plotMotion();
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()