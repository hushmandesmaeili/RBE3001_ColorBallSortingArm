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

try 
    pp.closeGripper();
    close all;
    tic
    
    %While loop runs for 60 seconds
    %Improvement note: Make non-blocking code, without while loop
    while (toc < 60)
        q = pp.measured_js(1,0);
        q = q(1,:);
        mod.plot_arm(q);
        drawnow;
        pause(0.05);
    end
%     mod.plot_arm([0 0 0]);
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()