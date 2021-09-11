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
    %Create empty matrix 10x4x4
    T = zero(10, 4, 4);
    
    for n = 1:10
        %Move to non-zero position, then back to zero position
        pp.interpolate_jp([45 45 45], 1000);
        pause(2);
        pp.interpolate_jp([0 0 0]);
        pause(2)
        
        %Create Transform Matrix
        T(n, :, :) = pp.measured_cp();
    end
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end


% Clear up memory upon termination
pp.shutdown()