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
    pp.closeGripper();
    pp.interpolate_jp([45 45 45], 1000);
%       pp.interpolate_jp([0 0 0], 1000);
%     pp.servo_jp([0 0 0]);
%     pp.goal_cp()
%     pp.setpoint_cp()
%     pp.measured_cp()
%     pause(2)
%     pp.goal_cp()
%     pp.setpoint_cp()
%     pp.measured_cp()
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end


% Clear up memory upon termination
pp.shutdown()