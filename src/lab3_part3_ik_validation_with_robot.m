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

% graph = graphing("part3_ik_validation.csv", pp);



try
    %define these position vectors in mm
    p1 = [45; 54; 24];
    p2 = [100; 0; 195];
    p3 = [41; -113; 111];
    q1 = pp.ik3001(p1);
    q2 = pp.ik3001(p2);
    q3 = pp.ik3001(p3);

    %Create empty matrix 10x4x4
    pp.closeGripper();
    pp.interpolate_jp(q1, 900);
    pause(2);
    
    close all;
    
    M = zeros(1, 4); %t, x, y, z, q1, q2, q3
    c = 0;
    
    q = pp.measured_js(1,0);
    q = q(1,:);
    mod.plot_arm(q);
    drawnow;
    
    tic
        
    %While loop runs for 60 seconds
    %Improvement note: Make non-blocking code, without while loop
    while (toc < 4)
        g = floor(toc)
        disp(c)
        if floor(toc) > c
            c = c + 1;
        end
        
        if c == 0
%             pp.servo_jp(q2);
            pp.interpolate_jp(q2, 900);
        elseif c == 1
%             pp.servo_jp(q3);
            pp.interpolate_jp(q3, 900);
        elseif c == 2
%             pp.servo_jp(q1);
            pp.interpolate_jp(q1, 900);
        end
        
        q = pp.measured_js(1,0);
        q = q(1,:);
        mod.plot_arm(q);
        drawnow;
        M(end+1, 1) = toc;
        M(end, 2:4) = pp.position(q); %position
        M(end, 5:7) = q; %joint angle
%         graph.record();
    end
    
    %Plot of Tip position vs Time
%         subplot(2, 1, 1)
%         hold on
%         plot(M(2:end, 1), M(2:end, 2), 'r-') %End Effector X Position
%         plot(M(2:end, 1), M(2:end, 3), 'g-') %End Effector Y Position
%         plot(M(2:end, 1), M(2:end, 4), 'b-') %End Effector Z Position
%         legend('End Effector X Position', 'End Effector Y Position', 'End Effector Z Position')
%         title("Plot of End Effector Position vs Time")
%         xlabel('Time (s)')
%         ylabel('End Effector Position (mm)')
%         hold off
%         
%         subplot(2, 1, 2)
%         hold on
%         plot(M(2:end, 1), M(2:end, 5), 'r-') %End Effector X Position
%         plot(M(2:end, 1), M(2:end, 6), 'g-') %End Effector Y Position
%         plot(M(2:end, 1), M(2:end, 7), 'b-') %End Effector Z Position
%         legend('q1 angle', 'q2 angle', 'q3 angle')
%         title("Plot of Joint Angle vs Time")
%         xlabel('Time (s)')
%         ylabel('Joint Angle (deg)')
%         hold off
        
          %Trajectory of this part
          plot3(M(2:end,2), M(2:end, 3), M(2:end, 4));
          title('Plot of Interpolated End Effector Path');
          xlabel('x (mm)');
          ylabel('y (mm)');
          zlabel('z (mm)');
          
%           writematrix(M(2:end, 2:4), 'RawMotion_TaskSpaceCoord_Trajectory.csv');



%     graph.plot3DMotion(q1, q2, q3);
    
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end


% Clear up memory upon termination
pp.shutdown()