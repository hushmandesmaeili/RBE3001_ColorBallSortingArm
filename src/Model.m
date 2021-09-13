
classdef Model < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties 
        
        robot;
        frame;
    end
    
    methods
        
        function self = Model(bot, frame)
            
            self.robot = bot;
            self.frame = frame;
        end
        
        %Method plots arm given joint configuration q. Local frames are
        %attached to each joint.
        function plot_arm(self, q)
            
            arrowSize = 30; %arrow size for local frames
            
            L0 = 55;
            L1 = 40;
            L2 = 100;
            L3 = 100;
            
            DH0 = [         0  L0  0    0];
            DH1 = [q(1, 1)     L1  0  -90];
            DH2 = [q(1, 2)-90   0  L2   0];
            DH3 = [q(1, 3)+90   0  L3  90];
            %
            %             T01 = self.robot.dh2fk([DH0])
            %             T02 = self.robot.dh2fk([DH0 DH1])
            %             T03 = self.robot.dh2fk([DH0 DH1 DH2])
            %             T04 = self.robot.dh2fk([DH0 DH1 DH2 DH3])
            
            T00 = [1, 0, 0, 0;
                   0, 1, 0, 0;
                   0, 0, 1, 0;
                   0, 0, 0, 1];
            
            T01 = [1, 0, 0, 0;
                   0, 1, 0, 0;
                   0, 0, 1, 55;
                   0, 0, 0, 1];
            
            T02 = [cos((pi*q(1, 1))/180),  0, -sin((pi*q(1, 1))/180),  0;
                   sin((pi*q(1, 1))/180),  0,  cos((pi*q(1, 1))/180),  0;
                   0, -1,                      0, 95;
                   0, 0, 0, 1];
            
            T03 = [cos((pi*q(1, 1))/180)*cos((pi*(q(1, 2) - 90))/180), -cos((pi*q(1, 1))/180)*sin((pi*(q(1, 2) - 90))/180), -sin((pi*q(1, 1))/180), 100*cos((pi*q(1, 1))/180)*cos((pi*(q(1, 2) - 90))/180);
                   sin((pi*q(1, 1))/180)*cos((pi*(q(1, 2) - 90))/180), -sin((pi*q(1, 1))/180)*sin((pi*(q(1, 2) - 90))/180),  cos((pi*q(1, 1))/180), 100*sin((pi*q(1, 1))/180)*cos((pi*(q(1, 2) - 90))/180);
                   -sin((pi*(q(1, 2) - 90))/180),                       -cos((pi*(q(1, 2) - 90))/180),                      0,                  95 - 100*sin((pi*(q(1, 2) - 90))/180);
                   0, 0, 0, 1];
            
            T04 = self.robot.fk3001(q);
            
            %x, y, and z coordinates of base and joints and end effector
            XJVals = [0 T01(1, 4) T02(1, 4) T03(1, 4) T04(1, 4)];
            YJVals = [0 T01(2, 4) T02(2, 4) T03(2, 4) T04(2, 4)];
            ZJVals = [0 T01(3, 4) T02(3, 4) T03(3, 4) T04(3, 4)];
         
            plot3(XJVals, YJVals, ZJVals, 'k-o', 'LineWidth', 2)
            hold on
            grid on;
            xlim([-250 250]);
            ylim([-250 250]);
            zlim([-250 250]);
            xlabel('x position')
            ylabel('y position')
            zlabel('z position')
            
            % Frames 0 to Frame 4 shown on plot
            % Using Frame3D object
            self.frame.showFrame(T00, arrowSize);
            self.frame.showFrame(T01, arrowSize);
            self.frame.showFrame(T02, arrowSize);
            self.frame.showFrame(T03, arrowSize);
            self.frame.showFrame(T04, arrowSize);
        end
        
    end
end

