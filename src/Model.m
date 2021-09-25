
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
            
            T00 = self.robot.T00();
            T01 = self.robot.T01();
            T02 = self.robot.T02(q);
            T03 = self.robot.T03(q);
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
            zlim([-10 250]);
            xlabel('x position')
            ylabel('y position')
            zlabel('z position')
            view(135, 25);  %Adjusts the plots default view, for better visualization
            
            % Frames 0 to Frame 4 shown on plot
            % Using Frame3D object
            self.frame.showFrame(T00, arrowSize);
            self.frame.showFrame(T01, arrowSize);
            self.frame.showFrame(T02, arrowSize);
            self.frame.showFrame(T03, arrowSize);
            self.frame.showFrame(T04, arrowSize);
            
            hold off
        end
        
        function plot_arm_fvk(self, q, P)
            
            arrowSize = 30; %arrow size for local frames
            
            T00 = self.robot.T00();
            T01 = self.robot.T01();
            T02 = self.robot.T02(q);
            T03 = self.robot.T03(q);
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
            zlim([-10 250]);
            xlabel('x position')
            ylabel('y position')
            zlabel('z position')
            view(135, 25);  %Adjusts the plots default view, for better visualization
            
            % Frames 0 to Frame 4 shown on plot
            % Using Frame3D object
            self.frame.showFrame(T00, arrowSize);
            self.frame.showFrame(T01, arrowSize);
            self.frame.showFrame(T02, arrowSize);
            self.frame.showFrame(T03, arrowSize);
            self.frame.showFrame(T04, arrowSize);
            
            magP = norm(P);
            
            quiver3(T04(1, 4), T04(2, 4), T04(3, 4), P(1), P(2), P(3), magP)  
            
            hold off
        end
        
    end
end

