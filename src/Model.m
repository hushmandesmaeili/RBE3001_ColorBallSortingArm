
classdef Model < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    
    
    properties 
    
        robot;
       % pol;
        
    end
    
    methods
        
        function self = Model(bot)
            
            self.robot = bot;
        %    gs.pol = java.lang.Boolean(false);
            
        end
               
        function plot_arm(self, q)
            
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
            
            T01 = [1, 0, 0, 0;
                0, 1, 0, 0;
                0, 0, 1, 55;
                0, 0, 0, 1];
            
            
            
            T02 = [cos((pi*q(1, 1))/180),  0, -sin((pi*q(1, 1))/180),  0;
                sin((pi*q(1, 1))/180),  0,  cos((pi*q(1, 1))/180),  0;
                0, -1,                      0, 95;
                0,  0,                      0,  1];
            
            
            T03 = [cos((pi*q(1, 1))/180)*cos((pi*(q(1, 2) - 90))/180), -cos((pi*q(1, 1))/180)*sin((pi*(q(1, 2) - 90))/180), -sin((pi*q(1, 1))/180), 100*cos((pi*q(1, 1))/180)*cos((pi*(q(1, 2) - 90))/180);
                sin((pi*q(1, 1))/180)*cos((pi*(q(1, 2) - 90))/180), -sin((pi*q(1, 1))/180)*sin((pi*(q(1, 2) - 90))/180),  cos((pi*q(1, 1))/180), 100*sin((pi*q(1, 1))/180)*cos((pi*(q(1, 2) - 90))/180);
                -sin((pi*(q(1, 2) - 90))/180),                       -cos((pi*(q(1, 2) - 90))/180),                      0,                  95 - 100*sin((pi*(q(1, 2) - 90))/180);
                0,                                                   0,                      0,                                                      1];
            
            T04 = self.robot.fk3001(q);
            
            %x, y, and z coordinates of base and joints and end effector
            XJVals = [0 T01(1, 4) T02(1, 4) T03(1, 4) T04(1, 4)]
            YJVals = [0 T01(2, 4) T02(2, 4) T03(2, 4) T04(2, 4)]
            ZJVals = [0 T01(3, 4) T02(3, 4) T03(3, 4) T04(3, 4)]
            
            %x, y, and z coordinates of x-, y-, and z-axis and respective
            %origin for base, multiplied by 10 so i can better see it
            XAJ0 = 10*[XJVals(1, 1)+T01(1, 1); YJVals(1, 1)+T01(2, 1); ZJVals(1, 1)+T01(3, 1)]; %Each row represents axis represented in new frame
            YAJ0 = 10*[XJVals(1, 1)+T01(1, 2); YJVals(1, 1)+T01(2, 2); ZJVals(1, 1)+T01(3, 2)];
            ZAJ0 = 10*[XJVals(1, 1)+T01(1, 3); YJVals(1, 1)+T01(2, 3); ZJVals(1, 1)+T01(3, 3)];
            
            %x, y, and z coordinates of x-, y-, and z-axis and respective
            %origin for Joint 1
            XAJ1 = 10*[XJVals(1, 2)+T02(1, 1); YJVals(1, 2)+T02(2, 1); ZJVals(1, 2)+T02(3, 1)]
            YAJ1 = [XJVals(1, 2) XJVals(1, 2)+T02(1, 2); YJVals(1, 2) YJVals(1, 2)+T02(2, 2); ZJVals(1, 2) ZJVals(1, 2)+T02(3, 2)];
            ZAJ1 = [XJVals(1, 2) XJVals(1, 2)+T02(1, 3); YJVals(1, 2) YJVals(1, 2)+T02(2, 3); ZJVals(1, 2) ZJVals(1, 2)+T02(3, 3)]
            
            plot3(XJVals, YJVals, ZJVals, 'k-o', 'LineWidth', 3)
            hold on
            
            %Axis for Base
            plot3([XJVals(1,1) XAJ0(1, 1)], [XJVals(1,1) XAJ0(2, 1)], [XJVals(1,1) XAJ0(3, 1)], 'r', 'LineWidth', 5) %plots line between point (origin of frame) and endpoint of frame axis
            plot3([YJVals(1,1) YAJ0(1, 1)], [YJVals(1,1) YAJ0(2, 1)], [YJVals(1,1) YAJ0(3, 1)], 'g', 'LineWidth', 5)
            plot3([ZJVals(1,1) ZAJ0(1, 1)], [ZJVals(1,1) ZAJ0(2, 1)], [ZJVals(1,1) ZAJ0(3, 1)], 'b', 'LineWidth', 5)

            %Axis for Joint 1
            plot3([XJVals(1, 2) XAJ1(1, 1)], [YJVals(1, 2) XAJ1(2, 1)], [ZJVals(1, 2) XAJ1(3, 1)], 'r', 'LineWidth', 5)
            plot3(YAJ1(1, 1:2), YAJ1(2, 1:2), YAJ1(3, 1:2), 'g', 'LineWidth', 5)
            plot3(ZAJ1(1, 1:2), ZAJ1(2, 1:2), ZAJ1(3, 1:2), 'b', 'LineWidth', 5)
            
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

