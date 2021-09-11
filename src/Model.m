
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
           
            
            T01 = [1, 0, 0, 0;
                   0, 1, 0, 0;
                   0, 0, 1, 55;
                   0, 0, 0, 1];
            
            T = self.robot.measured_cp(q);
            
            %Origin
            plot3(0, 0, 0, 'b.')
            
            %Joint 1 point
            plot3(0, 0, T01(3, 4), 'b.')
            
            
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

