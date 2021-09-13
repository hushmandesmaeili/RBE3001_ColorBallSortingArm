classdef graphing
    %collects data into a csv file
    %graphs various plots from a csv file
    
    
    properties
        csv;
        robot;
    end
    
    methods
        function self = graphing(file, bot)
            self.csv = file;
            self.robot = bot;
        end
        
        %records current joint values and position to csv file
        function record(self)
            T = self.robot.measured_js(1, 0);
            T = T(:, 1);
            T = [T self.robot.measured_cp()];
            G = readmatrix(self.csv);
            writematrix([G;T], self.csv);
        end
        
        %plots all subplots
        function plotMotion(self)
            
        end
        
    end
    
end