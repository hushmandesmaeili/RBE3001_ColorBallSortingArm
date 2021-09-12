classdef Frame3D
    %Frame3D class creates a local reference frame at a position, with a
    %given orientation. 
    
    %Potential additions to class include adding properties and options for
    %the frames. E.g. arrowheads, frame sizes, color options for vectors,
    %manually
    
    properties
        
    end
    
    methods
        
        function showFrame(obj, HT_matrix, arrowSize)
            %showFrame method passes to parameters, HT_matrix and
            %arrowSize. 
            
            %HT_matrix is a 4x4 homogenous transformation
            %matrix. E.g. given an intermediate homogeneous transformation
            %matrix, we can see the frame of the joint by extracting the
            %position from matrix and setting that as the initial position
            %of vector defined by quiver3. Then, the orientation 
            %(direction) of the vector is determined by the R part of the
            %HT matrix. 
            
            %arrowSize is the unit length of the plotted vector. 
            
            % X-axis, red vector
            quiver3(HT_matrix(1, 4), HT_matrix(2, 4), HT_matrix(3, 4), arrowSize*HT_matrix(1, 1), arrowSize*HT_matrix(2, 1), arrowSize*HT_matrix(3, 1), 'LineWidth', 2, 'Color', 'r');

            % Y-axis, green vector
            quiver3(HT_matrix(1, 4), HT_matrix(2, 4), HT_matrix(3, 4), arrowSize*HT_matrix(1, 2), arrowSize*HT_matrix(2, 2), arrowSize*HT_matrix(3, 2), 'LineWidth', 2, 'Color', 'g');

            % Z-axis, blue vector
            quiver3(HT_matrix(1, 4), HT_matrix(2, 4), HT_matrix(3, 4), arrowSize*HT_matrix(1, 3), arrowSize*HT_matrix(2, 3), arrowSize*HT_matrix(3, 3), 'LineWidth', 2, 'Color', 'b')
        
        end
    end
end

