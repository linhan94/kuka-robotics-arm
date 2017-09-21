function theta = ikine_closedform(matrix)
    %Initialize
    theta = zeros(size(matrix,3), 5);
    d1 = 0.147;
    d5 = 0.218;
    a1 = 0.033;
    a2 = 0.155;
    a3 = 0.135;
    
    for i = 1:size(matrix,3)
        rotation = matrix(1:3,1:3,i);
        positionX = matrix(1,4,i);
        positionY = matrix(2,4,i);
        
        theta(i,1) = atan(positionY/positionX);
        theta(i,5) = atan2(rotation(1,1)*sin(theta(i,1)) - rotation(2,1)*...
            cos(theta(i,1)), rotation(1,2)*sin(theta(i,1)) - rotation(2,2)*...
            (cos(theta(i,1))));
        
        matrix_0_1 = [cos(theta(i,1)),0,sin(theta(i,1)),0;
                      sin(theta(i,1)),0,-cos(theta(i,1)),0;
                      0,1,0,d1;
                      0,0,0,1];
        matrix_4_5 = [cos(theta(i,5)),-sin(theta(i,5)),0,0;
                      sin(theta(i,5)),cos(theta(i,5)),0,0;
                      0,0,1,d5;
                      0,0,0,1];
                  
        %matrix(1,4,i) = matrix(1,4,i)+cos(theta(i,1))*a1;
        %matrix(2,4,i) = matrix(2,4,i)+sin(theta(i,1))*a1;
        matrix_1_4 = (matrix_0_1)\matrix(:,:,i)/(matrix_4_5);
        
        positionX_1_4 = matrix_1_4(1,4);
        positionY_1_4 = matrix_1_4(2,4);
        rotation_1_4 = matrix_1_4(1:3,1:3);
        
        ((positionX_1_4)^2+(positionY_1_4)^2-(a2)^2-(a3)^2)/(2*a2*a3)
        theta(i,3) = acos(((positionX_1_4)^2+(positionY_1_4)^2-(a2)^2-(a3)^2)/(2*a2*a3));
        theta(i,2) = atan2(-positionX_1_4, positionY_1_4)-atan2(a3*sin(theta(1,3)),...
            a2+a3*cos(theta(i,3)));
        theta(i,4) = atan2(-rotation_1_4(2,1), -rotation_1_4(1,1))-theta(i,2)-theta(i,3);
    end
end