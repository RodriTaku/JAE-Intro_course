function [ Y ] = SE3_element( X )
    
    Y = [[SO3_element(X(1:3)), X(4:6)];
         [         zeros(1,3),      1]];
end