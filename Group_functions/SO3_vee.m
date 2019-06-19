function [ Y ] = SO3_vee( X )

    Y = [X(3,2);
         X(1,3);
         X(2,1)];
end