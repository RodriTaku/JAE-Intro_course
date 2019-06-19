function [ Y ] = SE3_vee( X )

    Y = [SO3_vee(X(1:3,1:3));
                    X(1:3,4)];
end