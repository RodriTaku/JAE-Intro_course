function [ Y ] = SE3_coordinates( X, varargin )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
% Euler angles: Algorithm by Gregory G. Slabaugh
    Y = [ SO3_coordinates( X, varargin );
                                X(1:3,4)];
end