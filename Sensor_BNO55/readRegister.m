function [value] = readRegister(i2cobj,SYS_STATUS)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


% System Status Check
% write command to read Status register
fwrite(i2cobj,SYS_STATUS)
% get the answer from the Status register
value = fread (i2cobj, 1); 

end 