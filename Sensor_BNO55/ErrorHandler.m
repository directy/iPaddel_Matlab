function ErrorHandler(errorcode)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
% ErrorHandler

   
   switch errorcode
       case 1
           disp('Peripheral initialization error')
       case 2
           disp('System initialization error')
       case 3
           disp('Selftest result failed')
       case 4
           disp('Register map value out of range')
       case 5
           disp('Register map address out of range')
       case 6
           disp('Register map write error')
       case 7
           disp('BNO low power mode not available for selected operation mode')
       case 8
           disp('Accelerometer power mode not available')
       case 9
           disp('Fusion algorithm configuration error')
       case A
           disp('Sensor configuration error')
   end

end

