 classdef enumBNO055_OpMode < uint32
    enumeration
        CONFIGMODE (0)      % Config Mode
        ACCONLY (1)         % None Fusion Mode
        MAGONLY (2)         % None Fusion Mode
        GYROONLY (3)        % None Fusion Mode
        ACCMAG (4)          % None Fusion Mode
        ACCGYRO (5)         % None Fusion Mode
        MAGGYRO (6)         % None Fusion Mode
        AMG (7)             % None Fusion Mode
        IMU (8)             % Fusion Mode
        COMPASS (9)         % Fusion Mode
        M4G (10)            % Fusion Mode
        NDOF_FMC_OFF (11)   % Fusion Mode
        NDOF (12)           % Fusion Mode
    end
end

