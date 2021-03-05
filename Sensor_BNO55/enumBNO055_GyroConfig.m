classdef enumBNO055_GyroConfig < uint32
    % Auto controlled in Fusion-Mode
    enumeration
        dps200 (0)      
        dps1000 (1)         
        dps500 (2)         
        dps250 (3)
        dps125 (4)
        Bandwidth_523 (0)         
        Bandwidth_230 (4)
        Bandwidth_116 (8)
        Bandwidth_47 (12)
        Bandwidth_23 (16)
        Bandwidth_12 (20)
        Bandwidth_64 (24)
        Bandwidth_32 (28)
        OpMode_Normal (0)
        OpMode_FastPowerUp (32)
        OpMode_DeepSuspend (64) 
        OpMode_Suspend (96)
        OpMode_AdvancedPowerSave (128)
    end
end