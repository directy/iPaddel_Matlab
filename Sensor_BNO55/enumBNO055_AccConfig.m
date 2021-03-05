classdef enumBNO055_AccConfig < uint32
    % Auto controlled in Fusion-Mode
    enumeration
        G_Range_2G (0)      
        G_Range_4G (1)         
        G_Range_8G (2)         
        G_Range_16G (3)        
        Bandwidth_7_81 (0)         
        Bandwidth_15_63 (4)
        Bandwidth_31_25 (8)
        Bandwidth_62_5 (12)
        Bandwidth_125 (16)
        Bandwidth_250 (20)
        Bandwidth_500 (24)
        Bandwidth_1000 (28)
        OpMode_Normal (0)
        OpMode_Suspend (32)
        OpMode_LowPower1 (64) 
        OpMode_Standby (96)
        OpMode_LowPower2 (128)
        OpMode_DeepSuspend (160)
    end
end

