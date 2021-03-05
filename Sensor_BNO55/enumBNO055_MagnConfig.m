classdef enumBNO055_MagnConfig < uint32
    % Auto controlled in Fusion-Mode
    enumeration
        DataOutputeRate2Hz (0)      
        DataOutputeRate6Hz (1)         
        DataOutputeRate8Hz (2)         
        DataOutputeRate10Hz (3)
        DataOutputeRate15Hz (4)
        DataOutputeRate20Hz (5)
        DataOutputeRate25Hz (6)
        DataOutputeRate30Hz (7)
        OpModeLowPower (0)         
        OpModeRegular (8)
        OpModeEnhancedRegular (16)
        OpModeHighAccuray (24)
        PowerMode_Normal (0)
        PowerMode_Sleep (64) 
        PowerMode_Suspend (96)
        PowerMode_ForceMode (128)
    end
end
