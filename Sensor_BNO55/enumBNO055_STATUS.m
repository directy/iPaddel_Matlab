classdef enumBNO055_STATUS < uint32
    enumeration
        SystemIdle (0) 
        SystemError (1) 
        InitializingPeripherals (2) 
        SystemInitialization (3) 
        ExecutingSelftest (4)
        SensorFusionAlgorithmRunning (5)
        SystemRunningWithoutFusionAlgorithm (6)
    end
end