classdef clsBNO055 < handle
    %BNO055 Smart sensor combining accelerometer, gyroscope, magnetometer
    % The smart sensor BNO055 is a System in Package (SiP) solution that 
    % integrates a triaxial 14-bit accelerometer, an accurate close-loop 
    % triaxial 16-bit gyroscope, a triaxial geomagnetic sensor and a 
    % 32-bit microcontroller running the BSX3.0 FusionLib software. 
    % This smart sensor is significantly smaller than comparable solutions. 
    % By integrating sensors and sensor fusion in a single device, 
    % the BNO055 makes integration easy, avoids complex multivendor 
    % solutions and thus simplifies innovations, e.g. novel applications 
    % such as IoT hardware. The BNO055 is the perfect choice for AR, 
    % immersive gaming, personal health and fitness, indoor navigation and 
    % any other application requiring context awareness. It is ideally 
    % suited for demanding applications such as augmented reality, 
    % navigation, gaming, robotics, or industrial applications.   
    % 
    % https://www.bosch-sensortec.com/products/smart-sensors/bno055.html
    % https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
    %
    % You have to install first: 
    % Instrument Control Toolbox Support Package for National Instruments NI-845x I2C/SPI Interface 
    %
    % NI-845x I2C/SPI Interface enables you to use the NI-845x adaptor to 
    % communicate using I2C or SPI interfaces to your chips, circuit boards,
    % or sensors remotely from a computer running MATLAB®.

   
    properties (SetAccess = private)

        Page_1= struct(...
        "Page_ID"          , 0x07,... % Page ID
        "ACC_Config"       , 0x08,... % ACC_PWR_Mode <2:0> ACC_BW <2:0> ACC_Range <1:0>
        "MAG_Config"       , 0x09,... % MAG_Power_mode <1:0> MAG_OPR_Mode <1:0> MAG_Data_output_rate <2:0>
        "GYR_Config_0"     , 0x0A,... % GYR_Bandwidth <2:0> GYR_Range <2:0>
        "GYR_Config_1"     , 0x0B,... % GYR_Power_Mode <2:0>
        "ACC_Sleep_Config" , 0x0C,... % SLP_DURATION <3:0> SLP_MODE
        "GYR_Sleep_Config" , 0x0D,... % AUTO_SLP_DURATION <2:0> SLP_DURATION <2:0>
        "INT_MSK"          , 0x0F,... % ACC_NM ACC_AM ACC_HIGH_GH GYR_HIGH_RATE
        "GYRO_INT_EN"      , 0x10,... % ACC_NM ACC_AM ACC_HIGH_G GYR_HIGH_RATE GYRO_AM
        "ACC_AM_THRES"     , 0x11,... % Accelerometer Any motion threshold
        "ACC_INT_Settings" , 0x12,... % HG_Z_AXIS HG_Y_AXIS HG_X_ AXIS AM/NM_Z_AXIS AM/NM_Y_AXIS AM/NM_ X_AXIS AM_DUR <1:0>
        "ACC_HG_DURAION"   , 0x13,... % Accelerometer High G Duration
        "ACC_HG_THRES"     , 0x14,... % Accelerometer High G Threshold
        "ACC_NM_THRE"      , 0x15,... % Accelerometer NO/SLOW motion threshold
        "ACC_NM_SET"       , 0x16,... % NO/SLOW Motion Duration <5:0> SMNM
        "GYR_INT_SETING"   , 0x17,... % HR_FILT AM_FILT HR_Z_AXIS HR_Y_AXIS HR_X_AXIS AM_Z_AXIS AM_Y_AXIS AM_X_AXIS
        "GYR_HR_X_SET"     , 0x18,... % HR_X_THRES_HYST <1:0> HR_X_Threshold <4:0>
        "GYR_DUR_X"        , 0x19,... % HR_X_Duration
        "GYR_HR_Y_SET"     , 0x1A,... % HR_Y_THRES_HYST <1:0> HR_Y_Threshold <4:0>
        "GYR_DUR_Y"        , 0x1B,... % HR_Y_Duration
        "GYR_HR_Z_SET"     , 0x1C,... % R_Z_THRES_HYST <1:0> HR_Z_Threshold <4:0>
        "GYR_DUR_Z"        , 0x1D,... % HR_Z_Duration
        "GYR_AM_THRES"     , 0x1E,... % Gyro Any Motion Threshold <6:0>
        "GYR_AM_SET"       , 0x1F,... % Awake Duration <1:0> Slope Samples <1:0>
        "UNIQUE_ID"        , 0x5F ... % 5F - 50 BNO unique ID
        )
        Page_0= struct(...
        'CHIP_ID'          , 0x00,...  % BNO055 CHIP ID
        'ACC_ID'           , 0x01,... % ACC chip ID
        'MAG_ID'           , 0x02,... % MAG chip ID
        'GYR_ID'           , 0x03,... % GYRO chip ID
        'SW_REV_ID_LSB'    , 0x04,... % SW Revision ID <7:0>
        'SW_REV_ID_MSB'    , 0x05,... % SW Revision ID <15:8>
        'BL_Rev_ID'        , 0x06,... % Bootloader Version
        'ACC_DATA_X_LSB'   , 0x08,... % Acceleration Data X <7:0>
        'ACC_DATA_X_MSB'   , 0x09,... % Acceleration Data X <15:8>
        'ACC_DATA_Y_LSB'   , 0x0A,... % Acceleration Data Y <7:0>
        'ACC_DATA_Y_MSB'   , 0x0B,... % Acceleration Data Y <15:8>
        'ACC_DATA_Z_LSB'   , 0x0C,... % Acceleration Data Z <7:0>
        'ACC_DATA_Z_MSB'   , 0x0D,... % Acceleration Data Z <15:8>
        'MAG_DATA_X_LSB'   , 0x0E,... % Magnetometer Data X <7:0>
        'MAG_DATA_X__MSB'  , 0x0F,... % Magnetometer Data X <15:8>
        'MAG_DATA_Y_LSB'   , 0x10,... % Magnetometer Data Y <7:0>
        'MAG_DATA_Y__MSB'  , 0x11,... % Magnetometer Data Y <15:8>
        'MAG_DATA_Z_LSB'   , 0x12,... % Magnetometer Data Z <7:0>
        'MAG_DATA_Z_MSB'   , 0x13,... % Magnetometer Data Z <15:8>
        'GYR_DATA_X_LSB'   , 0x14,... % Gyroscope Data X <7:0>
        'GYR_DATA_X_MSB'   , 0x15,... % Gyroscope Data X <15:8>
        'GYR_DATA_Y_LSB'   , 0x16,... % Gyroscope Data Y <7:0>
        'GYR_DATA_Y_MSB'   , 0x17,... % Gyroscope Data Y <15:8>
        'GYR_DATA_Z_LSB'   , 0x18,... % Gyroscope Data Z <7:0>
        'GYR_DATA_Z_MSB'   , 0x19,... % Gyroscope Data Z <15:8>
        'EUL_Heading_LSB'  , hex2dec('1A'),... % Heading Data <7:0>
        'EUL_Heading_MSB'  , 0x1B,... % Heading Data <15:8>
        'EUL_Roll_LSB'     , hex2dec('1C'),... % Roll Data <7:0>
        'EUL_Roll_MSB'     , 0x1D,... % Roll Data <15:8>
        'EUL_Pitch_LSB'    , hex2dec('1E'),... % Pitch Data <7:0>
        'EUL_Pitch_MSB'    , 0x1F,... % Pitch Data <15:8>
        'QUA_Data_w_LSB'   , 0x20,... % Quaternion w Data <7:0>
        'QUA_Data_w_MSB'   , 0x21,... % Quaternion w Data <15:8>
        'QUA_Data_x_LSB'   , 0x22,... % Quaternion x Data <7:0>
        'QUA_Data_x_MSB'   , 0x23,... % Quaternion x Data <15:8>
        'QUA_Data_y_LSB'   , 0x24,... % Quaternion y Data <7:0>
        'QUA_Data_y_MSB'   , 0x25,... % Quaternion y Data <15:8>
        'QUA_Data_z_LSB'   , 0x26,... % Quaternion z Data <7:0>
        'QUA_Data_z_MSB'   , 0x27,... % Quaternion z Data <15:8>
        'LIA_Data_X_LSB'   , 0x28,... % Linear Acceleration Data X <7:0>
        'LIA_Data_X_MBS'   , 0x29,... % Linear Acceleration Data X <15:8>
        'LIA_Data_Y_LSB'   , 0x2A,... % Linear Acceleration Data Y <7:0>
        'LIA_Data_Y_MSB'   , 0x2B,... % Linear Acceeration Data Y <15:8>
        'LIA_Data_Z_LSB'   , 0x2C,... % Linear Acceleration Data Z <7:0>
        'LIA_Data_Z_MSB'   , 0x2D,... % Linear Acceleration Data Z <15:8> 
        'GRV_Data_X_LSB'   , 0x2E,... % Gravity Vector Data X <7:0>
        'GRV_Data_X_MSB'   , 0x2F,... % Gravity Vector Data X <15:8>
        'GRV_Data_Y_LSB'   , 0x30,... % Gravity Vector Data Y <7:0>
        'GRV_Data_Y_MSB'   , 0x31,... % Gravity Vector Data Y <15:8>
        'GRV_Data_Z_LSB'   , 0x32,... % Gravity Vector Data Z <7:0>
        'GRV_Data_Z_MSB'   , 0x33,... % Gravity Vector Data Z <15:8>
        'TEMP'             , 0x34,... % Temperature
        'CALIB_STAT'       , 0x35,... % SYS Calib Status 0:3 GYR Calib Status 0:3 ACC Calib Status 0:3 MAG Calib Status 0:3
        'ST_RESULT'        , 0x36,... % ST_MCU ST_GYR ST_MAG ST_ACC
        'INT_STA'          , 0x37,... % ACC_NM ACC_AM ACC_HI GH_G GYR_HIG H_RATEGYRO_AM
        'SYS_CLK_STATUS'   , 0x38,... % ST_MAIN_CLK
        'SYS_STATUS'       , hex2dec('39'),... % System Status Code
        'SYS_ERR'          , 0x3A,... % System Error Code
        'UNIT_SEL'         , 0x3B,... % ORI_Android_Windows TEMP_Unit EUL_Unit GYR_Unit ACC_Unit
        'OPR_MODE'         , hex2dec('3D'),... % Operation Mode <3:0>
        'PWR_MODE'         , 0x3E,... % Power Mode <1:0>
        'SYS_TRIGGER'      , 0x3F,... % CLK_SEL RST_INT RST_SYS Self_Test
        'TEMP_SOURCE'      , 0x40,... % TEMP_Source <1:0>
        'AXIS_MAP_CONFIG'  , 0x41,... % Remapped Z axis value Remapped Y axis value Remapped X axis value
        'AXIS_MAP_SIGN'    , 0x42,... % RemappedX axis sign Remapped Y axis sign Remapped Z axis sign
        'ACC_OFFSET_X_LSB' , 0x55,... % Accelerometer Offset X <7:0>
        'ACC_OFFSET_X_MSB' , 0x56,... % Accelerometer Offset X <15:8>
        'ACC_OFFSET_Y_LSB' , 0x57,... % Accelerometer Offset Y <7:0>
        'ACC_OFFSET_Y_MSB' , 0x58,... % Accelerometer Offset Y <15:8>
        'ACC_OFFSET_Z_LSB' , 0x59,... % Accelerometer Offset Z <7:0>
        'ACC_OFFSET_Z_MSB' , 0x5A,... % Accelerometer Offset Z <15:8>
        'MAG_OFFSET_X_LSB' , 0x5B,... % Magnetometer Offset X <7:0>
        'MAG_OFFSET_X_MSB' , 0x5C,... % Magnetometer Offset X <15:8>
        'MAG_OFFSET_Y_LSB' , 0x5D,... % Magnetometer Offset Y <7:0>
        'MAG_OFFSET_Y_MSB' , 0x5E,... % Magnetometer Offset Y <15:8>
        'MAG_OFFSET_Z_LSB' , 0x5F,... % Magnetometer Offset Z <7:0>
        'MAG_OFFSET_Z_MSB' , 0x60,... % Magnetometer Offset Z <15:8>
        'GYR_OFFSET_X_LSB' , 0x61,... % Gyroscope Offset X <7:0>
        'GYR_OFFSET_X_MSB' , 0x62,... % Gyroscope Offset X <15:8>
        'GYR_OFFSET_Y_LSB' , 0x63,... % Gyroscope Offset Y <7:0>
        'GYR_OFFSET_Y_MSB' , 0x64,... % Gyroscope Offset Y <15:8>
        'GYR_OFFSET_Z_LSB' , 0x65,... % Gyroscope Offset Z <7:0>
        'GYR_OFFSET_Z_MSB' , 0x66,... % Gyroscope Offset Z <15:8>
        'ACC_RADIUS_LSB'   , 0x67,... % Accelerometer Radius
        'ACC_RADIUS_MSB'   , 0x68,... % Accelerometer Radius
        'MAG_RADIUS_LSB'   , 0x69,... % Magnetometer Radius LSB
        'MAG_RADIUS_MSB'   , 0x6A ... % Magnetometer Radius MSB
        )
        %PowerManagement = 'NormalMode' 'LowPowerMode' 'SuspendMode';
        %OperationMode = 'ConfigMode','NoneFusionMode', 'FusionMode', 'MagnetometerConfiguration'
        %SensorConfiguration 'DefaultSensorConfiguration' 'AccelerometerConfiguration' 'GyroscopeConfiguration'
    end
    

    
    properties (SetAccess = public)
     BNO055_ADDRESS = '28h';
     NI_Hardware = 'NI845x';
     i2cobj
     lastEuler
    end
    
        methods
            
        function obj = getCALIB_STAT(obj)
            %Power On Self Test ( Construct an instance of this class
            %   Detailed explanation goes here
            fwrite(obj.i2cobj,obj.Page_0.CALIB_STAT);  % System  Register
            readByte = fread (obj.i2cobj, 1);     % get 1 Byte
           
            binValue = dec2bin(readByte);
           
            
            MAG_Calib_Status =  bitget(readByte,1:2);
            
            if (sum(MAG_Calib_Status) == 2) % Beide Bits sind gestzt
            % Auswertung kann auch in der Klasse stehen
            end
            
            ACC_Calib_Status =  bitget(readByte,3:4);
            GYR_Calib_Status =  bitget(readByte,5:6);
            SYS_Calib_Status =  bitget(readByte,7:8);
            
            q=1;
            
                
        end
        
        function obj = POST(obj)
            %Power On Self Test ( Construct an instance of this class
            %   Detailed explanation goes here
            fwrite(obj.i2cobj,obj.Page_0.ST_RESULT);  % System  Register
            status = fread (obj.i2cobj, 1);     % get 1 Byte
            % 15 alles ok (Alle 4 Bits =1)
        end
        
    
        function obj = openI2C(obj)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            
            opengl hardware;
            instrreset;
            
            obj.i2cobj = i2c(obj.NI_Hardware, 0, obj.BNO055_ADDRESS);           %Objekt NI-Karte, Boardindex 0, BNO055_ADDRESS_A (0x28)
            fopen(obj.i2cobj);
            
        end
        
        function  status(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            fwrite(obj.i2cobj,obj.Page_0.SYS_STATUS);  % System Status Register
            status = fread (obj.i2cobj, 1);     % get 1 Byte
                                            % 0 System idle,
                                            % 1 System Error,
                                            % 2 Initializing peripherals
                                            % 3 System Initialization
                                            % 4 Executing selftest
                                            % 5 Sensor fusion algorithm running,
                                            % 6 System running without fusion algorithm
                                            
           

            if status ~= enumBNO055_STATUS.SystemIdle % Falls Status not 
                % excute ErrorHandler
                
            end
        end
        
        function ConfigMode(obj)
            if ~isempty(obj.i2cobj)
                fwrite(obj.i2cobj,[obj.Page_0.OPR_MODE , 12]);  % Configmodus (Register 3D) auf NDOF (12=1100) umschreiben
               % fwrite(obj.i2cobj,obj.OPR_MODE);         % Start-Register festlegen zum Auslesen
                config = fread (obj.i2cobj, 1);          % ein Byte ausgeben
            else
                % Bitte Bus starten
            end
        end
        
        function [x,y,z] = getEuler(obj)
            if ~isempty(obj.i2cobj)
       
                fwrite (obj.i2cobj,obj.Page_0.EUL_Heading_LSB); % Euler Register YAW (z)
                z = fread (obj.i2cobj, 2);               % LSB und MSB Register, je 8 bit
                z = bitshift(z(2),8)+z(1);
                z = dec2bin(z);
                z = typecast(uint16(bin2dec(z)),'int16');
                z = z/100;

                fwrite (obj.i2cobj,obj.Page_0.EUL_Roll_LSB, 'int8'); % Euler Register ROLL (x)
                x = fread (obj.i2cobj, 2);                    % LSB und MSB Register, je 8 bit
                x = bitshift(x(1),8)+x(2);
                x = dec2bin(x);
                x = typecast(uint16(bin2dec(x)),'int16');

                fwrite (obj.i2cobj,obj.Page_0.EUL_Pitch_LSB, 'int8'); % Euler Register Pitch (y)
                y = fread (obj.i2cobj, 2);                     % LSB und MSB Register, je 8 bit
                y = bitshift(y(1),8)+y(2);
                y = dec2bin(y);
                y = typecast(uint16(bin2dec(y)),'int16');
                
                obj.lastEuler=[x,y,z];
                
            else
                % Bitte Bus starten
            end
        end
        
    end
end

%% Arduino-Code 

% % #include <Wire.h>
% % #include <Adafruit_Sensor.h>
% % #include <Adafruit_BNO055.h>
% % #include <utility/imumaths.h>
% % 
% % /* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
% %    which provides a common 'type' for sensor data and some helper functions.
% % 
% %    To use this driver you will also need to download the Adafruit_Sensor
% %    library and include it in your libraries folder.
% % 
% %    You should also assign a unique ID to this sensor for use with
% %    the Adafruit Sensor API so that you can identify this particular
% %    sensor in any data logs, etc.  To assign a unique ID, simply
% %    provide an appropriate value in the constructor below (12345
% %    is used by default in this example).
% % 
% %    Connections
% %    ===========
% %    Connect SCL to SCL pin (analog 5 on Arduino UNO)
% %    Connect SDA to SDA pin (analog 4 on Arduino UNO)
% %    Connect VDD to 3-5V DC (depending on your board's logic level)
% %    Connect GROUND to common ground
% % 
% %    History
% %    =======
% %    2015/MAR/03  - First release (KTOWN)
% %    2015/AUG/27  - Added calibration and system status helpers
% % */
% % 
% % /* Set the delay between fresh samples */
% % #define BNO055_SAMPLERATE_DELAY_MS (25)
% % 
% % // Check I2C device address and correct line below (by default address is 0x29 or 0x28)
% % //                                   id, address
% % Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
% % 
% % /**************************************************************************/
% % /*
% %     Displays some basic information on this sensor from the unified
% %     sensor API sensor_t type (see Adafruit_Sensor for more information)
% % */
% % /**************************************************************************/
% % void displaySensorDetails(void)
% % {
% %   sensor_t sensor;
% %   bno.getSensor(&sensor);
% %   Serial.println("------------------------------------");
% %   Serial.print  ("Sensor:       "); Serial.println(sensor.name);
% %   Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
% %   Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
% %   Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
% %   Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
% %   Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
% %   Serial.println("------------------------------------");
% %   Serial.println("");
% %   delay(500);
% % }
% % 
% % /**************************************************************************/
% % /*
% %     Display some basic info about the sensor status
% % */
% % /**************************************************************************/
% % void displaySensorStatus(void)
% % {
% %   /* Get the system status values (mostly for debugging purposes) */
% %   uint8_t system_status, self_test_results, system_error;
% %   system_status = self_test_results = system_error = 0;
% %   bno.getSystemStatus(&system_status, &self_test_results, &system_error);
% % 
% %   /* Display the results in the Serial Monitor */
% %   Serial.println("");
% %   Serial.print("System Status: 0x");
% %   Serial.println(system_status, HEX);
% %   Serial.print("Self Test:     0x");
% %   Serial.println(self_test_results, HEX);
% %   Serial.print("System Error:  0x");
% %   Serial.println(system_error, HEX);
% %   Serial.println("");
% %   delay(500);
% % }
% % 
% % /**************************************************************************/
% % /*
% %     Display sensor calibration status
% % */
% % /**************************************************************************/
% % void displayCalStatus(void)
% % {
% %   /* Get the four calibration values (0..3) */
% %   /* Any sensor data reporting 0 should be ignored, */
% %   /* 3 means 'fully calibrated" */
% %   uint8_t system, gyro, accel, mag;
% %   system = gyro = accel = mag = 0;
% %   bno.getCalibration(&system, &gyro, &accel, &mag);
% % 
% %   /* The data should be ignored until the system calibration is > 0 */
% %   Serial.print("\t");
% %   if (!system)
% %   {
% %     Serial.print("! ");
% %   }
% % 
% %   /* Display the individual values */
% %   Serial.print("Sys:");
% %   Serial.print(system, DEC);
% %   Serial.print(" G:");
% %   Serial.print(gyro, DEC);
% %   Serial.print(" A:");
% %   Serial.print(accel, DEC);
% %   Serial.print(" M:");
% %   Serial.print(mag, DEC);
% % }
% % 
% % /**************************************************************************/
% % /*
% %     Arduino setup function (automatically called at startup)
% % */
% % /**************************************************************************/
% % void setup(void)
% % {
% %   Serial.begin(115200);
% %   Serial.println("Orientation Sensor Test"); Serial.println("");
% % 
% %   /* Initialise the sensor */
% %   if(!bno.begin())
% %   {
% %     /* There was a problem detecting the BNO055 ... check your connections */
% %     Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
% %     while(1);
% %   }
% % 
% %   delay(1000);
% % 
% %   /* Display some basic information on this sensor */
% %   displaySensorDetails();
% % 
% %   /* Optional: Display current status */
% %   displaySensorStatus();
% % 
% %   bno.setExtCrystalUse(true);
% % }
% % 
% % /**************************************************************************/
% % /*
% %     Arduino loop function, called once 'setup' is complete (your own code
% %     should go here)
% % */
% % /**************************************************************************/
% % void loop(void)
% % {
% %   /* Get a new sensor event */
% %   sensors_event_t event;
% %   bno.getEvent(&event);
% % 
% %   /* Display the floating point data */
% %   Serial.print("X: ");
% %   Serial.print(event.orientation.x, 4);
% %   Serial.print("\tY: ");
% %   Serial.print(event.orientation.y, 4);
% %   Serial.print("\tZ: ");
% %   Serial.print(event.orientation.z, 4);
% % 
% %   /* Optional: Display calibration status */
% %   displayCalStatus();
% % 
% %   /* Optional: Display sensor status (debug only) */
% %   //displaySensorStatus();
% % 
% %   /* New line for the next sample */
% %   Serial.println("");
% % 
% %   /* Wait the specified delay before requesting nex data */
% %   delay(BNO055_SAMPLERATE_DELAY_MS);
% % }

