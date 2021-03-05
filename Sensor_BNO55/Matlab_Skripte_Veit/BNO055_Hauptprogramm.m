%% Kombination der Unterprogramme zur Messung von Euler Winkel, Quaternionen und Beschleunigung am BNO055

%% Pre definition

clc;
close all;
clear all;
opengl hardware;
instrreset;

i2cobj = i2c('NI845x', 0, '28h');           % Objekt NI-Karte, Boardindex 0, BNO055_ADDRESS_A (0x28)
fopen(i2cobj);

%% Error Check

fwrite(i2cobj,[hex2dec('39')]);             % System Status Register
status=fread (i2cobj, 1);                   % 0 System idle,
                                            % 1 System Error,
                                            % 2 Initializing peripherals
                                            % 3 System Initialization
                                            % 4 Executing selftest
                                            % 5 Sensor fusion algorithm running,
                                            % 6 System running without fusion algorithm

if status == 1
   fwrite(i2cobj,[hex2dec('3A')]);
   errorcode = fread (i2cobj, 1);
   disp('System Error');
   
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
       case 10
           disp('Sensor configuration error')
   end
end


%% Configmode festlegen

fwrite(i2cobj,[hex2dec('3D'), 8]);          % Configmodus (Register '3D') auf NDOF (12=1100) 
                                            % oder IMU (8=1000) umschreiben
fwrite(i2cobj,[hex2dec('3D')]);             % start Register festlegen zum Auslesen
config = fread (i2cobj, 1);                   % Check ob Config übernommen wurde

%% Start
t=0;
startTime = datetime('now');

%% Schleife

%Datenarray bereitstellen
data_e = zeros(1,4,'double');                 %Euler
data_a = zeros(1,4,'double');                 %Beschleunigung
data_q = zeros(1,5,'double');                 %Quaternion

i=1;
% pre_az=0;
t_summ = 0;
cycletime = 0;
while(i<10000)                               %1000 Messwerte aufnehmen
    tic
    %get Data from Sensor
    fwrite (i2cobj,[hex2dec('1A')]);        % Euler Register Addresse (z)
    regvalue = fread (i2cobj, 3, 'int16');  % 3 mal 16bit auslesen für Yaw (z), Pitch (x), Roll (y)
    eulerangles=double(regvalue)/16;        % 1 Grad = 16 LSB ->  Vektor 'regvalue' durch 16 teilen
    
    fwrite (i2cobj,[hex2dec('28')]);        % linear Accel Register Addresse (x)
    regvalue = fread (i2cobj, 3, 'int16');  % 3 mal 16bit auslesen für x-y-z
    linaccel=double(regvalue)/100;          % 1 m/s^2 = 100 LSB ->  Vektor 'regvalue' durch 100 teilen
    
    fwrite (i2cobj,[hex2dec('20')]);        % Quaternion Register Addresse (w)
    regvalue = fread (i2cobj, 4, 'int16');  % 4 mal 16bit auslesen für w-x-y-z
    quat = double(regvalue)/...
        (bitshift(1,14));                   % 1 = 2^14 LSB ->  Vektor 'regvalue' durch 2^14 teilen

    
    % Vektorwerte zuweisen
    ex = eulerangles(3);
    ey = eulerangles(2);
    ez = eulerangles(1);
    
    ax = linaccel(1);
    ay = linaccel(2);
    az = linaccel(3);
    
    qw = quat(1);
    qx = quat(2);
    qy = quat(3);
    qz = quat(4);
    
    % Get current time
    t =  datetime('now') - startTime;
    
    % reorg array to save data
    t=datenum(t);
    e = [t ex ey ez];
    data_e = [data_e; e];
    
    a = [t ax ay az];
    data_a = [data_a; a];
    
    q = [t qw qx qy qz];
    data_q = [data_q; q];
    
    cycletime = toc;
    t_summ = t_summ + cycletime ;
    disp(t_summ);
    
    i=i+1;
%     pre_az=az;
end