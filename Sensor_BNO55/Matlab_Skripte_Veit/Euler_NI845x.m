
%% Pre definition

clc;
close all;
clearvars -except data_a data_q;
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
config=fread (i2cobj, 1);                   % Check ob Config übernommen wurde

%% Plot definieren

figure(1);
sgtitle('Euler Winkel');
hold all

sp1 = subplot(3,1,1);
grid on;
title(sp1,'Roll - Rotation um X-Achse');
xlabel('Zeit');
ylabel('Grad');
linesp1 = animatedline('Color' , 'r');

sp2 = subplot(3,1,2);
grid on
title(sp2,'Pitch - Rotation um Y-Achse');
xlabel('Zeit');
ylabel('Grad');
linesp2 = animatedline('Color' , 'g');

sp3 = subplot(3,1,3);
grid on;
title(sp3,'Yaw - Rotation um Z-Achse');
xlabel('Zeit');
ylabel('Grad');
linesp3 = animatedline('Color' , 'b');

tp = theaterPlot('XLimit',[-2 2],'YLimit',[-2 2],'ZLimit',[-2 2]);
op = orientationPlotter(tp,'DisplayName','Stabmittelpunkt','LocalAxesLength',1);

%% Start
t=0;
startTime = datetime('now');

%% Schleife

data_e=zeros(1,4,'double');

while(1)
    fwrite (i2cobj,[hex2dec('1A')]);        % Euler Register Addresse (z)
    regvalue = fread (i2cobj, 3, 'int16');  % 3 mal 16bit auslesen für Yaw (z), Pitch (x), Roll (y)
    eulerangles=double(regvalue)/16;        % 1 Grad = 16 LSB ->  Vektor 'regvalue' durch 16 teilen
 
    
    %Vektorwerte zuweisen; laut Adafruit Datasheet sind die Richtungen
    %invertiert (CW = [-], CCW = [+]
    y = eulerangles(2);
    x = eulerangles(3);
    z = eulerangles(1);

    % Get current time
    t =  datetime('now') - startTime;

    % Graphen aktualisieren
    addpoints(linesp1,datenum(t),x);
    axis (sp1,[datenum([0 t -180 180])]);      % evtl bestimmtes Zeitfenster einrichten x-Koord: t-seconds(15) bis t
    addpoints(linesp2,datenum(t),y);
    axis (sp2,[datenum([0 t -180 180])]);
    addpoints(linesp3,datenum(t),z);
    axis (sp3,[datenum([0 t 0 360])]);

    %Zeit-Achse updaten
    datetick(sp1,'x','keeplimits')
    datetick(sp2,'x','keeplimits')
    datetick(sp3,'x','keeplimits')
    drawnow;
   
    %Orientationplot updaten
    plotOrientation(op,x,y,z);
    drawnow;
    
    %reorg array save data
    t=datenum(t);
    e=[t x y z];
    data_e=[data_e; e];    
end