
%% Pre definition

clc;
close all;
clearvars -except data_e data_a;
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
sgtitle('Quaternionen der Paddelbewegung - q=w+xi+yj+zk');
hold all

sp1 = subplot(2,2,1);
grid on;
title(sp1,'Realteil - w');
xlabel('Zeit');
ylabel('w');
linesp1 = animatedline;

sp2 = subplot(2,2,2);
grid on;
title(sp2,'1. Imaginärteil - x');
xlabel('Zeit');
ylabel('x');
linesp2 = animatedline('Color' , 'r');

sp3 = subplot(2,2,3);
grid on
title(sp3,'1. Imaginärteil - y');
xlabel('Zeit');
ylabel('y');
linesp3 = animatedline('Color' , 'g');

sp4 = subplot(2,2,4);
grid on;
title(sp4,'1. Imaginärteil - z');
xlabel('Zeit');
ylabel('z');
linesp4 = animatedline('Color' , 'b');

tp = theaterPlot('XLimit',[-2 2],'YLimit',[-2 2],'ZLimit',[-2 2]);
op = orientationPlotter(tp,'DisplayName','Stabmittelpunkt','LocalAxesLength',1);

%% Start
t=0;
startTime = datetime('now');

%% Schleife

data_q=zeros(1,5,'double');

while(1)
    fwrite (i2cobj,[hex2dec('20')]);        % Euler Register Addresse (z)
    regvalue = fread (i2cobj, 4, 'int16');  % 4 mal 16bit auslesen für w-x-y-z
    quat = double(regvalue)/...
        (bitshift(1,14));                   % 1 = 2^14 LSB ->  Vektor 'regvalue' durch 2^14 teilen
 
    
    %Vektorwerte zuweisen; 
    
    w = quat(1);
    x = quat(2);
    y = quat(3);
    z = quat(4);

    % Get current time
    t =  datetime('now') - startTime;

    % Graphen aktualisieren
    addpoints(linesp1,datenum(t),w);
    axis (sp1,[datenum([0 t -1 1])]);
    addpoints(linesp2,datenum(t),x);
    axis (sp2,[datenum([0 t -1 1])]);       % evtl bestimmtes Zeitfenster einrichten x-Koord: t-seconds(15) bis t
    addpoints(linesp3,datenum(t),y);
    axis (sp3,[datenum([0 t -1 1])]);
    addpoints(linesp4,datenum(t),z);
    axis (sp4,[datenum([0 t -1 1])]);

    %Zeit-Achse updaten
    datetick(sp1,'x','keeplimits')
    datetick(sp2,'x','keeplimits')
    datetick(sp3,'x','keeplimits')
    datetick(sp4,'x','keeplimits')
    drawnow;
   
    % Umrechnung in Eulerwinkel für Orientationplot
    q = [w x y z];                          %Umformung von vektor in 4x1 Array
    q1=q;
    eul = quat2eul(q, 'XYZ');
    eul = eul * 180/pi;                     %Umrechnung rad in Grad
    x=-eul(1);
    y=eul(2);
    z=-eul(3);
    
    %Orientationplot updaten
    plotOrientation(op,x,y,z);
    drawnow;
    
    %reorg array save data
    t=datenum(t);
    q =[t q1];
    data_q=[data_q; q];
    
end