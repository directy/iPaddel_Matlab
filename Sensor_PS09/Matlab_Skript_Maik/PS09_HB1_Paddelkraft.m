% Hauptprogramm zur Krafterfassung mit Halbbrückenschaltung %
% In Anlehnung Projektarbeit Jan Töbe %
%% Alle Variablen zurücksetzen %% 
clear all;                          % Leere Workspace 
instrreset;                         % Verbindungen schließen und lösche 
                                    % alle Objekte  
clc;                                % Leere Eingabe Fenster
close(findall(0,'type','figure'));  % Schließe alle Plots

%% Variablen Deklaration %% 
profile on                          % Ausführungszeit, Speicherverbrauch 
                                    % aufzeichnen - Software-Profiling
i2cobj = i2c('NI845x', 0, '60h');   % Erstelle I²C-Objekt Vendor 'NI845x',
                                    % BoardIndex 0, Remote-Adresse 60h     
fopen(i2cobj);						% Öffne Verbindung USB-845x adapter.
meas = 1;
rawData = [];					
j = 1;					           
jj = 1;
ii= 1;
counter = 10;
catchcounter =0;
time = 0;
langzeit = [];
csvwrite('Paddel_Kraft.csv',langzeit);	% Erstelle Matrix langzeit für
                                        % Paddel_Kraft.csv
Config_PS09(i2cobj);               % Starte Funktion  

figure(1);                         % Erstelle Diagramm
 grid on
 hold on			
 title('Kraftmessung am Paddelprüfstand');
 xlabel('Zeit t [s]');
 ylabel('Kraft F [N]');

 %% Messschleife %%
while (meas == 1)
    tic;				   % Zeiterfassung        
    pause(0.01);           % Pause zwischen Messwerterfassung
    try                    % Anweisung loggt Fehler und überspingt 
                           % Standartfehleranweisung u. springt zu catch
                
fwrite(i2cobj,[hex2dec('40'),hex2dec('14')]);   % Schreibe auf BUS, dass  
                                                % das HBO Register gelesen
                                                % werden soll
temp=fread (i2cobj, 3);                         % Schreibe gelesene 3 Byte
                                                % Daten von PS09 auf temp                             	
rawData = [rawData,temp];                       % Ausgabe in Zeilenvektor 
     
 b=bitget(temp(1),8,'uint8');                   % Überprüfe Vorzeichen
    if (b==1)
     sgnum=255;
    else
     sgnum=0;
    end    

value_int = int32(sgnum);           %sgnum als integer 32bit Zahl speichern 

%% Erstelle Variable aus den Datenbytes %%   
    for i = 1:3                                     
        value_int = bitshift(value_int,8);          
        value_int = value_int + temp(i);	
    end

value(j) = value_int;               % Schreibe Werte nacheinander in value				

%% Offsetbildung aus 10 Messwerten %%
    if j == 10
        offset = mean(value(1,1:10));                     
    end                                                 

    if j > 10 && offset < 0
        value(1,j) = value(1,j) + abs(offset); 
    elseif j > 10 && offset >=0
        value(1,j) = value(1,j) - offset;
    end

%% Mittelwertbildung und Kalibrierung %%

    if j > 10 && j == counter + 10
     counter = j;
     value_mean = mean(value(1,j-10:j));
    
	% Kalibrierung %
      value_true =  value_mean/(2185.21916);% Eingabe Kalibrierfaktor 
                                            % x=value_mean/(m*g)
      plot(time, value_true(jj),'b*');      % Erzeuge dynamischen Plotpunkt
                                            % in Diagramm Figure 1 
      langzeit(ii,1) = value_true;          % Schreibe Messergebnis in 
      langzeit(ii,3) = time;                % Matrix langzeit inkl. Zeit
     
        ii = ii +1;
    end
    
%% Schreiben von Paddelkraft.csv %% 
    
    if size(rawData,2)>200         % Schreibe wenn 200 Messwerte vorhanden	
      dlmwrite('Paddel_Kraft.csv', langzeit, '-append');% Anfügen der 
                                                        % Folgewerte    
      ii= 1;
      rawData = [];                % Leere Varaible rawData
      langzeit = [];               % Leere Varaible langzeit      
    end
    
    j = j + 1;                     % Erhöhe Zähler um 1

%% Bei Fehler: Fehlermeldung, Neukonfiguration und Neustart %%
    catch
        warning('Error using i2c/fwrite. Skip current loop run.')
        msg = ['executes I2C communication reset'];
        catchcounter = catchcounter +1;
        j = j + 1;
        write_config_PS09(i2cobj)   % Schreibe in PS09-Register
        
    end
    time = time + toc;              % Addiere benötigte Zeit
end
fclose(i2cobj);                     % Schließe I²C Verbindung
delete(i2cobj);                     % Lösche I²C Objekt
profile off

%% Übergeordnete Funktion für Fehlerbehandlung und Registerkonfiguration %%
function Config_PS09(i2cobj)
    error = 1;
    pause(1);
    while (error == 1) % Schleife für erstmaliges Beschreiben der Register
                       % und Wiederholung im Fehlerfahl 
        write_config_PS09(i2cobj);
        error = control_config_PS09(i2cobj);
        disp('Fehler bei PS09-Konfiguration'); %Anzeige im Command Window
    end
    pause(1);
end

%% Konfiguration des PS09-Registers %%
function write_config_PS09(i2cobj)
    power_reset = hex2dec('F0');  % Registerreset durchführen 
    watchdog_off = hex2dec('9E'); % Ausgeschalten, da Reset im Fehlerfall
    c_reg0 = [ hex2dec('00'), hex2dec('30'), hex2dec('23'), hex2dec('00'), hex2dec('D2')];
    c_reg1 = [ hex2dec('00'), hex2dec('31'), hex2dec('2C'), hex2dec('45'), hex2dec('C0')]; 
    c_reg2 = [ hex2dec('00'), hex2dec('32'), hex2dec('55'), hex2dec('44'), hex2dec('72')];  
    c_reg3 = [ hex2dec('00'), hex2dec('33'), hex2dec('82'), hex2dec('02'), hex2dec('78')]; 
    c_reg4 = [ hex2dec('00'), hex2dec('34'), hex2dec('40'), hex2dec('00'), hex2dec('00')]; 
    c_reg5 = [ hex2dec('00'), hex2dec('35'), hex2dec('40'), hex2dec('00'), hex2dec('00')]; 
    c_reg6 = [ hex2dec('00'), hex2dec('36'), hex2dec('40'), hex2dec('00'), hex2dec('00')]; 
    c_reg7 = [ hex2dec('00'), hex2dec('37'), hex2dec('40'), hex2dec('00'), hex2dec('00')]; 
    c_reg8 = [ hex2dec('00'), hex2dec('38'), hex2dec('00'), hex2dec('00'), hex2dec('00')]; 
    c_reg9 = [ hex2dec('00'), hex2dec('39'), hex2dec('10'), hex2dec('00'), hex2dec('00')]; 
    c_reg10 = [ hex2dec('00'), hex2dec('3A'), hex2dec('14'), hex2dec('FB'), hex2dec('A4')];
    c_reg11 = [ hex2dec('00'), hex2dec('3B'), hex2dec('00'), hex2dec('00'), hex2dec('3F')];
    c_reg12 = [ hex2dec('00'), hex2dec('3C'), hex2dec('35'), hex2dec('12'), hex2dec('00')];
    c_reg13 = [ hex2dec('00'), hex2dec('3D'), hex2dec('0C'), hex2dec('05'), hex2dec('40')];
    c_reg14 = [ hex2dec('00'), hex2dec('3E'), hex2dec('60'), hex2dec('A1'), hex2dec('33')];
    c_reg15 = [ hex2dec('00'), hex2dec('3F'), hex2dec('80'), hex2dec('64'), hex2dec('01')];
    init_reset = hex2dec('C0');
    start_newcycle = hex2dec('CC');
    
% Schreibe Register %
    fwrite(i2cobj, power_reset);
    fwrite(i2cobj, watchdog_off);
    fwrite(i2cobj, c_reg0);
    fwrite(i2cobj, c_reg1);
    fwrite(i2cobj, c_reg2);
    fwrite(i2cobj, c_reg3);
    fwrite(i2cobj, c_reg4);
    fwrite(i2cobj, c_reg5);
    fwrite(i2cobj, c_reg6);
    fwrite(i2cobj, c_reg7);
    fwrite(i2cobj, c_reg8);
    fwrite(i2cobj, c_reg9);
    fwrite(i2cobj, c_reg10);
    fwrite(i2cobj, c_reg11);
    fwrite(i2cobj, c_reg12);
    fwrite(i2cobj, c_reg13);
    fwrite(i2cobj, c_reg14);
    fwrite(i2cobj, c_reg15);
    fwrite(i2cobj, init_reset);
    fwrite(i2cobj, start_newcycle);

end
%%Lese und Vergleiche Registerinhalt mit Soll-Inhalt%%
function err = control_config_PS09(i2cobj)

    c_reg0 = [hex2dec('23'); hex2dec('00'); hex2dec('D2')]; 
    c_reg1 = [hex2dec('2C'); hex2dec('45'); hex2dec('C0')]; 
    c_reg2 = [hex2dec('55'); hex2dec('44'); hex2dec('72')];
    c_reg3 = [hex2dec('82'); hex2dec('02'); hex2dec('78')]; 
    c_reg4 = [hex2dec('40'); hex2dec('00'); hex2dec('00')];
    c_reg5 = [hex2dec('40'); hex2dec('00'); hex2dec('00')];
    c_reg6 = [hex2dec('40'); hex2dec('00'); hex2dec('00')];
    c_reg7 = [hex2dec('40'); hex2dec('00'); hex2dec('00')];
    c_reg8 = [hex2dec('00'); hex2dec('00'); hex2dec('00')];
    c_reg9 = [hex2dec('10'); hex2dec('00'); hex2dec('00')];
    c_reg10 = [hex2dec('14'); hex2dec('FB'); hex2dec('A4')];
    c_reg11 = [hex2dec('00'); hex2dec('00'); hex2dec('3F')];
    c_reg12 = [hex2dec('35'); hex2dec('12'); hex2dec('00')];
    c_reg13 = [hex2dec('0C'); hex2dec('05'); hex2dec('40')];
    c_reg14 = [hex2dec('60'); hex2dec('A1'); hex2dec('33')];
    c_reg15 = [hex2dec('80'); hex2dec('64'); hex2dec('01')];
    
%% Einlesen und Vergleichen des Config-Registers %%
    % Control of Config-Register 0
    fwrite(i2cobj,[hex2dec('40'), hex2dec('30')]);
    temp=fread (i2cobj, 3);
     disp(temp);
    if temp == c_reg0
        err0 = 0;
    else
        err0 = 1;
    end
 
    % Control of Config-Register 1
    fwrite(i2cobj,[hex2dec('40'), hex2dec('31')]);
    temp=fread (i2cobj, 3);
    if temp == c_reg1
        err1 = 0;
    else
        err1 = 1;
    end
    
    % Control of Config-Register 2
    fwrite(i2cobj,[hex2dec('40'), hex2dec('32')]);
    temp=fread (i2cobj, 3);
    if temp == c_reg2
        err2 = 0;
    else
        err2 = 1;
    end
    
    % Control of Config-Register 3
    fwrite(i2cobj,[hex2dec('40'), hex2dec('33')]);
    temp=fread (i2cobj, 3);
    if temp == c_reg3
        err3 = 0;
    else
        err3 = 1;
    end
    
    % Control of Config-Register 4
    fwrite(i2cobj,[hex2dec('40'), hex2dec('34')]);
    temp=fread (i2cobj, 3);
    if temp == c_reg4
        err4 = 0;
    else
        err4 = 1;
    end
    
    % Control of Config-Register 5
    fwrite(i2cobj,[hex2dec('40'), hex2dec('35')]);
    temp=fread (i2cobj, 3);
    if temp == c_reg5
        err5 = 0;
    else
        err5 = 1;
    end
    
    % Control of Config-Register 6
    fwrite(i2cobj,[hex2dec('40'), hex2dec('36')]);
    temp=fread (i2cobj, 3);
    if temp == c_reg6
        err6 = 0;
    else
        err6 = 1;
    end
    
    % Control of Config-Register 7
    fwrite(i2cobj,[hex2dec('40'), hex2dec('37')]);
    temp=fread (i2cobj, 3);
    if temp == c_reg7
        err7 = 0;
    else
        err7 = 1;
    end
    
    % Control of Config-Register 8
    fwrite(i2cobj,[hex2dec('40'), hex2dec('38')]);
    temp=fread (i2cobj, 3);
    if temp == c_reg8
        err8 = 0;
    else
        err8 = 1;
    end
    
    % Control of Config-Register 9
    fwrite(i2cobj,[hex2dec('40'), hex2dec('39')]);
    temp=fread (i2cobj, 3);
    if temp == c_reg9
        err9 = 0;
    else
        err9 = 1;
    end
    
    % Control of Config-Register 10
    fwrite(i2cobj,[hex2dec('40'), hex2dec('3A')]);
    temp=fread (i2cobj, 3);
    if temp == c_reg10
        err10 = 0;
    else
        err10 = 1;
    end
    
    % Control of Config-Register 11
    fwrite(i2cobj,[hex2dec('40'), hex2dec('3B')]);
    temp=fread (i2cobj, 3);
    if temp == c_reg11
        err11 = 0;
    else
        err11 = 1;
    end
    
    % Control of Config-Register 12
    fwrite(i2cobj,[hex2dec('40'), hex2dec('3C')]);
    temp=fread (i2cobj, 3);
    if temp == c_reg12
        err12 = 0;
    else
        err12 = 1;
    end
    
    % Control of Config-Register 13
    fwrite(i2cobj,[hex2dec('40'), hex2dec('3D')]);
    temp=fread (i2cobj, 3);
    if temp == c_reg13
        err13 = 0;
    else
        err13 = 1;
    end
    
    % Control of Config-Register 14
    fwrite(i2cobj,[hex2dec('40'), hex2dec('3E')]);
    temp=fread (i2cobj, 3);
    if temp == c_reg14
        err14 = 0;
    else
        err14 = 1;
    end
    
    % Control of Config-Register 15
    fwrite(i2cobj,[hex2dec('40'), hex2dec('3F')]);
    temp=fread (i2cobj, 3);
    if temp == c_reg15
        err15 = 0;
    else
        err15 = 1;
    end
  
    if err0==1 || err1==1 || err2==1 || err3==1 || err4==1 || err5==1 || err6==1 || err7==1 || err8==1 || err9==1 || err10==1 || err11==1 || err12==1 || err13==1 || err14==1 || err15==1
        err = 1;
    else
        err = 0;
    end    
end