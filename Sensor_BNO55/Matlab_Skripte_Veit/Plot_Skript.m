
%% Figure für Euler Winkel erstellen

if exist('data_e') == 1
figure(1);
sgtitle('Euler Winkel der Paddelbewegung');
hold all

sp1 = subplot(3,1,1);
sp2 = subplot(3,1,2);
sp3 = subplot(3,1,3);

%% Bereitgestellte Daten aufteilen

t=data_e(:,1);
x=data_e(:,2);
y=data_e(:,3);
z=data_e(:,4);

%% Plots zeichnen

plot(sp1, t, x, 'r');
hold on;
grid (sp1,'on');
ylim(sp1,[-180 180]);
datetick(sp1, 'x', 'HH:MM:SS','keeplimits')
ylabel(sp1,'Roll - X in °');


plot(sp2, t, y, 'g');
hold on;
grid (sp2,'on');
ylim(sp2,[-180 180]);
datetick(sp2, 'x', 'HH:MM:SS','keeplimits')
ylabel(sp2,'Pitch - Y in °');

plot(sp3, t, z, 'b');
hold on;
grid (sp3,'on');
ylim(sp3,[0 360]);
datetick(sp3, 'x', 'HH:MM:SS','keeplimits')
ylabel(sp3,'Yaw - Z in °');
xlabel(sp3,'Zeit');

end


%% Figure für Beschleunigung erstellen

if exist('data_a') == 1
figure(4);
sgtitle('Beschleunigung der Paddelbewegung');
hold all

sp4 = subplot(3,1,1);
sp5 = subplot(3,1,2);
sp6 = subplot(3,1,3);

%% Bereitgestellte Daten aufteilen

t=data_a(:,1);
x=data_a(:,2);
y=data_a(:,3);
z=data_a(:,4);

%% Plots zeichnen

plot(sp4, t, x, 'r');
hold on;
grid (sp4,'on');
ylim(sp4,[-15 15]);
datetick(sp4, 'x', 'HH:MM:SS','keeplimits')
ylabel(sp4,'x in m/s²');

plot(sp5, t, y, 'g');
hold on;
grid (sp5,'on');
ylim(sp5,[-15 15]);
datetick(sp5, 'x', 'HH:MM:SS','keeplimits')
ylabel(sp5,'y in m/s²');

plot(sp6, t, z, 'b');
hold on;
grid (sp6,'on');
ylim(sp6,[-30 30]);
datetick(sp6,'x', 'HH:MM:SS','keeplimits')
ylabel(sp6,'z in m/s²');
xlabel('Zeit');
end


%% Figure für Quaternionen erstellen

if exist('data_q') == 1
figure(3);
sgtitle('Quaternionen der Paddelbewegung');
hold all

sp7 = subplot(2,2,1);
sp8 = subplot(2,2,2);
sp9 = subplot(2,2,3);
sp10 = subplot(2,2,4);

%% Bereitgestellte Daten aufteilen

t=data_q(:,1);
w=data_q(:,2);
x=data_q(:,3);
y=data_q(:,4);
z=data_q(:,5);

%% Plots zeichnen

plot(sp7, t, w, 'black');
hold on;
grid (sp7,'on');
ylim(sp7,[-1 1]);
datetick(sp7, 'x', 'HH:MM:SS','keeplimits');
ylabel(sp7,'w - Realteil');
xlabel(sp7,'Zeit');


plot(sp8, t, x, 'r');
hold on;
grid (sp8,'on');
ylim(sp8,[-1 1]);
datetick(sp8, 'x', 'HH:MM:SS','keeplimits');
ylabel(sp8,'x - Imaginärteil');
xlabel(sp8,'Zeit');

plot(sp9, t, y, 'g');
hold on;
grid (sp9,'on');
ylim(sp9,[-1 1]);
datetick(sp9, 'x', 'HH:MM:SS','keeplimits');
ylabel(sp9,'y - Imaginärteil');
xlabel(sp9,'Zeit');

plot(sp10, t, z, 'b');
hold on;
grid (sp10,'on');
ylim(sp10,[-1 1]);
datetick(sp10, 'x', 'HH:MM:SS','keeplimits');
ylabel(sp10,'z - Imaginärteil');
xlabel(sp10,'Zeit');
end