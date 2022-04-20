close all;
clear all;

% read wake data
mat = csvread("2m.csv")

x = mat(:,1)
y = mat(:,2)
z = mat(:,3)


set(0,'defaultfigurecolor','w');
c = z;
fig = scatter3(x,y,z,50,c,'.')

set(gca,'FontSize',16);

% view point

%view([0,0,1]);
%view([0,-1,0]);
%view([0.9,-0.5,1.2]);

% save figures
%saveas(fig,"Figures\4m\V2\front.jpg");
