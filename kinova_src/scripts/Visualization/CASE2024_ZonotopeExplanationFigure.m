%% WAITR CASE 2024 Presentation Zonotope Explanation Figure
% Zachary Brei

clear all;
close all;
clc;

%% Individual Zonotopes

% Zonotope 1
c1 = [0;0];
g1 = [1;0];
zono1 = zonotope(c1,g1);
% Zonotope 2
c2 = c1; 
g2 = [0;1];
zono2 = zonotope(c2,g2);
% Zonotope 3
c3 = c1; 
g3 = [0.5;0.5];
zono3 = zonotope(c3,g3);

%% Combining Zonotopes

% Zonotope 1 + Zonotope 2
zono12 = zono1 + zono2;
% Zonotope 1+2 + Zonotope 3
zono123 = zono12 + zono3;

%% Plotting

figure(1)
hold on
plot(zono123,[1,2],'Filled',true,'LineWidth',2,'FaceColor','r')
plot(zono1,[1,2],'Filled',true,'LineWidth',2)

xlim([-2,2])
ylim([-2,2])
axis square