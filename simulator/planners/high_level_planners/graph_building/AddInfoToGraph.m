%% Add Info to Already Made Graph
clear all;
close all;
clc;

%% Load Info

config_struct = load('MillionNodes.mat');
q_valid_list = config_struct.aaa;

%% Load Graph

graph_struct = load('Q_Graph_MilNodes_Mult3.mat');
Q_Graph = graph_struct.G;

%% Add Info to Graph

Q_Graph.Nodes.Configuration = q_valid_list';

%% Save New Graph

save('QGraph1mil_Config.mat','Q_Graph','q_valid_list')
