%% Import Adjacency Matrix from Text File and Save to Matlab Graph

%% Import
% read the text file
AA = readmatrix('adj_matrix_mil_mult3.txt');

%% Processing

% preallocate sparse representation of adjacency matrix
adja = sparse(1e6,1e6);
% assemble the adjacency matrix
% Note that C++ index starts at 0, but MATLAB starts at one, so shift by
% one to assemble the matrix?
for i = 1:size(AA,1)
    adja(AA(i,1)+1, AA(i,2)+1) = AA(i,3);
end

% AA = readmatrix('adj_matrix_mil_mult3.txt');
% adja = sparse(1e6,1e6);
% adja(AA(:,1)+1, AA(:,2)+1) = AA(:,3);
% G = graph(adja, 'lower');

%% Building
% construct the graph
G = graph(adja, 'lower');

%% Checking Connectedness

[bin, binsize] = conncomp(G);
max(binsize)