using Plots

include("dijkstra.jl")
using .Dijkstra

graph = [
    [0, 2, 4, 0, 0, 0],
    [0, 0, 1, 7, 0, 0],
    [0, 0, 0, 0, 3, 0],
    [0, 0, 0, 0, 0, 1],
    [0, 0, 0, 2, 0, 5],
    [0, 0, 0, 0, 0, 0],
];

idx_start_node = 1

TreeSet = [false for i in 1:size(graph)[1]]

path = compute_shortes_path(graph, TreeSet, idx_start_node)
