using Plots

include("dijkstra.jl")
using .Dijkstra

graph = [
    [0, 5, 2, 0, 0],
    [5, 0, 1, 4, 0],
    [2, 1, 0, 3, 6],
    [0, 1, 3, 0, 2],
    [0, 0, 6, 2, 0],
];

idx_start_node = 5

TreeSet = [false for i in 1:size(graph)[1]]

path = compute_shortest_path(graph, TreeSet, idx_start_node)
