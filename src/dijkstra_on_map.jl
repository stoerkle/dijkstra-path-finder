using Plots

include("dijkstra.jl")
using .Dijkstra

booleanmap = [
    true true true true true
    true true true true true
    true false false true true
    true true false true true
    true true true true true
    true true true true true
];

idx_start_node = 1
idx_end_node = 25

h, w = size(booleanmap)

graph, TreeSet = convert_booleanmap2adjacencymatrix(booleanmap)

path = compute_shortest_path(graph, TreeSet, idx_start_node)

plot_map, plot_obs = convert_booleanmap2plotmap(booleanmap)

fig = plot(
    plot_map,
    color=:lightgrey,
    xlim = (0, w),
    ylims = (-h, 0),
    linecolor = :grey,
    opacity=.5,
    aspect_ratio=:equal,
    legend = :none);

plot!(fig, 
    plot_obs,
    color = :grey,
    linecolor = :grey,
    opacity = 0.5);

path_points = get_coordindates_on_map.(path[idx_end_node], h, w)
plot!(fig,
    path_points);

annotate!(fig,
    get_coordindates_on_map(idx_start_node, h, w)[1],
    get_coordindates_on_map(idx_start_node, h, w)[2],
    text("S"));

annotate!(fig,
    get_coordindates_on_map(idx_end_node, h, w)[1],
    get_coordindates_on_map(idx_end_node, h, w)[2],
    text("E"))
