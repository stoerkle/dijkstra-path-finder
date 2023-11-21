# https://dev.to/ashwanirathee/dijkstras-algorithm-in-julia-46ih

# To find the vertex with
# minimum distance value, from the set of vertices
# not yet included in shortest path tree
function mindist(dist, sptset)
    min = Inf  # Initialize minimum distance for next node
    minindex = 0
    # Search smallest value nearest vertex not in the
    # shortest path tree
    for i in 1:size(graph)[1]
        if dist[i] < min && sptset[i] == false
            min = dist[i]
            minindex = i
        end
    end
    println("\nNext Node to be processed: ", minindex)
    return minindex
end

# Dijkstra's single source
# shortest path algorithm for a graph represented
# using adjacency matrix representation
function dijkstra(graph, tree_set, initial_node)
    println("Source Node:", initial_node)
    # println("Graph's Adjacency Matrix:\n", graph)
    # TreeSet = [false for i in 1:size(graph)[1]] # step 1
    TreeSet = tree_set
    dist = [Inf for i in 1:size(graph)[1]] # step 2
    dist[initial_node] = 0

    path = fill([], size(graph)[1])
    path[initial_node] = [initial_node]

    for i in 1:size(graph)[1]

        # Pick the minimum distance vertex from
        # the set of vertices not yet processed.
        x = mindist(dist, TreeSet) # step 3

        if checkbounds(Bool, graph, x)
            println("Before relaxation: ", dist)

            # step 3 -> relaxation procedure
            # Update dist value of the adjacent vertices
            # of the picked vertex only if the current
            # distance is greater than new distance and
            # the vertex in not in the shortest path tree
            for j in 1:size(graph)[1]
                if graph[x][j] > 0 && TreeSet[j] == false && dist[j] > dist[x] + graph[x][j]
                    dist[j] = dist[x] + graph[x][j]
                    path[j] = cat(path[x], j; dims=1)
                    @show path[j]
                    println("j: ", j, " | x: ", x)
                end
            end
            println("After relaxation: ", dist)

            # Put the minimum distance vertex in the
            # shortest path tree
            TreeSet[x] = true # step 4
        end
    end
    println("v | d[v] ")
    println("---------")
    for (i, j) in enumerate(dist)
        println(i, " | ", j)
    end

    return path
end

function convert_booleanmap2adjacencymatrix(map)
    
    h, w = size(map)
    
    graph = [zeros(w * h) for i in 1:(w*h)]

    tree_set = [true for i in 1:size(graph)[1]] # step 1

    for x in 1:h, y in 1:w
        if map[x, y]
            idx = (x - 1) * w + y
            tree_set[idx] = false

            if checkbounds(Bool, map, x - 1, y)
                if map[x-1, y]
                    graph[idx][((x-1-1)*w+y)] = 1
                end
            end

            if checkbounds(Bool, map, x + 1, y)
                if map[x+1, y]
                    graph[idx][((x-1+1)*w+y)] = 1
                end
            end

            if checkbounds(Bool, map, x, y - 1)
                if map[x, y-1]
                    graph[idx][((x-1)*w+y-1)] = 1
                end
            end

            if checkbounds(Bool, map, x, y + 1)
                if map[x, y+1]
                    graph[idx][((x-1)*w+y+1)] = 1
                end
            end


            if checkbounds(Bool, map, x + 1, y + 1)
                if map[x+1, y+1] && map[x+1, y] && map[x, y+1]
                    graph[idx][((x-1+1)*w+y+1)] = sqrt(2)
                end
            end

            if checkbounds(Bool, map, x + 1, y - 1)
                if map[x+1, y-1] && map[x+1, y] && map[x, y-1]
                    graph[idx][((x-1+1)*w+y-1)] = sqrt(2)
                end
            end

            if checkbounds(Bool, map, x - 1, y + 1)
                if map[x-1, y+1] && map[x-1, y] && map[x, y+1]
                    graph[idx][((x-1-1)*w+y+1)] = sqrt(2)
                end
            end

            if checkbounds(Bool, map, x - 1, y - 1)
                if map[x-1, y-1] && map[x-1, y] && map[x, y-1]
                    graph[idx][((x-1-1)*w+y-1)] = sqrt(2)
                end
            end
        end
    end
    return graph, tree_set
end

booleanmap = [
    true true true true true
    true true true true true
    true false false true true
    true true false true true
    true true true true true
    true true true true true
];

graph, TreeSet = convert_booleanmap2adjacencymatrix(booleanmap)

path = dijkstra(graph, TreeSet, 1)