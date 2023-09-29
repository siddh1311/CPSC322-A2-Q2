import java.util.*;

public class Main {
    public static Graph createGraph() {
        Graph graph = new Graph();
        Vertex s = new Vertex("S", 24);
        Vertex a = new Vertex("A", 21);
        Vertex b = new Vertex("B", 19);
        Vertex c = new Vertex("C", 19);
        Vertex d = new Vertex("D", 9);
        Vertex e = new Vertex("E", 11);
        Vertex f = new Vertex("F", 12);
        Vertex g = new Vertex("G", 4);
        Vertex h = new Vertex("H", 6);
        Vertex z = new Vertex("Z", 0);
        graph.addVertex(s);
        graph.addVertex(a);
        graph.addVertex(b);
        graph.addVertex(c);
        graph.addVertex(d);
        graph.addVertex(e);
        graph.addVertex(f);
        graph.addVertex(g);
        graph.addVertex(h);
        graph.addVertex(z);
        graph.addEdge(s, a, 3);
        graph.addEdge(s, b, 9);
        graph.addEdge(s, c, 4);
        graph.addEdge(a, c, 2);
        graph.addEdge(b, c, 13);
        graph.addEdge(c, d, 5);
        graph.addEdge(c, e, 4);
        graph.addEdge(c, f, 8);
        graph.addEdge(d, f, 5);
        graph.addEdge(e, f, 7);
        graph.addEdge(f, g, 8);
        graph.addEdge(f, h, 7);
        graph.addEdge(f, z, 18);
        graph.addEdge(g, z, 9);
        graph.addEdge(h, z, 6);
        return graph;
    }
    public static void main(String[] args) {
        Graph graph = createGraph();
        Vertex startNode = graph.getVertex("S");
        Vertex goal = graph.getVertex("Z");
        performDFSSearch(graph, startNode, goal);
        performBFSSearch(graph, startNode, goal);
        performLCFSSearch(graph, startNode, goal);
        performIDSSearch(graph, startNode, goal);
        performBestFSSearch(graph, startNode, goal);
        performAStarSearch(graph, startNode, goal);
        performBAndBSearch(graph, startNode, goal);
    }

    public static PathNode search(Graph graph, Vertex start, Vertex goal, Frontier<PathNode> fr,
                                  int limit, FuncInterface fobj) {
        // Create a hybrid Frontier data structure. It can act as a stack, queue, or priority queue
        // based on initialization from the function calling the search method.
        Frontier<PathNode> frontier = fr;
        // Add the start node as a PathNode
        frontier.push(new PathNode(start, null, 0));

        // Loop until the frontier is empty to find the goal node.
        while (!frontier.empty()) {
            // Get the next PathNode to see if we have found the goal path.
            PathNode currentPath = frontier.pop();
            // Check if the end vertex is the goal node
            Vertex end = currentPath.node;
            System.out.println("Expanded Node: " + end.label + " - Cost: " + currentPath.pathCost +
                    " - Path: " + currentPath.convert());
            if (end.equals(goal)) {
                // If we have found the goal, then return the PathNode converted to a list of vertex
                return currentPath;
            }
            // Extend the current PathNode to its neighbors, and all neighbor PathNodes to frontier.
            for (Edge neighbor : graph.getAdjVertices(end).reversed()) {
                // Uses the provided higher order function to dictate the add method. This prevents
                // the algorithm from knowing what search it is doing.
                fobj.addToFrontier(frontier, currentPath, neighbor, limit);
            }
        }
        return null;
    }

    public static void addToFrontier(Frontier<PathNode> frontier, PathNode currentPath, Edge neighbor,
                                     int limit) {
        Vertex v = currentPath.node;
        Vertex dest = neighbor.destination;
        int cost = neighbor.weight;
        PathNode newPath = new PathNode(dest, currentPath, currentPath.pathCost + cost);
        if (newPath.getSize() - 1 <= limit) {
            frontier.push(newPath);
        }
    }

    public static void addToFrontierBestFS(Frontier<PathNode> frontier, PathNode currentPath, Edge neighbor,
                                     int limit) {
        Vertex v = currentPath.node;
        Vertex dest = neighbor.destination;
        PathNode newPath = new PathNode(dest, currentPath, dest.heuristic);
        if (newPath.getSize() - 1 <= limit) {
            frontier.push(newPath);
        }
    }

    public static void addToFrontierAStar(Frontier<PathNode> frontier, PathNode currentPath, Edge neighbor,
                                         int limit) {
        Vertex v = currentPath.node;
        Vertex dest = neighbor.destination;
        int cost = neighbor.weight;
        int totalCost = 0;
        if (currentPath.getSize() >= 2) {
            totalCost += currentPath.pathCost + cost + dest.heuristic - v.heuristic;
        } else {
            totalCost += currentPath.pathCost + cost + dest.heuristic;
        }
        PathNode newPath = new PathNode(dest, currentPath, totalCost);
        if (newPath.getSize() - 1 <= limit) {
            frontier.push(newPath);
        }
    }

    public static PathNode ids(Graph graph, Vertex start, Vertex goal, Frontier<PathNode> fr, int maxDepth,
                                   FuncInterface fobj) {
        for (int i = 1; i <= maxDepth; i++) {
            if (search(graph, start, goal, fr, i, fobj) != null) {
                return search(graph, start, goal, fr, i, fobj);
            }
        }
        return null;
    }

    public static PathNode bAndB(Graph graph, Vertex start, Vertex goal, Frontier<PathNode> fr,
                                 int limit, FuncInterface fobj) {
        int upperBound = Integer.MAX_VALUE;
        PathNode bestSolution = null;

        Frontier<PathNode> frontier = fr;
        // Add the start node as a PathNode
        frontier.push(new PathNode(start, null, 0));

        while (!frontier.empty()) {
            PathNode currentPath = frontier.pop();
            Vertex end = currentPath.node;
            System.out.println("Expanded Node: " + end.label + " - Cost: " + currentPath.pathCost
                    + " - Path: " + currentPath.convert());
            if (end.equals(goal)) {
                if (currentPath.pathCost < upperBound) {
                    upperBound = currentPath.pathCost;
                    bestSolution = currentPath;
                }
            }
            if (currentPath.pathCost < upperBound) {
                for (Edge neighbor : graph.getAdjVertices(end).reversed()) {
                    fobj.addToFrontier(frontier, currentPath, neighbor, limit);
                }
            }
        }
        return bestSolution;
    }

    public static void performDFSSearch(Graph graph, Vertex startNode, Vertex goal) {
        System.out.println("---------------------------------------------------------");
        System.out.println("Performing DFS Search");
        FuncInterface fobj = (Frontier<PathNode> frontier, PathNode currentPath, Edge neighbor,
                              int limit) ->
        {
            addToFrontier(frontier, currentPath, neighbor, limit);
        };
        // Set the frontier data structure in order to change to different kinds of search.
        Frontier<PathNode> frontier = new Frontier<>("stack");

        PathNode p = search(graph, startNode, goal, frontier, 6, fobj);
        if (p != null) {
            List<Vertex> path = p.convert();
            System.out.println("Path Returned: " + path + " - Cost: " + p.pathCost);
        } else {
            System.out.println("No path exists from Start Node " + startNode.label +
                    " to Goal Node " + goal.label);
        }
    }

    public static void performBFSSearch(Graph graph, Vertex startNode, Vertex goal) {
        System.out.println("---------------------------------------------------------");
        System.out.println("Performing BFS Search");
        FuncInterface fobj = (Frontier<PathNode> frontier, PathNode currentPath, Edge neighbor,
                              int limit) ->
        {
            addToFrontier(frontier, currentPath, neighbor, limit);
        };
        // Set the frontier data structure in order to change to different kinds of search.
        Frontier<PathNode> frontier = new Frontier<>("queue");

        PathNode p = search(graph, startNode, goal, frontier, 6, fobj);
        if (p != null) {
            List<Vertex> path = p.convert();
            System.out.println("Path Returned: " + path + " - Cost: " + p.pathCost);
        } else {
            System.out.println("No path exists from Start Node " + startNode.label +
                    " to Goal Node " + goal.label);
        }
    }

    public static void performIDSSearch(Graph graph, Vertex startNode, Vertex goal) {
        System.out.println("---------------------------------------------------------");
        System.out.println("Performing IDS Search");
        FuncInterface fobj = (Frontier<PathNode> frontier, PathNode currentPath, Edge neighbor,
                              int limit) ->
        {
            addToFrontier(frontier, currentPath, neighbor, limit);
        };
        // Set the frontier data structure in order to change to different kinds of search.
        Frontier<PathNode> frontier = new Frontier<>("stack");

        PathNode p = ids(graph, startNode, goal, frontier, 6, fobj);
        if (p != null) {
            List<Vertex> path = p.convert();
            System.out.println("Path Returned: " + path + " - Cost: " + p.pathCost);
        } else {
            System.out.println("No path exists from Start Node " + startNode.label +
                    " to Goal Node " + goal.label);
        }
    }

    public static void performBAndBSearch(Graph graph, Vertex startNode, Vertex goal) {
        System.out.println("---------------------------------------------------------");
        System.out.println("Performing BandB Search");
        FuncInterface fobj = (Frontier<PathNode> frontier, PathNode currentPath, Edge neighbor,
                              int limit) ->
        {
            addToFrontierAStar(frontier, currentPath, neighbor, limit);
        };
        // Set the frontier data structure in order to change to different kinds of search.
        Frontier<PathNode> frontier = new Frontier<>("stack");

        PathNode p = bAndB(graph, startNode, goal, frontier, 6, fobj);
        if (p != null) {
            List<Vertex> path = p.convert();
            System.out.println("Path Returned: " + path + " - Cost: " + p.pathCost);
        } else {
            System.out.println("No path exists from Start Node " + startNode.label +
                    " to Goal Node " + goal.label);
        }
    }

    public static void performLCFSSearch(Graph graph, Vertex startNode, Vertex goal) {
        System.out.println("---------------------------------------------------------");
        System.out.println("Performing LCFS Search");
        FuncInterface fobj = (Frontier<PathNode> frontier, PathNode currentPath, Edge neighbor,
                              int limit) ->
        {
            addToFrontier(frontier, currentPath, neighbor, limit);
        };
        // Set the frontier data structure in order to change to different kinds of search.
        Frontier<PathNode> frontier = new Frontier<>("lcfs");
        PathNode p = search(graph, startNode, goal, frontier, 6, fobj);
        if (p != null) {
            List<Vertex> path = p.convert();
            System.out.println("Path Returned: " + path + " - Cost: " + p.pathCost);
        } else {
            System.out.println("No path exists from Start Node " + startNode.label +
                    " to Goal Node " + goal.label);
        }
    }

    public static void performAStarSearch(Graph graph, Vertex startNode, Vertex goal) {
        System.out.println("---------------------------------------------------------");
        System.out.println("Performing AStar Search");
        FuncInterface fobj = (Frontier<PathNode> frontier, PathNode currentPath, Edge neighbor,
                              int limit) ->
        {
            addToFrontierAStar(frontier, currentPath, neighbor, limit);
        };
        // Set the frontier data structure in order to change to different kinds of search.
        Frontier<PathNode> frontier = new Frontier<>("astar");

        PathNode p = search(graph, startNode, goal, frontier, 6, fobj);
        if (p != null) {
            List<Vertex> path = p.convert();
            System.out.println("Path Returned: " + path + " - Cost: " + p.pathCost);
        } else {
            System.out.println("No path exists from Start Node " + startNode.label +
                    " to Goal Node " + goal.label);
        }
    }

    public static void performBestFSSearch(Graph graph, Vertex startNode, Vertex goal) {
        System.out.println("---------------------------------------------------------");
        System.out.println("Performing BestFS Search");
        FuncInterface fobj = (Frontier<PathNode> frontier, PathNode currentPath, Edge neighbor,
                              int limit) ->
        {
            addToFrontierBestFS(frontier, currentPath, neighbor, limit);
        };
        // Set the frontier data structure in order to change to different kinds of search.
        Frontier<PathNode> frontier = new Frontier<>("bestfs");

        PathNode p = search(graph, startNode, goal, frontier, 6, fobj);
        if (p != null) {
            List<Vertex> path = p.convert();
            System.out.println("Path Returned: " + path + " - Cost: " + p.pathCost);
        } else {
            System.out.println("No path exists from Start Node " + startNode.label +
                    " to Goal Node " + goal.label);
        }
    }
}