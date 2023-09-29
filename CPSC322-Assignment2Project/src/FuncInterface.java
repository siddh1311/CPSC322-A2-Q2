public interface FuncInterface {
    void addToFrontier(Frontier<PathNode> frontier, PathNode currentPath, Edge neighbor,
                       int limit);
}
