public class Edge {
    Vertex source;
    Vertex destination;
    int weight;

    Edge(Vertex source, Vertex destination, int weight) {
        this.source = source;
        this.destination = destination;
        this.weight = weight;
    }
}
