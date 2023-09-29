import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Graph {
    private Map<Vertex, List<Edge>> adjVertices;
    private List<Vertex> vertices;
    private List<Edge> edges;
    Graph() {
        adjVertices = new HashMap<>();
        vertices = new ArrayList<>();
        edges = new ArrayList<>();
    }

    void addVertex(Vertex v) {
        adjVertices.put(v, new ArrayList<>());
        vertices.add(v);
    }
    void addEdge(Vertex source, Vertex dest, int weight) {
        Edge e = new Edge(source, dest, weight);
        adjVertices.get(source).add(e);
        edges.add(e);
    }

    Edge getEdge(Vertex source, Vertex dest) {
        for (Edge e : edges) {
            if (e.source.equals(source) && e.destination.equals(dest)) {
                return e;
            }
        }
        return null;
    }

    List<Edge> getAdjVertices(Vertex v) {
        return adjVertices.get(v);
    }

    List<Vertex> getVertices() {
        return this.vertices;
    }

    Vertex getVertex(String label) {
        for (Vertex v : getVertices()) {
            if (v.label.equals(label)) {
                return v;
            }
        }
        return null;
    }
}
