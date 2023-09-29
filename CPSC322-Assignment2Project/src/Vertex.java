public class Vertex {
    String label;
    int heuristic;
    Vertex(String label, int heuristic) {
        this.label = label;
        this.heuristic = heuristic;
    }

    @Override
    public boolean equals(Object obj) {
        return super.equals(obj);
    }

    @Override
    public String toString() {
        return label;
    }
}
