import java.util.ArrayList;
import java.util.List;

public class PathNode implements Comparable{
    Vertex node;
    PathNode parent;
    int pathCost;
    public PathNode(Vertex node, PathNode parent, int pathCost) {
        this.node = node;
        this.parent = parent;
        this.pathCost = pathCost;
    }

    List<Vertex> convert() {
        PathNode temp = this;
        List<Vertex> path = new ArrayList<>();
        while (temp != null) {
            path.add(temp.node);
            temp = temp.parent;
        }
        return path.reversed();
    }

    int getSize() {
        int size = 0;
        PathNode temp = this;
        while(temp != null) {
            size++;
            temp = temp.parent;
        }
        return size;
    }

    @Override
    public int compareTo(Object o) {
        if (this.pathCost < ((PathNode)o).pathCost) {
            return -1;
        } else if (this.pathCost > ((PathNode)o).pathCost) {
            return 1;
        }
        return 0;
    }
}
