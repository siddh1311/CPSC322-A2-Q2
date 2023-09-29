import java.util.*;

public class Frontier<E> {
    Collection<E> frontier;
    String type;
    Frontier(String struc) {
        this.type = struc;
        if (struc.equals("queue")) {
            frontier = new LinkedList<E>();
        } else if (struc.equals("stack")) {
            frontier = new Stack<E>();
        } else if (struc.equals("lcfs") || struc.equals("astar") || struc.equals("bestfs")) {
            frontier = new PriorityQueue<E>();
        }
    }

    public void push(E element) {
        frontier.add(element);
    }

    public E pop() {
        if (type.equals("queue")) {
            return ((List<E>)frontier).remove(0);
//            return frontier.remove(0);
        } else if (type.equals("stack")) {
            return ((Stack<E>)frontier).removeLast();
        } else if (type.equals("lcfs") || type.equals("astar") || type.equals("bestfs")) {
            return ((Queue<E>)frontier).remove();
        }
        return null;
    }

    public boolean empty() {
        return frontier.isEmpty();
    }

}
