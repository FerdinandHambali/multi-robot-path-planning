public class SpaceTimeNode
{
    public Node node;

    public int time;

    public int gCost;
    public int hCost;

    public SpaceTimeNode parent;


    public int fCost
    {
        get
        {
            return gCost + hCost;
        }
    }


    public SpaceTimeNode(
        Node node,
        int time,
        int gCost,
        int hCost,
        SpaceTimeNode parent)
    {
        this.node = node;
        this.time = time;

        this.gCost = gCost;
        this.hCost = hCost;

        this.parent = parent;
    }
}