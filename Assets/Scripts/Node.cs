using UnityEngine;

public class Node
{
    public bool walkable;
    public Vector3 worldPosition;

    public int gridX;
    public int gridZ;

    public int gCost;
    public int hCost;

    public Node parent;

    public Node(bool walkable, Vector3 worldPosition, int gridX, int gridZ)
    {
        this.walkable = walkable;
        this.worldPosition = worldPosition;
        this.gridX = gridX;
        this.gridZ = gridZ;
    }

    public int fCost
    {
        get
        {
            return gCost + hCost;
        }
    }
}

