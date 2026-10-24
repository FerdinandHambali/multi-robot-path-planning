using UnityEngine;
using System.Collections.Generic;

public class PathGrid : MonoBehaviour
{
    public Vector2 gridWorldSize = new Vector2(10, 10);

    public float nodeRadius = 0.25f;
    public float robotRadius = 0.5f;

    public LayerMask obstacleMask;

    Node[,] grid;
    public List<Node> path;

    float nodeDiameter;
    int gridSizeX;
    int gridSizeZ;

    public HashSet<Node> GetNodesWithinRadius(
    Vector3 worldPosition,
    int radiusCells)
    {
        HashSet<Node> blockedNodes =
            new HashSet<Node>();

        Node centre =
            NodeFromWorldPoint(worldPosition);

        for (int x = -radiusCells;
             x <= radiusCells;
             x++)
        {
            for (int z = -radiusCells;
                 z <= radiusCells;
                 z++)
            {
                int checkX = centre.gridX + x;
                int checkZ = centre.gridZ + z;

                if (checkX >= 0 &&
                    checkX < gridSizeX &&
                    checkZ >= 0 &&
                    checkZ < gridSizeZ)
                {
                    // Use same Manhattan-distance idea
                    // as our conflict detection
                    if (Mathf.Abs(x) + Mathf.Abs(z)
                        <= radiusCells)
                    {
                        blockedNodes.Add(
                            grid[checkX, checkZ]
                        );
                    }
                }
            }
        }

        return blockedNodes;
    }

    void Awake()
    {
        nodeDiameter = nodeRadius * 2;

        gridSizeX = Mathf.RoundToInt(gridWorldSize.x / nodeDiameter);
        gridSizeZ = Mathf.RoundToInt(gridWorldSize.y / nodeDiameter);

        CreateGrid();
    }

    void CreateGrid()
    {
        grid = new Node[gridSizeX, gridSizeZ];

        Vector3 bottomLeft =
            transform.position
            - Vector3.right * gridWorldSize.x / 2
            - Vector3.forward * gridWorldSize.y / 2;

        for (int x = 0; x < gridSizeX; x++)
        {
            for (int z = 0; z < gridSizeZ; z++)
            {
                Vector3 worldPoint =
                    bottomLeft
                    + Vector3.right * (x * nodeDiameter + nodeRadius)
                    + Vector3.forward * (z * nodeDiameter + nodeRadius);

                bool walkable =
                    !Physics.CheckSphere(
                        worldPoint,
                        robotRadius,
                        obstacleMask
                    );

                grid[x, z] =
                    new Node(
                        walkable,
                        worldPoint,
                        x,
                        z
                    );
            }
        }
    }
    public Node NodeFromWorldPoint(Vector3 worldPosition)
    {
        Vector3 localPosition = worldPosition - transform.position;

        float percentX =
            (localPosition.x + gridWorldSize.x / 2) / gridWorldSize.x;

        float percentZ =
            (localPosition.z + gridWorldSize.y / 2) / gridWorldSize.y;

        percentX = Mathf.Clamp01(percentX);
        percentZ = Mathf.Clamp01(percentZ);

        int x = Mathf.RoundToInt((gridSizeX - 1) * percentX);
        int z = Mathf.RoundToInt((gridSizeZ - 1) * percentZ);

        return grid[x, z];
    }

    public void ResetSearchData()
    {
        if (grid == null)
            return;

        foreach (Node node in grid)
        {
            node.gCost = int.MaxValue;
            node.hCost = 0;
            node.parent = null;
        }
    }

    public List<Node> GetNeighbours(Node node)
    {
        List<Node> neighbours = new List<Node>();

        int[,] directions = //Only check 4 directions (up, down, left, right)
        {
        { 1, 0 },
        { -1, 0 },
        { 0, 1 },
        { 0, -1 }
    };

        for (int i = 0; i < 4; i++)
        {
            int checkX = node.gridX + directions[i, 0];
            int checkZ = node.gridZ + directions[i, 1];

            if (checkX >= 0 &&
                checkX < gridSizeX &&
                checkZ >= 0 &&
                checkZ < gridSizeZ)
            {
                neighbours.Add(grid[checkX, checkZ]);
            }
        }

        return neighbours;
    }

    void OnDrawGizmos()
    {
        if (Application.isPlaying)
            return;
        Gizmos.DrawWireCube(
            transform.position,
            new Vector3(
                gridWorldSize.x,
                0.1f,
                gridWorldSize.y
            )
        );

        if (grid != null)
        {
            foreach (Node node in grid)
            {
                if (!node.walkable)
                {
                    Gizmos.color = Color.red;
                }
                else if (path != null && path.Contains(node))
                {
                    Gizmos.color = Color.blue;
                }
                else
                {
                    Gizmos.color = Color.green;
                }

                Gizmos.DrawCube(
                    node.worldPosition + Vector3.up * 0.05f,
                    Vector3.one * (nodeDiameter - 0.05f)
                );
            }
        }
    }
}