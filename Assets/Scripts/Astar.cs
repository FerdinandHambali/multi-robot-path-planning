using UnityEngine;
using System.Collections.Generic;

public class AStar : MonoBehaviour
{
    public Transform robot;
    public Transform target;

    PathGrid grid;

    void Start()
    {
        grid = GetComponent<PathGrid>();

        FindPath(robot.position, target.position);
    }

    void FindPath(Vector3 startPosition, Vector3 targetPosition)
    {
        Node startNode =
            grid.NodeFromWorldPoint(startPosition);

        Node targetNode =
            grid.NodeFromWorldPoint(targetPosition);

        grid.ResetSearchData();

        startNode.gCost = 0;

        List<Node> openSet = new List<Node>();
        HashSet<Node> closedSet = new HashSet<Node>();

        openSet.Add(startNode);

        while (openSet.Count > 0)
        {
            Node currentNode = openSet[0];

            for (int i = 1; i < openSet.Count; i++)
            {
                if (openSet[i].fCost < currentNode.fCost ||
                    openSet[i].fCost == currentNode.fCost &&
                    openSet[i].hCost < currentNode.hCost)
                {
                    currentNode = openSet[i];
                }
            }

            openSet.Remove(currentNode);
            closedSet.Add(currentNode);

            if (currentNode == targetNode)
            {
                RetracePath(startNode, targetNode);

                Debug.Log("Path found!");

                return;
            }

            foreach (Node neighbour in grid.GetNeighbours(currentNode))
            {
                if (!neighbour.walkable ||
                    closedSet.Contains(neighbour))
                {
                    continue;
                }

                int newMovementCost =
                    currentNode.gCost +
                    GetDistance(currentNode, neighbour);

                if (newMovementCost < neighbour.gCost ||
                    !openSet.Contains(neighbour))
                {
                    neighbour.gCost = newMovementCost;

                    neighbour.hCost =
                        GetDistance(neighbour, targetNode);

                    neighbour.parent = currentNode;

                    if (!openSet.Contains(neighbour))
                    {
                        openSet.Add(neighbour);
                    }
                }
            }
        }

        Debug.Log("No path found!");
    }

    void RetracePath(Node startNode, Node endNode)
    {
        List<Node> path = new List<Node>();

        Node currentNode = endNode;

        while (currentNode != startNode)
        {
            path.Add(currentNode);

            currentNode = currentNode.parent;
        }

        path.Reverse();

        grid.path = path;
    }

    int GetDistance(Node nodeA, Node nodeB)
    {
        int distanceX =
            Mathf.Abs(nodeA.gridX - nodeB.gridX);

        int distanceZ =
            Mathf.Abs(nodeA.gridZ - nodeB.gridZ);

        if (distanceX > distanceZ)
        {
            return 14 * distanceZ +
                   10 * (distanceX - distanceZ);
        }

        return 14 * distanceX +
               10 * (distanceZ - distanceX);
    }
}