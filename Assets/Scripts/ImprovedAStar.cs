using UnityEngine;
using System.Collections.Generic;

public class ImprovedAStar : MonoBehaviour
{
    // ==================================================
    // REFERENCES
    // ==================================================
    public double PlanningTimeMilliseconds
    {
        get;
        private set;
    }

    public Transform robot;
    public Transform target;
    public RobotPath robotPath;


    // ==================================================
    // SETTINGS
    // ==================================================

    // Maximum number of paths stored in the
    // Path Expert Database.
    public int maximumPaths = 5;

    [Header("Fair Timing Experiment")]
    [Tooltip("ON: suppress messages inside measured planning operations. The path database is still printed after timing finishes.")]
    public bool fairBenchmarkMode = true;

    // 0 = Path 1
    // 1 = Path 2
    // etc.
    //
    // Stage 3 will change this automatically.
    public int selectedPathIndex = 0;


    // ==================================================
    // INTERNAL
    // ==================================================

    private PathGrid grid;

    private List<List<Node>> pathDatabase =
        new List<List<Node>>();


    // ==================================================
    // SAVED NODE STATE
    // ==================================================

    private class NodeSearchState
    {
        public int gCost;
        public int hCost;
        public Node parent;


        public NodeSearchState(Node node)
        {
            gCost = node.gCost;
            hCost = node.hCost;
            parent = node.parent;
        }
    }


    // ==================================================
    // MARK
    //
    // Stores OPEN and CLOSE at a point where
    // multiple nodes have the same A* valuation.
    // ==================================================

    private class SearchSnapshot
    {
        public List<Node> openSet;

        public HashSet<Node> closedSet;

        public Dictionary<Node, NodeSearchState>
            nodeStates;

        public List<Node> alternativeNodes;
    }


    // ==================================================
    // START
    // ==================================================

    void Start()
    {
        grid =
            GetComponent<PathGrid>();


        if (grid == null)
        {
            Debug.LogError(
                "ImprovedAStar requires PathGrid."
            );

            return;
        }


        if (robot == null)
        {
            Debug.LogError(
                "Robot is not assigned."
            );

            return;
        }


        if (target == null)
        {
            Debug.LogError(
                robot.name +
                ": Target is not assigned."
            );

            return;
        }


        if (robotPath == null)
        {
            Debug.LogError(
                robot.name +
                ": RobotPath is not assigned."
            );

            return;
        }


        System.Diagnostics.Stopwatch planningWatch =
            System.Diagnostics.Stopwatch.StartNew();


        GeneratePathDatabase(
            robot.position,
            target.position
        );


        planningWatch.Stop();

        // All verbose database output happens OUTSIDE the stopwatch.
        PrintPathDatabase();

        PlanningTimeMilliseconds =
            planningWatch.Elapsed.TotalMilliseconds;


        Debug.Log(
            robot.name +
            " Path Planning Computation Time = " +
            PlanningTimeMilliseconds.ToString("F3") +
            " ms"
        );
    }


    // ==================================================
    // ALGORITHM 3
    //
    // GENERATE PATH EXPERT DATABASE
    // ==================================================

    void GeneratePathDatabase(
        Vector3 startPosition,
        Vector3 targetPosition)
    {
        pathDatabase.Clear();


        Node startNode =
            grid.NodeFromWorldPoint(
                startPosition
            );


        Node targetNode =
            grid.NodeFromWorldPoint(
                targetPosition
            );


        // Stores Mark states.
        List<SearchSnapshot> marks =
            new List<SearchSnapshot>();


        // ==================================================
        // 1. PRIMARY A* SEARCH
        // ==================================================

        List<Node> primaryPath =
            FindPrimaryPath(
                startNode,
                targetNode,
                marks
            );


        if (primaryPath == null)
        {
            Debug.LogError(
                robot.name +
                ": no path found."
            );

            return;
        }


        pathDatabase.Add(
            primaryPath
        );


        if (!fairBenchmarkMode) Debug.Log(
            robot.name +
            ": Primary path found."
        );


        if (!fairBenchmarkMode) Debug.Log(
            robot.name +
            ": Mark states recorded = " +
            marks.Count
        );


        // ==================================================
        // 2. REMOUNT SAVED MARK STATES
        // ==================================================

        int attempts = 0;


        // Safety termination.
        int maximumAttempts =
            Mathf.Max(
                50,
                maximumPaths * 10
            );


        foreach (
            SearchSnapshot snapshot
            in marks
        )
        {
            foreach (
                Node alternativeNode
                in snapshot.alternativeNodes
            )
            {
                if (
                    pathDatabase.Count >=
                    maximumPaths
                )
                {
                    break;
                }


                if (
                    attempts >=
                    maximumAttempts
                )
                {
                    break;
                }


                attempts++;


                RestoreSnapshot(
                    snapshot
                );


                List<Node> openSet =
                    new List<Node>(
                        snapshot.openSet
                    );


                HashSet<Node> closedSet =
                    new HashSet<Node>(
                        snapshot.closedSet
                    );


                if (
                    !openSet.Contains(
                        alternativeNode
                    )
                )
                {
                    continue;
                }


                // Continue search from saved Mark,
                // forcing an alternative equal-f node.
                List<Node> alternativePath =
                    ContinueSearch(
                        startNode,
                        targetNode,
                        openSet,
                        closedSet,
                        alternativeNode
                    );


                if (
                    alternativePath != null &&
                    !PathAlreadyStored(
                        alternativePath
                    )
                )
                {
                    pathDatabase.Add(
                        alternativePath
                    );
                }
            }


            if (
                pathDatabase.Count >=
                maximumPaths ||
                attempts >=
                maximumAttempts
            )
            {
                break;
            }
        }


        // ==================================================
        // 3. SORT PATH DATABASE
        // ==================================================

        pathDatabase.Sort(
            (pathA, pathB) =>
                GetPathCost(pathA)
                .CompareTo(
                    GetPathCost(pathB)
                )
        );


        // ==================================================
        // 4. ALGORITHM 2 INITIAL CONDITION
        //
        // Every robot starts with Path 1.
        //
        // Stage 3 Select / Replace may later replace it.
        // ==================================================

        selectedPathIndex = 0;


        robotPath.path =
            pathDatabase[0];


        if (!fairBenchmarkMode) Debug.Log(
            robot.name +
            " selected Path 1"
        );


        // Database printing moved to Start(), after the stopwatch stops.
    }


    // ==================================================
    // PRIMARY A*
    //
    // Only this search creates Mark states.
    // ==================================================

    List<Node> FindPrimaryPath(
        Node startNode,
        Node targetNode,
        List<SearchSnapshot> marks)
    {
        grid.ResetSearchData();


        List<Node> openSet =
            new List<Node>();


        HashSet<Node> closedSet =
            new HashSet<Node>();


        startNode.gCost = 0;


        startNode.hCost =
            GetDistance(
                startNode,
                targetNode
            );


        startNode.parent = null;


        openSet.Add(
            startNode
        );


        while (
            openSet.Count > 0
        )
        {
            Node currentNode =
                GetBestNode(
                    openSet
                );


            // ==================================================
            // MARK
            //
            // Find other OPEN nodes having
            // the same f = g + h value.
            // ==================================================

            List<Node> equalValueNodes =
                GetEqualValueNodes(
                    openSet,
                    currentNode
                );


            if (
                equalValueNodes.Count > 0
            )
            {
                SearchSnapshot snapshot =
                    CreateSnapshot(
                        openSet,
                        closedSet,
                        equalValueNodes
                    );


                marks.Add(
                    snapshot
                );
            }


            // ==================================================
            // TARGET FOUND
            // ==================================================

            if (
                currentNode ==
                targetNode
            )
            {
                return RetracePath(
                    startNode,
                    targetNode
                );
            }


            // ==================================================
            // NORMAL A*
            // ==================================================

            openSet.Remove(
                currentNode
            );


            closedSet.Add(
                currentNode
            );


            foreach (
                Node neighbour
                in grid.GetNeighbours(
                    currentNode
                )
            )
            {
                if (
                    !neighbour.walkable ||
                    closedSet.Contains(
                        neighbour
                    )
                )
                {
                    continue;
                }


                int newMovementCost =
                    currentNode.gCost +
                    GetDistance(
                        currentNode,
                        neighbour
                    );


                if (
                    newMovementCost <
                    neighbour.gCost ||
                    !openSet.Contains(
                        neighbour
                    )
                )
                {
                    neighbour.gCost =
                        newMovementCost;


                    neighbour.hCost =
                        GetDistance(
                            neighbour,
                            targetNode
                        );


                    neighbour.parent =
                        currentNode;


                    if (
                        !openSet.Contains(
                            neighbour
                        )
                    )
                    {
                        openSet.Add(
                            neighbour
                        );
                    }
                }
            }
        }


        return null;
    }


    // ==================================================
    // CONTINUE SEARCH AFTER REMOUNT
    //
    // Does NOT create more Mark states.
    // ==================================================

    List<Node> ContinueSearch(
        Node startNode,
        Node targetNode,
        List<Node> openSet,
        HashSet<Node> closedSet,
        Node forcedNode)
    {
        bool firstIteration = true;


        while (
            openSet.Count > 0
        )
        {
            Node currentNode;


            if (
                firstIteration &&
                openSet.Contains(
                    forcedNode
                )
            )
            {
                currentNode =
                    forcedNode;

                firstIteration =
                    false;
            }
            else
            {
                currentNode =
                    GetBestNode(
                        openSet
                    );

                firstIteration =
                    false;
            }


            // ==================================================
            // TARGET FOUND
            // ==================================================

            if (
                currentNode ==
                targetNode
            )
            {
                return RetracePath(
                    startNode,
                    targetNode
                );
            }


            openSet.Remove(
                currentNode
            );


            closedSet.Add(
                currentNode
            );


            foreach (
                Node neighbour
                in grid.GetNeighbours(
                    currentNode
                )
            )
            {
                if (
                    !neighbour.walkable ||
                    closedSet.Contains(
                        neighbour
                    )
                )
                {
                    continue;
                }


                int newMovementCost =
                    currentNode.gCost +
                    GetDistance(
                        currentNode,
                        neighbour
                    );


                if (
                    newMovementCost <
                    neighbour.gCost ||
                    !openSet.Contains(
                        neighbour
                    )
                )
                {
                    neighbour.gCost =
                        newMovementCost;


                    neighbour.hCost =
                        GetDistance(
                            neighbour,
                            targetNode
                        );


                    neighbour.parent =
                        currentNode;


                    if (
                        !openSet.Contains(
                            neighbour
                        )
                    )
                    {
                        openSet.Add(
                            neighbour
                        );
                    }
                }
            }
        }


        return null;
    }


    // ==================================================
    // GET BEST A* NODE
    // ==================================================

    Node GetBestNode(
        List<Node> openSet)
    {
        Node bestNode =
            openSet[0];


        for (
            int i = 1;
            i < openSet.Count;
            i++
        )
        {
            if (
                openSet[i].fCost <
                bestNode.fCost
            )
            {
                bestNode =
                    openSet[i];
            }
        }


        return bestNode;
    }


    // ==================================================
    // SAME-VALUATION NODES
    //
    // f(n) = g(n) + h(n)
    // ==================================================

    List<Node> GetEqualValueNodes(
        List<Node> openSet,
        Node currentNode)
    {
        List<Node> equalNodes =
            new List<Node>();


        foreach (
            Node node
            in openSet
        )
        {
            if (
                node ==
                currentNode
            )
            {
                continue;
            }


            if (
                node.fCost ==
                currentNode.fCost
            )
            {
                equalNodes.Add(
                    node
                );
            }
        }


        return equalNodes;
    }


    // ==================================================
    // CREATE MARK
    // ==================================================

    SearchSnapshot CreateSnapshot(
        List<Node> openSet,
        HashSet<Node> closedSet,
        List<Node> alternatives)
    {
        SearchSnapshot snapshot =
            new SearchSnapshot();


        snapshot.openSet =
            new List<Node>(
                openSet
            );


        snapshot.closedSet =
            new HashSet<Node>(
                closedSet
            );


        snapshot.alternativeNodes =
            new List<Node>(
                alternatives
            );


        snapshot.nodeStates =
            new Dictionary<
                Node,
                NodeSearchState
            >();


        foreach (
            Node node
            in openSet
        )
        {
            if (
                !snapshot.nodeStates
                    .ContainsKey(node)
            )
            {
                snapshot.nodeStates.Add(
                    node,
                    new NodeSearchState(
                        node
                    )
                );
            }
        }


        foreach (
            Node node
            in closedSet
        )
        {
            if (
                !snapshot.nodeStates
                    .ContainsKey(node)
            )
            {
                snapshot.nodeStates.Add(
                    node,
                    new NodeSearchState(
                        node
                    )
                );
            }
        }


        return snapshot;
    }


    // ==================================================
    // REMOUNT MARK
    // ==================================================

    void RestoreSnapshot(
        SearchSnapshot snapshot)
    {
        grid.ResetSearchData();


        foreach (
            KeyValuePair<
                Node,
                NodeSearchState
            > entry
            in snapshot.nodeStates
        )
        {
            Node node =
                entry.Key;


            NodeSearchState state =
                entry.Value;


            node.gCost =
                state.gCost;


            node.hCost =
                state.hCost;


            node.parent =
                state.parent;
        }
    }


    // ==================================================
    // RETRACE PATH
    // ==================================================

    List<Node> RetracePath(
        Node startNode,
        Node endNode)
    {
        List<Node> path =
            new List<Node>();


        Node currentNode =
            endNode;


        while (
            currentNode !=
            startNode
        )
        {
            if (
                currentNode == null
            )
            {
                return null;
            }


            path.Add(
                currentNode
            );


            currentNode =
                currentNode.parent;
        }


        path.Add(
            startNode
        );


        path.Reverse();


        return path;
    }


    // ==================================================
    // PATH COST
    // ==================================================

    int GetPathCost(
        List<Node> path)
    {
        int cost = 0;


        for (
            int i = 0;
            i < path.Count - 1;
            i++
        )
        {
            cost +=
                GetDistance(
                    path[i],
                    path[i + 1]
                );
        }


        return cost;
    }


    // ==================================================
    // DUPLICATE CHECK
    // ==================================================

    bool PathAlreadyStored(
        List<Node> candidate)
    {
        foreach (
            List<Node> stored
            in pathDatabase
        )
        {
            if (
                stored.Count !=
                candidate.Count
            )
            {
                continue;
            }


            bool same = true;


            for (
                int i = 0;
                i < stored.Count;
                i++
            )
            {
                if (
                    stored[i] !=
                    candidate[i]
                )
                {
                    same = false;

                    break;
                }
            }


            if (same)
            {
                return true;
            }
        }


        return false;
    }


    // ==================================================
    // MANHATTAN DISTANCE
    //
    // 4-direction grid movement.
    // ==================================================

    int GetDistance(
        Node nodeA,
        Node nodeB)
    {
        int dx =
            Mathf.Abs(
                nodeA.gridX -
                nodeB.gridX
            );


        int dz =
            Mathf.Abs(
                nodeA.gridZ -
                nodeB.gridZ
            );


        return
            10 *
            (dx + dz);
    }


    // ==================================================
    // DATABASE OUTPUT
    // ==================================================

    void PrintPathDatabase()
    {
        Debug.Log(
            "===== " +
            robot.name +
            " PATH EXPERT DATABASE ====="
        );


        for (
            int i = 0;
            i < pathDatabase.Count;
            i++
        )
        {
            Debug.Log(
                robot.name +
                " | Path " +
                (i + 1) +
                " | Nodes: " +
                pathDatabase[i].Count +
                " | Cost: " +
                GetPathCost(
                    pathDatabase[i]
                )
            );
        }


        Debug.Log(
            robot.name +
            " | Total Paths: " +
            pathDatabase.Count
        );


        Debug.Log(
            "================================"
        );
    }


    // ==================================================
    // PUBLIC ACCESS
    //
    // Used by FleetConflictAnalyzer / Stage 3
    // ==================================================


    // Current selected path.
    public List<Node> GetSelectedPath()
    {
        if (
            robotPath == null
        )
        {
            return null;
        }


        return robotPath.path;
    }


    // Current path number.
    //
    // Returns:
    // Path 1 -> 1
    // Path 2 -> 2
    // etc.
    public int GetSelectedPathNumber()
    {
        if (
            pathDatabase == null ||
            pathDatabase.Count == 0
        )
        {
            return -1;
        }


        int index =
            Mathf.Clamp(
                selectedPathIndex,
                0,
                pathDatabase.Count - 1
            );


        return index + 1;
    }


    // ==================================================
    // NUMBER OF STORED PATHS
    // ==================================================

    public int GetPathCount()
    {
        if (
            pathDatabase == null
        )
        {
            return 0;
        }


        return pathDatabase.Count;
    }


    // ==================================================
    // GET SPECIFIC DATABASE PATH
    //
    // index 0 = Path 1
    // index 1 = Path 2
    // etc.
    // ==================================================

    public List<Node> GetDatabasePath(
        int index)
    {
        if (
            pathDatabase == null ||
            index < 0 ||
            index >= pathDatabase.Count
        )
        {
            return null;
        }


        return pathDatabase[index];
    }


    // ==================================================
    // SELECT DATABASE PATH
    //
    // Stage 3 will use this when a replacement
    // gives a lower Tc.
    // ==================================================

    public void SelectDatabasePath(
        int index)
    {
        if (
            pathDatabase == null ||
            pathDatabase.Count == 0
        )
        {
            return;
        }


        index =
            Mathf.Clamp(
                index,
                0,
                pathDatabase.Count - 1
            );


        selectedPathIndex =
            index;


        if (
            robotPath != null
        )
        {
            robotPath.path =
                pathDatabase[index];
        }
    }


    // ==================================================
    // RETURN WHOLE DATABASE
    // ==================================================

    public List<List<Node>>
        GetPathDatabase()
    {
        return pathDatabase;
    }
}