using UnityEngine;
using System.Collections.Generic;
using System.Threading.Tasks;

public class SpaceTimeAStar : MonoBehaviour
{
    // ==================================================
    // REFERENCES
    // ==================================================

    public Transform robot;
    public Transform target;

    private PathGrid grid;


    // ==================================================
    // STAGE 2 TEST SETTINGS
    //
    // KEEP BOTH OFF FOR NORMAL CBS / PCBS TESTING.
    // ==================================================

    [Header("Stage 2 Test Settings")]

    public bool useTestVertexConstraint = false;
    public int testVertexConstraintTime = 107;

    public bool useTestEdgeConstraint = false;

    public int testEdgeFromX = 57;
    public int testEdgeFromZ = 55;

    public int testEdgeToX = 57;
    public int testEdgeToZ = 56;

    public int testEdgeTime = 106;


    // ==================================================
    // FREEZE PROTECTION
    // ==================================================

    [Header("STA* Freeze Protection")]

    [Tooltip(
        "Maximum timestep STA* may search."
    )]
    public int maxTimeStep = 250;


    [Tooltip(
        "Maximum number of space-time states " +
        "STA* may expand in one search."
    )]
    public int maxExpandedStates = 15000;


    [Tooltip(
        "Maximum real computation time for one STA* search."
    )]
    public int maxSearchMilliseconds = 1500;


    // ==================================================
    // HETEROGENEOUS ROBOT SPEED
    //
    // All robots share ONE global base timestep.
    // Each robot's RobotMovement.moveSpeed determines
    // how many base timesteps are required per grid edge.
    // ==================================================

    [Header("Speed-Aware STA*")]

    [Tooltip(
        "Fallback grid-edge distance used only if the planner " +
        "cannot estimate it from neighbouring grid nodes."
    )]
    public float fallbackGridCellDistance = 0.5f;


    [HideInInspector]
    public int moveDurationSlots = 1;


    [HideInInspector]
    public float resolvedRobotSpeed = 1f;


    [HideInInspector]
    public float resolvedBaseTimeStep = 0.1f;


    [HideInInspector]
    public float resolvedGridCellDistance = 0.5f;


    // ==================================================
    // CONSTRAINTS
    // ==================================================

    public List<SpaceTimeVertexConstraint>
        activeVertexConstraints =
            new List<SpaceTimeVertexConstraint>();


    public List<SpaceTimeEdgeConstraint>
        activeEdgeConstraints =
            new List<SpaceTimeEdgeConstraint>();


    // ==================================================
    // FINAL PATH
    // ==================================================

    public List<SpaceTimeNode> finalPath =
        new List<SpaceTimeNode>();

    // Measured only for the first independent STA* search.
    // CBS replans do not overwrite this value.
    public double initialPlanningMilliseconds { get; private set; } = 0.0;


    // ==================================================
    // PARALLEL STA* SNAPSHOT
    //
    // IMPORTANT:
    // Worker threads NEVER touch Unity objects.
    //
    // The Unity grid is copied on the main thread into
    // plain C# coordinate / neighbour data.
    // ==================================================

    private ParallelGridSnapshot parallelGridSnapshot;

    private Dictionary<long, Node>
        parallelNodeLookup =
            new Dictionary<long, Node>();

    private int parallelSnapshotStartX =
        int.MinValue;

    private int parallelSnapshotStartZ =
        int.MinValue;


    // ==================================================
    // PARALLEL RESULT TYPES
    // ==================================================

    public class ParallelPathStep
    {
        public int x;
        public int z;
        public int time;
        public int gCost;
        public int hCost;


        public ParallelPathStep(
            int x,
            int z,
            int time,
            int gCost,
            int hCost)
        {
            this.x = x;
            this.z = z;
            this.time = time;
            this.gCost = gCost;
            this.hCost = hCost;
        }
    }


    public class ParallelReplanResult
    {
        public bool success;

        public string message;

        public int expandedStates;

        public long elapsedMilliseconds;
        public double elapsedMillisecondsPrecise; // full worker compute, sub-ms precision

        public List<ParallelPathStep> steps =
            new List<ParallelPathStep>();
    }


    // ==================================================
    // PURE C# PARALLEL SEARCH DATA
    // ==================================================

    private struct ParallelCoord
    {
        public int x;
        public int z;


        public ParallelCoord(
            int x,
            int z)
        {
            this.x = x;
            this.z = z;
        }
    }


    private class ParallelGridNode
    {
        public int x;
        public int z;

        public List<ParallelCoord> neighbours =
            new List<ParallelCoord>();


        public ParallelGridNode(
            int x,
            int z)
        {
            this.x = x;
            this.z = z;
        }
    }


    private class ParallelGridSnapshot
    {
        public Dictionary<long, ParallelGridNode>
            nodes =
                new Dictionary<long, ParallelGridNode>();
    }


    private struct ParallelVertexConstraintData
    {
        public int x;
        public int z;
        public int time;


        public ParallelVertexConstraintData(
            int x,
            int z,
            int time)
        {
            this.x = x;
            this.z = z;
            this.time = time;
        }
    }


    private struct ParallelEdgeConstraintData
    {
        public int fromX;
        public int fromZ;

        public int toX;
        public int toZ;

        public int time;


        public ParallelEdgeConstraintData(
            int fromX,
            int fromZ,
            int toX,
            int toZ,
            int time)
        {
            this.fromX = fromX;
            this.fromZ = fromZ;

            this.toX = toX;
            this.toZ = toZ;

            this.time = time;
        }
    }


    private class ParallelSearchRequest
    {
        public ParallelGridSnapshot grid;

        public int startX;
        public int startZ;

        public int targetX;
        public int targetZ;

        public List<ParallelVertexConstraintData>
            vertexConstraints =
                new List<ParallelVertexConstraintData>();

        public List<ParallelEdgeConstraintData>
            edgeConstraints =
                new List<ParallelEdgeConstraintData>();

        public int maxTimeStep;

        public int maxExpandedStates;

        public int maxSearchMilliseconds;

        public int moveDurationSlots;

        public string robotLabel;
    }


    private class ParallelSearchNode
    {
        public int x;
        public int z;

        public int time;

        public int gCost;
        public int hCost;

        public ParallelSearchNode parent;


        public int fCost
        {
            get
            {
                return
                    gCost +
                    hCost;
            }
        }


        public ParallelSearchNode(
            int x,
            int z,
            int time,
            int gCost,
            int hCost,
            ParallelSearchNode parent)
        {
            this.x = x;
            this.z = z;

            this.time = time;

            this.gCost = gCost;
            this.hCost = hCost;

            this.parent = parent;
        }
    }


    // ==================================================
    // SEQUENTIAL CBS REPLAN
    //
    // Kept as a fallback / comparison mode.
    // ==================================================

    public List<SpaceTimeNode> ReplanForCBS(
        List<SpaceTimeVertexConstraint> vertexConstraints,
        List<SpaceTimeEdgeConstraint> edgeConstraints)
    {
        if (grid == null)
        {
            grid =
                GetComponent<PathGrid>();
        }


        if (
            robot == null ||
            target == null ||
            grid == null
        )
        {
            Debug.LogError(
                "STA*: Replan failed because references are missing."
            );

            return null;
        }


        Node startNode =
            grid.NodeFromWorldPoint(
                robot.position
            );


        Node targetNode =
            grid.NodeFromWorldPoint(
                target.position
            );


        Debug.Log(
            gameObject.name +
            " | START = (" +
            startNode.gridX + "," +
            startNode.gridZ + ")" +
            " | Walkable = " +
            startNode.walkable
        );


        Debug.Log(
            gameObject.name +
            " | TARGET = (" +
            targetNode.gridX + "," +
            targetNode.gridZ + ")" +
            " | Walkable = " +
            targetNode.walkable
        );


        if (
            !ResolveSpeedTiming(
                startNode,
                false
            )
        )
        {
            return null;
        }


        List<SpaceTimeVertexConstraint>
            previousVertexConstraints =
                activeVertexConstraints;


        List<SpaceTimeEdgeConstraint>
            previousEdgeConstraints =
                activeEdgeConstraints;


        try
        {
            activeVertexConstraints =
                vertexConstraints;

            activeEdgeConstraints =
                edgeConstraints;


            return
                FindPath(
                    startNode,
                    targetNode
                );
        }

        finally
        {
            activeVertexConstraints =
                previousVertexConstraints;

            activeEdgeConstraints =
                previousEdgeConstraints;
        }
    }


    // ==================================================
    // PARALLEL CBS REPLAN
    //
    // This method MUST be called from Unity's main thread.
    //
    // It performs all Unity-dependent work first,
    // creates a pure C# request,
    // then starts the actual STA* search on a worker thread.
    // ==================================================

    public Task<ParallelReplanResult>
        StartParallelReplanForCBS(
            List<SpaceTimeVertexConstraint> vertexConstraints,
            List<SpaceTimeEdgeConstraint> edgeConstraints)
    {
        ParallelSearchRequest request;

        ParallelReplanResult preparationFailure;


        if (
            !TryCreateParallelSearchRequest(
                vertexConstraints,
                edgeConstraints,
                out request,
                out preparationFailure
            )
        )
        {
            return
                Task.FromResult(
                    preparationFailure
                );
        }


        return
            Task.Run(
                () =>
                    RunParallelSearch(
                        request
                    )
            );
    }


    // ==================================================
    // CONVERT WORKER RESULT BACK TO UNITY PATH
    //
    // MUST be called on Unity's main thread.
    // ==================================================

    public List<SpaceTimeNode>
        MaterializeParallelPath(
            ParallelReplanResult result)
    {
        if (
            result == null ||
            !result.success ||
            result.steps == null ||
            result.steps.Count == 0
        )
        {
            return null;
        }


        List<SpaceTimeNode> path =
            new List<SpaceTimeNode>();


        SpaceTimeNode parent =
            null;


        foreach (
            ParallelPathStep step
            in result.steps
        )
        {
            long key =
                GetCoordKey(
                    step.x,
                    step.z
                );


            Node node;


            if (
                !parallelNodeLookup.TryGetValue(
                    key,
                    out node
                )
            )
            {
                Debug.LogError(
                    "Parallel STA*: Could not map grid cell (" +
                    step.x +
                    "," +
                    step.z +
                    ") back to a Unity Node."
                );

                return null;
            }


            SpaceTimeNode state =
                new SpaceTimeNode(
                    node,
                    step.time,
                    step.gCost,
                    step.hCost,
                    parent
                );


            path.Add(
                state
            );


            parent =
                state;
        }


        return path;
    }


    // ==================================================
    // BUILD PARALLEL SEARCH REQUEST
    // ==================================================

    bool TryCreateParallelSearchRequest(
        List<SpaceTimeVertexConstraint> vertexConstraints,
        List<SpaceTimeEdgeConstraint> edgeConstraints,
        out ParallelSearchRequest request,
        out ParallelReplanResult failure)
    {
        request =
            null;

        failure =
            null;


        if (grid == null)
        {
            grid =
                GetComponent<PathGrid>();
        }


        if (
            grid == null ||
            robot == null ||
            target == null
        )
        {
            failure =
                CreateParallelFailure(
                    "missing Unity references",
                    0,
                    0
                );

            return false;
        }


        Node startNode =
            grid.NodeFromWorldPoint(
                robot.position
            );


        Node targetNode =
            grid.NodeFromWorldPoint(
                target.position
            );


        if (
            startNode == null ||
            targetNode == null
        )
        {
            failure =
                CreateParallelFailure(
                    "start or target node is null",
                    0,
                    0
                );

            return false;
        }


        if (
            !startNode.walkable
        )
        {
            failure =
                CreateParallelFailure(
                    "start node is not walkable",
                    0,
                    0
                );

            return false;
        }


        if (
            !targetNode.walkable
        )
        {
            failure =
                CreateParallelFailure(
                    "target node is not walkable",
                    0,
                    0
                );

            return false;
        }


        if (
            !ResolveSpeedTiming(
                startNode,
                false
            )
        )
        {
            failure =
                CreateParallelFailure(
                    "invalid robot speed or global timestep",
                    0,
                    0
                );

            return false;
        }


        if (
            parallelGridSnapshot == null ||
            parallelSnapshotStartX !=
                startNode.gridX ||
            parallelSnapshotStartZ !=
                startNode.gridZ
        )
        {
            if (
                !BuildParallelGridSnapshot(
                    startNode
                )
            )
            {
                failure =
                    CreateParallelFailure(
                        "could not build pure C# grid snapshot",
                        0,
                        0
                    );

                return false;
            }
        }


        long targetKey =
            GetCoordKey(
                targetNode.gridX,
                targetNode.gridZ
            );


        if (
            !parallelGridSnapshot.nodes.ContainsKey(
                targetKey
            )
        )
        {
            failure =
                CreateParallelFailure(
                    "target is not connected to the start " +
                    "in the walkable grid graph",
                    0,
                    0
                );

            return false;
        }


        request =
            new ParallelSearchRequest();


        request.grid =
            parallelGridSnapshot;


        request.startX =
            startNode.gridX;

        request.startZ =
            startNode.gridZ;


        request.targetX =
            targetNode.gridX;

        request.targetZ =
            targetNode.gridZ;


        request.maxTimeStep =
            maxTimeStep;

        request.maxExpandedStates =
            maxExpandedStates;

        request.maxSearchMilliseconds =
            maxSearchMilliseconds;


        request.moveDurationSlots =
            moveDurationSlots;


        request.robotLabel =
            robot.name;


        if (
            vertexConstraints !=
            null
        )
        {
            foreach (
                SpaceTimeVertexConstraint constraint
                in vertexConstraints
            )
            {
                request.vertexConstraints.Add(
                    new ParallelVertexConstraintData(
                        constraint.gridX,
                        constraint.gridZ,
                        constraint.time
                    )
                );
            }
        }


        if (
            edgeConstraints !=
            null
        )
        {
            foreach (
                SpaceTimeEdgeConstraint constraint
                in edgeConstraints
            )
            {
                request.edgeConstraints.Add(
                    new ParallelEdgeConstraintData(
                        constraint.fromX,
                        constraint.fromZ,
                        constraint.toX,
                        constraint.toZ,
                        constraint.time
                    )
                );
            }
        }


        return true;
    }


    // ==================================================
    // BUILD PURE C# GRID SNAPSHOT
    //
    // This is intentionally done on the Unity main thread.
    //
    // Only walkable nodes reachable from this robot's
    // start cell are copied.
    // ==================================================

    bool BuildParallelGridSnapshot(
        Node startNode)
    {
        parallelGridSnapshot =
            new ParallelGridSnapshot();


        parallelNodeLookup.Clear();


        Queue<Node> queue =
            new Queue<Node>();


        HashSet<long> visited =
            new HashSet<long>();


        queue.Enqueue(
            startNode
        );


        while (
            queue.Count > 0
        )
        {
            Node current =
                queue.Dequeue();


            if (
                current == null ||
                !current.walkable
            )
            {
                continue;
            }


            long currentKey =
                GetCoordKey(
                    current.gridX,
                    current.gridZ
                );


            if (
                visited.Contains(
                    currentKey
                )
            )
            {
                continue;
            }


            visited.Add(
                currentKey
            );


            parallelNodeLookup[
                currentKey
            ] =
                current;


            ParallelGridNode copiedNode =
                new ParallelGridNode(
                    current.gridX,
                    current.gridZ
                );


            List<Node> neighbours =
                grid.GetNeighbours(
                    current
                );


            foreach (
                Node neighbour
                in neighbours
            )
            {
                if (
                    neighbour == null ||
                    !neighbour.walkable
                )
                {
                    continue;
                }


                copiedNode.neighbours.Add(
                    new ParallelCoord(
                        neighbour.gridX,
                        neighbour.gridZ
                    )
                );


                long neighbourKey =
                    GetCoordKey(
                        neighbour.gridX,
                        neighbour.gridZ
                    );


                if (
                    !visited.Contains(
                        neighbourKey
                    )
                )
                {
                    queue.Enqueue(
                        neighbour
                    );
                }
            }


            parallelGridSnapshot.nodes[
                currentKey
            ] =
                copiedNode;
        }


        parallelSnapshotStartX =
            startNode.gridX;

        parallelSnapshotStartZ =
            startNode.gridZ;


        return
            parallelGridSnapshot.nodes.Count >
            0;
    }


    // ==================================================
    // PURE C# PARALLEL STA*
    //
    // NO UnityEngine calls are allowed below this point.
    // ==================================================

    static ParallelReplanResult RunParallelSearch(
        ParallelSearchRequest request)
    {
        System.Diagnostics.Stopwatch searchTimer =
            System.Diagnostics.Stopwatch.StartNew();

        List<ParallelSearchNode> openSet =
            new List<ParallelSearchNode>();


        HashSet<string> closedSet =
            new HashSet<string>();


        Dictionary<string, int> bestG =
            new Dictionary<string, int>();


        int expandedStates =
            0;


        // ==================================================
        // START VALIDATION
        // ==================================================

        if (
            ParallelViolatesVertexConstraint(
                request,
                request.startX,
                request.startZ,
                0
            )
        )
        {
            searchTimer.Stop();


            return
                CreateParallelFailure(
                    "start node violates a vertex constraint",
                    expandedStates,
                    searchTimer.Elapsed.TotalMilliseconds
                );
        }


        ParallelSearchNode startState =
            new ParallelSearchNode(
                request.startX,
                request.startZ,
                0,
                0,
                ParallelGetHeuristic(
                    request.startX,
                    request.startZ,
                    request.targetX,
                    request.targetZ,
                    request.moveDurationSlots
                ),
                null
            );


        openSet.Add(
            startState
        );


        bestG[
            ParallelGetStateKey(
                request.startX,
                request.startZ,
                0
            )
        ] =
            0;


        // ==================================================
        // SEARCH
        // ==================================================

        while (
            openSet.Count > 0
        )
        {
            // ==================================================
            // REAL-TIME SEARCH LIMIT
            // ==================================================

            if (
                request.maxSearchMilliseconds > 0 &&
                searchTimer.ElapsedMilliseconds >=
                    request.maxSearchMilliseconds
            )
            {
                searchTimer.Stop();


                return
                    CreateParallelFailure(
                        "search timeout",
                        expandedStates,
                        searchTimer.Elapsed.TotalMilliseconds
                    );
            }


            // ==================================================
            // EXPANDED STATE LIMIT
            // ==================================================

            if (
                request.maxExpandedStates > 0 &&
                expandedStates >=
                    request.maxExpandedStates
            )
            {
                searchTimer.Stop();


                return
                    CreateParallelFailure(
                        "expanded-state limit",
                        expandedStates,
                        searchTimer.Elapsed.TotalMilliseconds
                    );
            }


            ParallelSearchNode current =
                ParallelGetBestState(
                    openSet
                );


            openSet.Remove(
                current
            );


            string currentKey =
                ParallelGetStateKey(
                    current.x,
                    current.z,
                    current.time
                );


            if (
                closedSet.Contains(
                    currentKey
                )
            )
            {
                continue;
            }


            closedSet.Add(
                currentKey
            );


            expandedStates++;


            // ==================================================
            // TIME HORIZON
            // ==================================================

            if (
                request.maxTimeStep > 0 &&
                current.time >
                    request.maxTimeStep
            )
            {
                continue;
            }


            // ==================================================
            // GOAL
            // ==================================================

            if (
                current.x ==
                    request.targetX &&
                current.z ==
                    request.targetZ
            )
            {
                if (
                    ParallelGoalIsSafeFromNowOn(
                        request,
                        current.time
                    )
                )
                {
                    ParallelReplanResult success =
                        new ParallelReplanResult();


                    success.success =
                        true;

                    success.message =
                        "success";

                    success.expandedStates =
                        expandedStates;

                    success.steps =
                        ParallelRetracePath(
                            current,
                            request
                        );

                    // Stop AFTER path reconstruction, same boundary as sequential STA*.
                    searchTimer.Stop();
                    success.elapsedMilliseconds = searchTimer.ElapsedMilliseconds;
                    success.elapsedMillisecondsPrecise =
                        searchTimer.Elapsed.TotalMilliseconds;

                    return success;
                }
            }


            // ==================================================
            // DO NOT EXPAND PAST MAX TIME
            // ==================================================

            if (
                request.maxTimeStep > 0 &&
                current.time >=
                    request.maxTimeStep
            )
            {
                continue;
            }


            int waitNextTime =
                current.time + 1;


            int moveNextTime =
                current.time +
                request.moveDurationSlots;


            // ==================================================
            // MOVEMENT
            // ==================================================

            ParallelGridNode currentGridNode;


            long currentCoordKey =
                GetCoordKey(
                    current.x,
                    current.z
                );


            if (
                request.grid.nodes.TryGetValue(
                    currentCoordKey,
                    out currentGridNode
                )
            )
            {
                foreach (
                    ParallelCoord neighbour
                    in currentGridNode.neighbours
                )
                {
                    ParallelTryAddState(
                        request,
                        current,
                        neighbour.x,
                        neighbour.z,
                        moveNextTime,
                        openSet,
                        closedSet,
                        bestG
                    );
                }
            }


            // ==================================================
            // WAIT
            // ==================================================

            ParallelTryAddState(
                request,
                current,
                current.x,
                current.z,
                waitNextTime,
                openSet,
                closedSet,
                bestG
            );
        }


        searchTimer.Stop();


        return
            CreateParallelFailure(
                "OPEN exhausted - no path",
                expandedStates,
                searchTimer.Elapsed.TotalMilliseconds
            );
    }


    // ==================================================
    // PURE C# TRY ADD STATE
    // ==================================================

    static void ParallelTryAddState(
        ParallelSearchRequest request,
        ParallelSearchNode current,
        int nextX,
        int nextZ,
        int nextTime,
        List<ParallelSearchNode> openSet,
        HashSet<string> closedSet,
        Dictionary<string, int> bestG)
    {
        if (
            request.maxTimeStep > 0 &&
            nextTime >
                request.maxTimeStep
        )
        {
            return;
        }


        bool isMove =
            current.x != nextX ||
            current.z != nextZ;


        // ==================================================
        // SPEED-AWARE INTERMEDIATE OCCUPANCY
        //
        // For a move taking multiple slots, the discrete
        // reservation stays on the source cell until the
        // final source -> destination transition.
        // ==================================================

        if (isMove)
        {
            for (
                int t = current.time + 1;
                t < nextTime;
                t++
            )
            {
                if (
                    ParallelViolatesVertexConstraint(
                        request,
                        current.x,
                        current.z,
                        t
                    )
                )
                {
                    return;
                }
            }
        }


        // ==================================================
        // VERTEX CONSTRAINT AT ARRIVAL / WAIT SLOT
        // ==================================================

        if (
            ParallelViolatesVertexConstraint(
                request,
                nextX,
                nextZ,
                nextTime
            )
        )
        {
            return;
        }


        // ==================================================
        // EDGE CONSTRAINT
        //
        // The visible grid-edge transition happens during
        // nextTime - 1 -> nextTime.
        // ==================================================

        if (
            isMove &&
            ParallelViolatesEdgeConstraint(
                request,
                current.x,
                current.z,
                nextX,
                nextZ,
                nextTime - 1
            )
        )
        {
            return;
        }


        string key =
            ParallelGetStateKey(
                nextX,
                nextZ,
                nextTime
            );


        if (
            closedSet.Contains(
                key
            )
        )
        {
            return;
        }


        int durationSlots =
            nextTime -
            current.time;


        int newG =
            current.gCost +
            (
                10 *
                durationSlots
            );


        if (
            bestG.ContainsKey(
                key
            ) &&
            bestG[key] <=
                newG
        )
        {
            return;
        }


        bestG[key] =
            newG;


        int newH =
            ParallelGetHeuristic(
                nextX,
                nextZ,
                request.targetX,
                request.targetZ,
                request.moveDurationSlots
            );


        ParallelSearchNode newState =
            new ParallelSearchNode(
                nextX,
                nextZ,
                nextTime,
                newG,
                newH,
                current
            );


        openSet.Add(
            newState
        );
    }


    // ==================================================
    // PURE C# CONSTRAINT CHECKS
    // ==================================================

    static bool ParallelViolatesVertexConstraint(
        ParallelSearchRequest request,
        int x,
        int z,
        int time)
    {
        foreach (
            ParallelVertexConstraintData constraint
            in request.vertexConstraints
        )
        {
            if (
                x ==
                    constraint.x &&
                z ==
                    constraint.z &&
                time ==
                    constraint.time
            )
            {
                return true;
            }
        }


        return false;
    }


    static bool ParallelViolatesEdgeConstraint(
        ParallelSearchRequest request,
        int fromX,
        int fromZ,
        int toX,
        int toZ,
        int time)
    {
        foreach (
            ParallelEdgeConstraintData constraint
            in request.edgeConstraints
        )
        {
            if (
                fromX ==
                    constraint.fromX &&
                fromZ ==
                    constraint.fromZ &&

                toX ==
                    constraint.toX &&
                toZ ==
                    constraint.toZ &&

                time ==
                    constraint.time
            )
            {
                return true;
            }
        }


        return false;
    }


    static bool ParallelGoalIsSafeFromNowOn(
        ParallelSearchRequest request,
        int arrivalTime)
    {
        foreach (
            ParallelVertexConstraintData constraint
            in request.vertexConstraints
        )
        {
            bool sameGoal =
                request.targetX ==
                    constraint.x &&
                request.targetZ ==
                    constraint.z;


            bool constraintNowOrLater =
                constraint.time >=
                    arrivalTime;


            if (
                sameGoal &&
                constraintNowOrLater
            )
            {
                return false;
            }
        }


        return true;
    }


    // ==================================================
    // PURE C# SELECT BEST STATE
    // ==================================================

    static ParallelSearchNode ParallelGetBestState(
        List<ParallelSearchNode> openSet)
    {
        ParallelSearchNode best =
            openSet[0];


        for (
            int i = 1;
            i < openSet.Count;
            i++
        )
        {
            ParallelSearchNode candidate =
                openSet[i];


            if (
                candidate.fCost <
                    best.fCost
            )
            {
                best =
                    candidate;
            }

            else if (
                candidate.fCost ==
                    best.fCost &&

                candidate.hCost <
                    best.hCost
            )
            {
                best =
                    candidate;
            }
        }


        return best;
    }


    // ==================================================
    // PURE C# HEURISTIC
    // ==================================================

    static int ParallelGetHeuristic(
        int xA,
        int zA,
        int xB,
        int zB,
        int moveSlots)
    {
        int dx =
            System.Math.Abs(
                xA -
                xB
            );


        int dz =
            System.Math.Abs(
                zA -
                zB
            );


        return
            10 *
            moveSlots *
            (dx + dz);
    }


    // ==================================================
    // PURE C# STATE KEY
    // ==================================================

    static string ParallelGetStateKey(
        int x,
        int z,
        int time)
    {
        return
            x +
            "," +
            z +
            "," +
            time;
    }


    // ==================================================
    // PURE C# RETRACE
    // ==================================================

    static List<ParallelPathStep>
        ParallelRetracePath(
            ParallelSearchNode goalState,
            ParallelSearchRequest request)
    {
        List<ParallelSearchNode> sparsePath =
            new List<ParallelSearchNode>();


        ParallelSearchNode current =
            goalState;


        while (
            current !=
            null
        )
        {
            sparsePath.Add(
                current
            );


            current =
                current.parent;
        }


        sparsePath.Reverse();


        List<ParallelPathStep> expandedPath =
            new List<ParallelPathStep>();


        if (
            sparsePath.Count ==
            0
        )
        {
            return expandedPath;
        }


        ParallelSearchNode first =
            sparsePath[0];


        expandedPath.Add(
            new ParallelPathStep(
                first.x,
                first.z,
                first.time,
                first.gCost,
                first.hCost
            )
        );


        for (
            int i = 1;
            i < sparsePath.Count;
            i++
        )
        {
            ParallelSearchNode previous =
                sparsePath[i - 1];


            ParallelSearchNode next =
                sparsePath[i];


            for (
                int t = previous.time + 1;
                t < next.time;
                t++
            )
            {
                int intermediateG =
                    previous.gCost +
                    (
                        10 *
                        (t - previous.time)
                    );


                int intermediateH =
                    ParallelGetHeuristic(
                        previous.x,
                        previous.z,
                        request.targetX,
                        request.targetZ,
                        request.moveDurationSlots
                    );


                expandedPath.Add(
                    new ParallelPathStep(
                        previous.x,
                        previous.z,
                        t,
                        intermediateG,
                        intermediateH
                    )
                );
            }


            expandedPath.Add(
                new ParallelPathStep(
                    next.x,
                    next.z,
                    next.time,
                    next.gCost,
                    next.hCost
                )
            );
        }


        return expandedPath;
    }


    // ==================================================
    // PARALLEL FAILURE RESULT
    // ==================================================

    static ParallelReplanResult CreateParallelFailure(
        string message,
        int expandedStates,
        double elapsedMilliseconds)
    {
        ParallelReplanResult result =
            new ParallelReplanResult();


        result.success =
            false;

        result.message =
            message;

        result.expandedStates =
            expandedStates;

        result.elapsedMilliseconds =
            (long)elapsedMilliseconds;
        result.elapsedMillisecondsPrecise = elapsedMilliseconds;


        return result;
    }


    // ==================================================
    // COORDINATE KEY
    // ==================================================

    static long GetCoordKey(
        int x,
        int z)
    {
        return
            ((long)x << 32) ^
            (uint)z;
    }


    // ==================================================
    // RESOLVE SPEED -> GLOBAL TIME SLOTS
    // ==================================================

    bool ResolveSpeedTiming(
        Node startNode,
        bool printSummary)
    {
        if (
            robot ==
            null
        )
        {
            Debug.LogError(
                "STA*: Cannot resolve speed because robot is null."
            );

            return false;
        }


        RobotMovement movement =
            robot.GetComponent<RobotMovement>();


        if (
            movement ==
            null
        )
        {
            Debug.LogError(
                "STA*: " +
                robot.name +
                " requires RobotMovement for speed-aware STA*."
            );

            return false;
        }


        if (
            movement.moveSpeed <=
            0f
        )
        {
            Debug.LogError(
                "STA*: " +
                robot.name +
                " moveSpeed must be greater than 0."
            );

            return false;
        }


        if (
            movement.timePerStep <=
            0f
        )
        {
            Debug.LogError(
                "STA*: " +
                robot.name +
                " timePerStep must be greater than 0."
            );

            return false;
        }


        resolvedRobotSpeed =
            movement.moveSpeed;


        resolvedBaseTimeStep =
            movement.timePerStep;


        resolvedGridCellDistance =
            EstimateGridCellDistance(
                startNode
            );


        float exactTravelTime =
            resolvedGridCellDistance /
            resolvedRobotSpeed;


        moveDurationSlots =
            Mathf.Max(
                1,
                Mathf.CeilToInt(
                    exactTravelTime /
                    resolvedBaseTimeStep
                )
            );


        if (printSummary)
        {
            Debug.Log(
                robot.name +
                " | SPEED-AWARE STA*" +
                " | Speed = " +
                resolvedRobotSpeed.ToString("F2") +
                " units/s" +
                " | Cell = " +
                resolvedGridCellDistance.ToString("F3") +
                " units" +
                " | Base dt = " +
                resolvedBaseTimeStep.ToString("F3") +
                " s" +
                " | Move Slots = " +
                moveDurationSlots +
                " | Scheduled Edge Time = " +
                (
                    moveDurationSlots *
                    resolvedBaseTimeStep
                ).ToString("F3") +
                " s"
            );
        }


        return true;
    }


    // ==================================================
    // ESTIMATE GRID EDGE DISTANCE
    // ==================================================

    float EstimateGridCellDistance(
        Node startNode)
    {
        if (
            grid != null &&
            startNode != null
        )
        {
            List<Node> neighbours =
                grid.GetNeighbours(
                    startNode
                );


            foreach (
                Node neighbour
                in neighbours
            )
            {
                if (
                    neighbour ==
                    null
                )
                {
                    continue;
                }


                float distance =
                    Vector3.Distance(
                        startNode.worldPosition,
                        neighbour.worldPosition
                    );


                if (
                    distance >
                    0.0001f
                )
                {
                    return distance;
                }
            }
        }


        return
            Mathf.Max(
                0.0001f,
                fallbackGridCellDistance
            );
    }


    public int GetMoveDurationSlots()
    {
        return
            Mathf.Max(
                1,
                moveDurationSlots
            );
    }


    public float GetBaseTimeStepSeconds()
    {
        return
            resolvedBaseTimeStep;
    }


    public float GetResolvedRobotSpeed()
    {
        return
            resolvedRobotSpeed;
    }


    // ==================================================
    // START
    // ==================================================

    void Start()
    {
        // ==================================================
        // GET GRID
        // ==================================================

        grid =
            GetComponent<PathGrid>();


        // ==================================================
        // REFERENCE CHECKS
        // ==================================================

        if (grid == null)
        {
            Debug.LogError(
                "SpaceTimeAStar requires PathGrid " +
                "on the same GameObject."
            );

            return;
        }


        if (robot == null)
        {
            Debug.LogError(
                "SpaceTimeAStar: Robot is not assigned."
            );

            return;
        }


        if (target == null)
        {
            Debug.LogError(
                "SpaceTimeAStar: Target is not assigned."
            );

            return;
        }


        // ==================================================
        // START / TARGET NODES
        // ==================================================

        Node startNode =
            grid.NodeFromWorldPoint(
                robot.position
            );


        Node targetNode =
            grid.NodeFromWorldPoint(
                target.position
            );


        // ==================================================
        // DEBUG START / TARGET NODE
        // ==================================================

        Debug.Log(
            robot.name +
            " | START = (" +
            startNode.gridX + "," +
            startNode.gridZ + ")" +
            " | Walkable = " +
            startNode.walkable
        );


        Debug.Log(
            robot.name +
            " | TARGET = (" +
            targetNode.gridX + "," +
            targetNode.gridZ + ")" +
            " | Walkable = " +
            targetNode.walkable
        );


        if (
            !ResolveSpeedTiming(
                startNode,
                true
            )
        )
        {
            return;
        }


        // ==================================================
        // CLEAR OLD TEST CONSTRAINTS
        // ==================================================

        activeVertexConstraints.Clear();

        activeEdgeConstraints.Clear();


        // ==================================================
        // OPTIONAL VERTEX TEST
        // ==================================================

        if (useTestVertexConstraint)
        {
            activeVertexConstraints.Add(
                new SpaceTimeVertexConstraint(
                    targetNode.gridX,
                    targetNode.gridZ,
                    testVertexConstraintTime
                )
            );


            Debug.Log(
                "TEST VERTEX CONSTRAINT: Cannot occupy (" +
                targetNode.gridX +
                "," +
                targetNode.gridZ +
                ") at t=" +
                testVertexConstraintTime
            );
        }


        // ==================================================
        // OPTIONAL EDGE TEST
        // ==================================================

        if (useTestEdgeConstraint)
        {
            activeEdgeConstraints.Add(
                new SpaceTimeEdgeConstraint(
                    testEdgeFromX,
                    testEdgeFromZ,

                    testEdgeToX,
                    testEdgeToZ,

                    testEdgeTime
                )
            );


            Debug.Log(
                "TEST EDGE CONSTRAINT: (" +
                testEdgeFromX +
                "," +
                testEdgeFromZ +
                ") -> (" +
                testEdgeToX +
                "," +
                testEdgeToZ +
                ") during t=" +
                testEdgeTime +
                " -> " +
                (testEdgeTime + 1)
            );
        }


        // ==================================================
        // INITIAL INDEPENDENT STA*
        // ==================================================

        // Comparable search-work boundary: no Unity Debug.Log/path printing here.
        System.Diagnostics.Stopwatch initialPlannerTimer =
            System.Diagnostics.Stopwatch.StartNew();

        finalPath =
            FindPath(
                startNode,
                targetNode
            );

        initialPlannerTimer.Stop();
        initialPlanningMilliseconds =
            initialPlannerTimer.Elapsed.TotalMilliseconds;

        Debug.Log(
            robot.name +
            " | Initial STA* Planning Time = " +
            initialPlanningMilliseconds.ToString("F3") +
            " ms"
        );

        if (
            finalPath == null ||
            finalPath.Count == 0
        )
        {
            Debug.LogError(
                "STA*: No initial path found for " +
                robot.name
            );

            return;
        }


        // ==================================================
        // COPY TO ROBOTPATH
        // ==================================================

        List<Node> spatialPath =
            new List<Node>();


        foreach (
            SpaceTimeNode state
            in finalPath
        )
        {
            spatialPath.Add(
                state.node
            );
        }


        RobotPath robotPath =
            robot.GetComponent<RobotPath>();


        if (robotPath == null)
        {
            Debug.LogError(
                robot.name +
                " does not have a RobotPath component."
            );

            return;
        }


        robotPath.path =
            spatialPath;


        // Initial STA* path is NOT yet approved by CBS / PCBS.

        robotPath.routeFinalized =
            false;


        PrintSpaceTimePath();
    }


    // ==================================================
    // SPACE-TIME A*
    //
    // Sequential version used for initial paths
    // and optional fallback CBS testing.
    // ==================================================

    List<SpaceTimeNode> FindPath(
        Node startNode,
        Node targetNode)
    {
        List<SpaceTimeNode> openSet =
            new List<SpaceTimeNode>();


        HashSet<string> closedSet =
            new HashSet<string>();


        Dictionary<string, int> bestG =
            new Dictionary<string, int>();


        // ==================================================
        // START VALIDATION
        // ==================================================

        if (
            ViolatesVertexConstraint(
                startNode,
                0
            )
        )
        {
            Debug.LogWarning(
                "STA*: Start node violates a vertex constraint."
            );

            return null;
        }


        SpaceTimeNode startState =
            new SpaceTimeNode(
                startNode,
                0,
                0,

                GetHeuristic(
                    startNode,
                    targetNode
                ),

                null
            );


        openSet.Add(
            startState
        );


        bestG[
            GetStateKey(
                startNode,
                0
            )
        ] = 0;


        // ==================================================
        // FREEZE PROTECTION
        // ==================================================

        int expandedStates =
            0;


        System.Diagnostics.Stopwatch
            searchTimer =
                System.Diagnostics.Stopwatch.StartNew();


        // ==================================================
        // SEARCH
        // ==================================================

        while (
            openSet.Count > 0
        )
        {
            // ==================================================
            // REAL-TIME SEARCH LIMIT
            // ==================================================

            if (
                maxSearchMilliseconds > 0 &&
                searchTimer.ElapsedMilliseconds >=
                    maxSearchMilliseconds
            )
            {
                Debug.LogWarning(
                    "STA* ABORTED | Robot = " +
                    robot.name +
                    " | Reason = search timeout" +
                    " | Expanded = " +
                    expandedStates +
                    " | Limit = " +
                    maxSearchMilliseconds +
                    " ms"
                );

                return null;
            }


            // ==================================================
            // EXPANDED STATE LIMIT
            // ==================================================

            if (
                maxExpandedStates > 0 &&
                expandedStates >=
                    maxExpandedStates
            )
            {
                Debug.LogWarning(
                    "STA* ABORTED | Robot = " +
                    robot.name +
                    " | Reason = expanded-state limit" +
                    " | Expanded = " +
                    expandedStates
                );

                return null;
            }


            SpaceTimeNode current =
                GetBestState(
                    openSet
                );


            openSet.Remove(
                current
            );


            string currentKey =
                GetStateKey(
                    current.node,
                    current.time
                );


            if (
                closedSet.Contains(
                    currentKey
                )
            )
            {
                continue;
            }


            closedSet.Add(
                currentKey
            );


            expandedStates++;


            // ==================================================
            // TIME HORIZON LIMIT
            // ==================================================

            if (
                maxTimeStep > 0 &&
                current.time >
                    maxTimeStep
            )
            {
                continue;
            }


            // ==================================================
            // GOAL FOUND
            //
            // Robot remains at goal forever.
            // Therefore the goal must also be safe
            // from all future vertex constraints.
            // ==================================================

            if (
                current.node ==
                    targetNode
            )
            {
                if (
                    GoalIsSafeFromNowOn(
                        targetNode,
                        current.time
                    )
                )
                {
                    searchTimer.Stop();


                    return
                        RetracePath(
                            current
                        );
                }
            }


            // ==================================================
            // DO NOT GENERATE STATES PAST MAX TIME
            // ==================================================

            if (
                maxTimeStep > 0 &&
                current.time >=
                    maxTimeStep
            )
            {
                continue;
            }


            int waitNextTime =
                current.time +
                1;


            int moveNextTime =
                current.time +
                moveDurationSlots;


            // ==================================================
            // MOVEMENT
            // ==================================================

            foreach (
                Node neighbour
                in grid.GetNeighbours(
                    current.node
                )
            )
            {
                if (
                    !neighbour.walkable
                )
                {
                    continue;
                }


                TryAddState(
                    current,
                    neighbour,
                    moveNextTime,
                    targetNode,
                    openSet,
                    closedSet,
                    bestG
                );
            }


            // ==================================================
            // WAIT
            // ==================================================

            TryAddState(
                current,
                current.node,
                waitNextTime,
                targetNode,
                openSet,
                closedSet,
                bestG
            );
        }


        searchTimer.Stop();


        return null;
    }


    // ==================================================
    // GOAL SAFETY
    // ==================================================

    bool GoalIsSafeFromNowOn(
        Node targetNode,
        int arrivalTime)
    {
        foreach (
            SpaceTimeVertexConstraint constraint
            in activeVertexConstraints
        )
        {
            bool sameGoal =
                targetNode.gridX ==
                    constraint.gridX &&

                targetNode.gridZ ==
                    constraint.gridZ;


            bool constraintNowOrLater =
                constraint.time >=
                    arrivalTime;


            if (
                sameGoal &&
                constraintNowOrLater
            )
            {
                return false;
            }
        }


        return true;
    }


    // ==================================================
    // TRY ADD STATE
    // ==================================================

    void TryAddState(
        SpaceTimeNode current,
        Node nextNode,
        int nextTime,
        Node targetNode,
        List<SpaceTimeNode> openSet,
        HashSet<string> closedSet,
        Dictionary<string, int> bestG)
    {
        if (
            maxTimeStep > 0 &&
            nextTime >
                maxTimeStep
        )
        {
            return;
        }


        bool isMove =
            current.node !=
            nextNode;


        // ==================================================
        // SPEED-AWARE INTERMEDIATE OCCUPANCY
        // ==================================================

        if (isMove)
        {
            for (
                int t = current.time + 1;
                t < nextTime;
                t++
            )
            {
                if (
                    ViolatesVertexConstraint(
                        current.node,
                        t
                    )
                )
                {
                    return;
                }
            }
        }


        // ==================================================
        // VERTEX CONSTRAINT AT ARRIVAL / WAIT SLOT
        // ==================================================

        if (
            ViolatesVertexConstraint(
                nextNode,
                nextTime
            )
        )
        {
            return;
        }


        // ==================================================
        // EDGE CONSTRAINT
        // ==================================================

        if (
            isMove &&
            ViolatesEdgeConstraint(
                current.node,
                nextNode,
                nextTime - 1
            )
        )
        {
            return;
        }


        string key =
            GetStateKey(
                nextNode,
                nextTime
            );


        if (
            closedSet.Contains(
                key
            )
        )
        {
            return;
        }


        // ==================================================
        // COST
        //
        // Move = 10
        // Wait = 10
        // ==================================================

        int durationSlots =
            nextTime -
            current.time;


        int newG =
            current.gCost +
            (
                10 *
                durationSlots
            );


        if (
            bestG.ContainsKey(
                key
            ) &&
            bestG[key] <=
                newG
        )
        {
            return;
        }


        bestG[key] =
            newG;


        int newH =
            GetHeuristic(
                nextNode,
                targetNode
            );


        SpaceTimeNode newState =
            new SpaceTimeNode(
                nextNode,
                nextTime,
                newG,
                newH,
                current
            );


        openSet.Add(
            newState
        );
    }


    // ==================================================
    // VERTEX CONSTRAINT CHECK
    // ==================================================

    bool ViolatesVertexConstraint(
        Node node,
        int time)
    {
        foreach (
            SpaceTimeVertexConstraint constraint
            in activeVertexConstraints
        )
        {
            if (
                node.gridX ==
                    constraint.gridX &&

                node.gridZ ==
                    constraint.gridZ &&

                time ==
                    constraint.time
            )
            {
                return true;
            }
        }


        return false;
    }


    // ==================================================
    // EDGE CONSTRAINT CHECK
    // ==================================================

    bool ViolatesEdgeConstraint(
        Node fromNode,
        Node toNode,
        int time)
    {
        foreach (
            SpaceTimeEdgeConstraint constraint
            in activeEdgeConstraints
        )
        {
            if (
                fromNode.gridX ==
                    constraint.fromX &&

                fromNode.gridZ ==
                    constraint.fromZ &&

                toNode.gridX ==
                    constraint.toX &&

                toNode.gridZ ==
                    constraint.toZ &&

                time ==
                    constraint.time
            )
            {
                return true;
            }
        }


        return false;
    }


    // ==================================================
    // SELECT BEST STATE
    // ==================================================

    SpaceTimeNode GetBestState(
        List<SpaceTimeNode> openSet)
    {
        SpaceTimeNode best =
            openSet[0];


        for (
            int i = 1;
            i < openSet.Count;
            i++
        )
        {
            SpaceTimeNode candidate =
                openSet[i];


            if (
                candidate.fCost <
                    best.fCost
            )
            {
                best =
                    candidate;
            }

            else if (
                candidate.fCost ==
                    best.fCost &&

                candidate.hCost <
                    best.hCost
            )
            {
                best =
                    candidate;
            }
        }


        return best;
    }


    // ==================================================
    // MANHATTAN HEURISTIC
    // ==================================================

    int GetHeuristic(
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
            moveDurationSlots *
            (dx + dz);
    }


    // ==================================================
    // STATE KEY
    // ==================================================

    string GetStateKey(
        Node node,
        int time)
    {
        return
            node.gridX +
            "," +
            node.gridZ +
            "," +
            time;
    }


    // ==================================================
    // RETRACE
    // ==================================================

    List<SpaceTimeNode> RetracePath(
        SpaceTimeNode goalState)
    {
        List<SpaceTimeNode> sparsePath =
            new List<SpaceTimeNode>();


        SpaceTimeNode current =
            goalState;


        while (
            current !=
            null
        )
        {
            sparsePath.Add(
                current
            );


            current =
                current.parent;
        }


        sparsePath.Reverse();


        List<SpaceTimeNode> expandedPath =
            new List<SpaceTimeNode>();


        if (
            sparsePath.Count ==
            0
        )
        {
            return expandedPath;
        }


        SpaceTimeNode expandedParent =
            null;


        SpaceTimeNode firstSparse =
            sparsePath[0];


        SpaceTimeNode firstExpanded =
            new SpaceTimeNode(
                firstSparse.node,
                firstSparse.time,
                firstSparse.gCost,
                firstSparse.hCost,
                null
            );


        expandedPath.Add(
            firstExpanded
        );


        expandedParent =
            firstExpanded;


        for (
            int i = 1;
            i < sparsePath.Count;
            i++
        )
        {
            SpaceTimeNode previous =
                sparsePath[i - 1];


            SpaceTimeNode next =
                sparsePath[i];


            for (
                int t = previous.time + 1;
                t < next.time;
                t++
            )
            {
                int intermediateG =
                    previous.gCost +
                    (
                        10 *
                        (t - previous.time)
                    );


                SpaceTimeNode intermediate =
                    new SpaceTimeNode(
                        previous.node,
                        t,
                        intermediateG,
                        previous.hCost,
                        expandedParent
                    );


                expandedPath.Add(
                    intermediate
                );


                expandedParent =
                    intermediate;
            }


            SpaceTimeNode expandedNext =
                new SpaceTimeNode(
                    next.node,
                    next.time,
                    next.gCost,
                    next.hCost,
                    expandedParent
                );


            expandedPath.Add(
                expandedNext
            );


            expandedParent =
                expandedNext;
        }


        return expandedPath;
    }


    // ==================================================
    // PRINT
    // ==================================================

    void PrintSpaceTimePath()
    {
        Debug.Log(
            "===== STA* PATH ====="
        );


        for (
            int i = 0;
            i < finalPath.Count;
            i++
        )
        {
            SpaceTimeNode state =
                finalPath[i];


            string action =
                "";


            if (
                i > 0 &&
                finalPath[i - 1].node ==
                    state.node
            )
            {
                action =
                    " | HOLD";
            }


            Debug.Log(
                "t=" +
                state.time +

                " | (" +
                state.node.gridX +
                "," +
                state.node.gridZ +
                ")" +

                " | g=" +
                state.gCost +

                " | h=" +
                state.hCost +

                " | f=" +
                state.fCost +

                action
            );
        }


        Debug.Log(
            "STA* finished | Global Timesteps = " +
            (finalPath.Count - 1)
        );
    }
}
