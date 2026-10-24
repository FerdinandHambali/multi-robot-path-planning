using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;

public class CBSController : MonoBehaviour
{
    // ==================================================
    // PLANNERS
    // ==================================================

    private SpaceTimeAStar[] planners;


    // ==================================================
    // OPEN LIST
    // ==================================================

    private List<CBSNode> openList =
        new List<CBSNode>();


    private int expandedNodeCount =
        0;


    // ==================================================
    // PCBS PARALLEL SETTINGS
    // ==================================================

    [Header("PCBS Parallel Low-Level Search")]

    [Tooltip(
        "ON = run Child A and Child B STA* replans in parallel. " +
        "OFF = use the old sequential CBS behaviour."
    )]
    public bool useParallelLowLevelReplanning = true;

    [Header("Fair Timing Experiment")]
    [Tooltip("ON: suppress detailed CBS logs INSIDE the timing measurement. Prints timing summary after the stopwatch stops.")]
    public bool fairBenchmarkMode = true;


    // ==================================================
    // FREEZE PROTECTION
    // ==================================================

    [Header("CBS Freeze Protection")]

    [Tooltip(
        "Maximum CT nodes CBS may expand."
    )]
    public int maxCTNodeExpansions = 2000;


    [Tooltip(
        "Maximum total real-time duration of CBS."
    )]
    public float maxCBSSearchSeconds = 15f;


    [Tooltip(
        "Maximum time CBS waits for initial STA* paths."
    )]
    public float initialPathWaitSeconds = 5f;


    // ==================================================
    // START
    // ==================================================

    IEnumerator Start()
    {
        planners =
            GetComponents<SpaceTimeAStar>();


        if (
            planners == null ||
            planners.Length < 2
        )
        {
            Debug.LogError(
                "CBSController requires at least " +
                "two SpaceTimeAStar components."
            );

            yield break;
        }


        // ==================================================
        // LOCK ALL ROBOTS
        // ==================================================

        foreach (
            SpaceTimeAStar planner
            in planners
        )
        {
            if (
                planner.robot ==
                    null
            )
            {
                continue;
            }


            RobotPath robotPath =
                planner.robot
                    .GetComponent<RobotPath>();


            if (
                robotPath !=
                    null
            )
            {
                robotPath.routeFinalized =
                    false;
            }
        }


        // ==================================================
        // CHECK REFERENCES
        // ==================================================

        for (
            int i = 0;
            i < planners.Length;
            i++
        )
        {
            if (
                planners[i].robot == null ||
                planners[i].target == null
            )
            {
                Debug.LogError(
                    "CBS ABORTED: Planner " +
                    (i + 1) +
                    " is missing its robot or target."
                );

                yield break;
            }
        }


        // ==================================================
        // CHECK IMPOSSIBLE SHARED GOALS
        //
        // Our model assumes robots remain at
        // their destination forever.
        //
        // Therefore two robots cannot share
        // the same final grid cell.
        // ==================================================

        if (
            HasDuplicateGoals()
        )
        {
            yield break;
        }


        // ==================================================
        // WAIT FOR INITIAL STA* PATHS
        // ==================================================

        float waitStart =
            Time.realtimeSinceStartup;


        while (true)
        {
            bool allReady =
                true;


            foreach (
                SpaceTimeAStar planner
                in planners
            )
            {
                if (
                    planner.finalPath == null ||
                    planner.finalPath.Count == 0
                )
                {
                    allReady =
                        false;

                    break;
                }
            }


            if (
                allReady
            )
            {
                break;
            }


            float waited =
                Time.realtimeSinceStartup -
                waitStart;


            if (
                initialPathWaitSeconds > 0f &&
                waited >=
                    initialPathWaitSeconds
            )
            {
                Debug.LogError(
                    "===== CBS ABORTED =====\n" +
                    "Reason: Initial STA* paths " +
                    "were not generated successfully.\n" +
                    "One or more robots may have " +
                    "an unreachable destination."
                );

                yield break;
            }


            yield return null;
        }


        // ==================================================
        // RUN CBS / PCBS
        // ==================================================

        yield return
            StartCoroutine(
                RunCBS()
            );
    }


    // ==================================================
    // CHECK DUPLICATE GOALS
    // ==================================================

    bool HasDuplicateGoals()
    {
        PathGrid grid =
            GetComponent<PathGrid>();


        if (
            grid == null
        )
        {
            Debug.LogError(
                "CBS ABORTED: PathGrid not found."
            );

            return true;
        }


        for (
            int i = 0;
            i < planners.Length;
            i++
        )
        {
            Node goalA =
                grid.NodeFromWorldPoint(
                    planners[i]
                        .target
                        .position
                );


            for (
                int j = i + 1;
                j < planners.Length;
                j++
            )
            {
                Node goalB =
                    grid.NodeFromWorldPoint(
                        planners[j]
                            .target
                            .position
                    );


                if (
                    goalA ==
                        goalB
                )
                {
                    Debug.LogError(
                        "===== CBS ABORTED =====\n" +
                        "Reason: Duplicate final goal.\n" +
                        "Robot " +
                        (i + 1) +
                        " and Robot " +
                        (j + 1) +
                        " both use grid cell (" +
                        goalA.gridX +
                        "," +
                        goalA.gridZ +
                        ").\n" +
                        "Robots remain at their goal " +
                        "after arrival, so this scenario " +
                        "has no conflict-free solution."
                    );


                    return true;
                }
            }
        }


        return false;
    }


    // ==================================================
    // RUN CBS / PCBS
    // ==================================================

    IEnumerator RunCBS()
    {
        if (!fairBenchmarkMode) Debug.Log(
            "================================"
        );


        if (
            useParallelLowLevelReplanning
        )
        {
            if (!fairBenchmarkMode) Debug.Log(
                "===== PCBS STARTED ====="
            );
        }

        else
        {
            if (!fairBenchmarkMode) Debug.Log(
                "===== SEQUENTIAL CBS STARTED ====="
            );
        }


        if (!fairBenchmarkMode) Debug.Log(
            "================================"
        );


        openList.Clear();

        expandedNodeCount =
            0;


        float searchStartTime =
            Time.realtimeSinceStartup;

        // A sum of sequential search time + CPU effort across parallel workers
        // is an appropriate search-work comparison with sequential Improved A*.
        System.Diagnostics.Stopwatch cbsWallWatch =
            System.Diagnostics.Stopwatch.StartNew();
        System.Diagnostics.Stopwatch cbsMainActiveWatch =
            System.Diagnostics.Stopwatch.StartNew();
        double parallelWorkerWorkMs = 0.0;
        double parallelWorkerCriticalMs = 0.0;



        // ==================================================
        // ROOT NODE
        // ==================================================

        CBSNode root =
            BuildRootNode();


        if (
            root == null
        )
        {
            Debug.LogError(
                "CBS ABORTED: Could not build root node."
            );

            yield break;
        }


        openList.Add(
            root
        );


        // ==================================================
        // HIGH-LEVEL CBS SEARCH
        // ==================================================

        while (
            openList.Count > 0
        )
        {
            // ==================================================
            // GLOBAL FREEZE PROTECTION
            // ==================================================

            if (
                ReachedCBSLimit(
                    searchStartTime
                )
            )
            {
                yield break;
            }


            // ==================================================
            // GET LOWEST-COST CT NODE
            // ==================================================

            CBSNode current =
                GetLowestCostNode();


            expandedNodeCount++;


            if (!fairBenchmarkMode) Debug.Log(
                "================================"
            );


            if (!fairBenchmarkMode) Debug.Log(
                "EXPANDING CT NODE #" +
                expandedNodeCount +
                " | Cost = " +
                current.cost +
                " | OPEN remaining = " +
                openList.Count
            );


            // ==================================================
            // FIND FIRST CONFLICT
            // ==================================================

            CBSConflict conflict =
                FindFirstConflict(
                    current.paths
                );


            // ==================================================
            // SOLUTION FOUND
            // ==================================================

            if (
                conflict ==
                    null
            )
            {
                if (!fairBenchmarkMode) Debug.Log(
                    "================================"
                );


                if (
                    useParallelLowLevelReplanning
                )
                {
                    if (!fairBenchmarkMode) Debug.Log(
                        "===== PCBS SOLUTION FOUND ====="
                    );
                }

                else
                {
                    if (!fairBenchmarkMode) Debug.Log(
                        "===== CBS SOLUTION FOUND ====="
                    );
                }


                if (!fairBenchmarkMode) Debug.Log(
                    "Solution Cost = " +
                    current.cost
                );


                if (!fairBenchmarkMode) Debug.Log(
                    "Expanded CT Nodes = " +
                    expandedNodeCount
                );



                cbsMainActiveWatch.Stop();
                cbsWallWatch.Stop();

                double initialSTAStarMs = 0.0;
                foreach (SpaceTimeAStar planner in planners)
                {
                    initialSTAStarMs += planner.initialPlanningMilliseconds;
                }

                double cbsMainMs = cbsMainActiveWatch.Elapsed.TotalMilliseconds;
                double cbsWorkerMs = parallelWorkerWorkMs;
                double cbsComputeWorkMs = cbsMainMs + cbsWorkerMs;
                double totalSearchWorkMs = initialSTAStarMs + cbsComputeWorkMs;
                double idealCriticalPathMs =
                    initialSTAStarMs + cbsMainMs + parallelWorkerCriticalMs;
                double observedUnityElapsedMs =
                    initialSTAStarMs + cbsWallWatch.Elapsed.TotalMilliseconds;

                // Log after timing: no console messages are counted in benchmark.
                UnityEngine.Debug.Log("===== FAIR PCBS COMPUTATION BENCHMARK =====");
                UnityEngine.Debug.Log("Solution Cost = " + current.cost
                    + " | Expanded CT Nodes = " + expandedNodeCount);
                UnityEngine.Debug.Log("Initial STA* Search Work = "
                    + initialSTAStarMs.ToString("F3") + " ms");
                UnityEngine.Debug.Log("CBS Main-Thread Active Work = "
                    + cbsMainMs.ToString("F3") + " ms");
                UnityEngine.Debug.Log("CBS Parallel Worker Work (SUM) = "
                    + cbsWorkerMs.ToString("F3") + " ms");
                UnityEngine.Debug.Log("CBS Total Search Work = "
                    + cbsComputeWorkMs.ToString("F3") + " ms");
                UnityEngine.Debug.Log("TOTAL FAIR PCBS SEARCH WORK = "
                    + totalSearchWorkMs.ToString("F3") + " ms");
                UnityEngine.Debug.Log("PCBS Idealized Critical-Path Estimate = "
                    + idealCriticalPathMs.ToString("F3") + " ms");
                UnityEngine.Debug.Log("PCBS Observed Unity Elapsed (contains frame waits) = "
                    + observedUnityElapsedMs.ToString("F3") + " ms");
                UnityEngine.Debug.Log("NOTE: Compare SEARCH WORK across both planners; "
                    + "do not compare observed PCBS wall time to sequential A* CPU work.");

                if (SimulationMetrics.Instance != null)
                {
                    // Shared reported field is consistently CPU search WORK, not latency.
                    SimulationMetrics.Instance.RecordComputationTime(
                        totalSearchWorkMs
                    );
                }

                ApplySolution(
                    current
                );

                yield break;

            }


            PrintConflict(
                conflict
            );


            // ==================================================
            // PARALLEL LOW-LEVEL REPLANNING
            // ==================================================

            if (
                useParallelLowLevelReplanning
            )
            {
                // ==================================================
                // PREPARE BOTH CT CHILDREN FIRST
                // ==================================================

                CBSNode childA =
                    PrepareChild(
                        current,
                        conflict,
                        conflict.robotA
                    );


                CBSNode childB =
                    PrepareChild(
                        current,
                        conflict,
                        conflict.robotB
                    );


                // ==================================================
                // START BOTH STA* SEARCHES
                //
                // Both Task objects are started before
                // we begin waiting.
                // ==================================================

                float parallelWallStart =
                    Time.realtimeSinceStartup;


                Task<SpaceTimeAStar.ParallelReplanResult>
                    taskA =
                        planners[
                            conflict.robotA
                        ].StartParallelReplanForCBS(

                            childA.vertexConstraints[
                                conflict.robotA
                            ],

                            childA.edgeConstraints[
                                conflict.robotA
                            ]
                        );


                Task<SpaceTimeAStar.ParallelReplanResult>
                    taskB =
                        planners[
                            conflict.robotB
                        ].StartParallelReplanForCBS(

                            childB.vertexConstraints[
                                conflict.robotB
                            ],

                            childB.edgeConstraints[
                                conflict.robotB
                            ]
                        );


                if (!fairBenchmarkMode) Debug.Log(
                    "PCBS PARALLEL LOW-LEVEL START" +
                    " | Robot " +
                    (conflict.robotA + 1) +
                    " + Robot " +
                    (conflict.robotB + 1)
                );


                // ==================================================
                // WAIT FOR BOTH CHILD SEARCHES
                //
                // Unity remains responsive because the expensive
                // pure C# STA* work is on worker threads.
                // ==================================================

                cbsMainActiveWatch.Stop();
                while (
                    !taskA.IsCompleted ||
                    !taskB.IsCompleted
                )
                {
                    yield return null;
                }
                cbsMainActiveWatch.Start();


                float parallelWallMilliseconds =
                    (
                        Time.realtimeSinceStartup -
                        parallelWallStart
                    ) *
                    1000f;


                SpaceTimeAStar.ParallelReplanResult
                    resultA =
                        GetParallelTaskResult(
                            taskA,
                            conflict.robotA
                        );


                SpaceTimeAStar.ParallelReplanResult
                    resultB =
                        GetParallelTaskResult(
                            taskB,
                            conflict.robotB
                        );

                // Include both parallel worker CPU times for total search effort.
                // Also retain the max worker duration as an idealized critical path.
                double workerAMs = resultA != null
                    ? resultA.elapsedMillisecondsPrecise : 0.0;
                double workerBMs = resultB != null
                    ? resultB.elapsedMillisecondsPrecise : 0.0;
                parallelWorkerWorkMs += workerAMs + workerBMs;
                parallelWorkerCriticalMs += System.Math.Max(workerAMs, workerBMs);


                if (!fairBenchmarkMode) Debug.Log(
                    "PCBS PARALLEL LOW-LEVEL DONE" +
                    " | Wall = " +
                    parallelWallMilliseconds.ToString("F1") +
                    " ms" +
                    " | A worker = " +
                    GetWorkerTimeText(
                        resultA
                    ) +
                    " | B worker = " +
                    GetWorkerTimeText(
                        resultB
                    )
                );


                // ==================================================
                // CONVERT RESULTS BACK TO CBS CHILDREN
                // ==================================================

                CBSNode finalizedChildA =
                    FinalizeParallelChild(
                        childA,
                        conflict.robotA,
                        resultA,
                        "Child A"
                    );


                CBSNode finalizedChildB =
                    FinalizeParallelChild(
                        childB,
                        conflict.robotB,
                        resultB,
                        "Child B"
                    );


                if (
                    finalizedChildA !=
                        null
                )
                {
                    openList.Add(
                        finalizedChildA
                    );


                    if (!fairBenchmarkMode) Debug.Log(
                        "Child A added to OPEN" +
                        " | Cost = " +
                        finalizedChildA.cost
                    );
                }

                else
                {
                    if (!fairBenchmarkMode) Debug.Log(
                        "Child A discarded: " +
                        "no valid parallel STA* path."
                    );
                }


                if (
                    finalizedChildB !=
                        null
                )
                {
                    openList.Add(
                        finalizedChildB
                    );


                    if (!fairBenchmarkMode) Debug.Log(
                        "Child B added to OPEN" +
                        " | Cost = " +
                        finalizedChildB.cost
                    );
                }

                else
                {
                    if (!fairBenchmarkMode) Debug.Log(
                        "Child B discarded: " +
                        "no valid parallel STA* path."
                    );
                }


                // ==================================================
                // IMPORTANT PCBS BEHAVIOUR
                //
                // Only after BOTH child low-level searches finish
                // are their CT nodes inserted into OPEN.
                //
                // The next CT node is selected on the next loop.
                // ==================================================

                cbsMainActiveWatch.Stop();
                yield return null;
                cbsMainActiveWatch.Start();
            }

            else
            {
                // ==================================================
                // OLD SEQUENTIAL CBS FALLBACK
                // ==================================================

                CBSNode childA =
                    CreateChildSequential(
                        current,
                        conflict,
                        conflict.robotA
                    );


                if (
                    childA !=
                        null
                )
                {
                    openList.Add(
                        childA
                    );


                    if (!fairBenchmarkMode) Debug.Log(
                        "Child A added to OPEN" +
                        " | Cost = " +
                        childA.cost
                    );
                }

                else
                {
                    if (!fairBenchmarkMode) Debug.Log(
                        "Child A discarded: " +
                        "no valid STA* path."
                    );
                }


                cbsMainActiveWatch.Stop();
                yield return null;
                cbsMainActiveWatch.Start();


                if (
                    ReachedCBSLimit(
                        searchStartTime
                    )
                )
                {
                    yield break;
                }


                CBSNode childB =
                    CreateChildSequential(
                        current,
                        conflict,
                        conflict.robotB
                    );


                if (
                    childB !=
                        null
                )
                {
                    openList.Add(
                        childB
                    );


                    if (!fairBenchmarkMode) Debug.Log(
                        "Child B added to OPEN" +
                        " | Cost = " +
                        childB.cost
                    );
                }

                else
                {
                    if (!fairBenchmarkMode) Debug.Log(
                        "Child B discarded: " +
                        "no valid STA* path."
                    );
                }


                cbsMainActiveWatch.Stop();
                yield return null;
                cbsMainActiveWatch.Start();
            }
        }


        // ==================================================
        // OPEN LIST EMPTY
        // ==================================================

        Debug.LogError(
            "===== CBS FAILED =====\n" +
            "CBS OPEN list became empty.\n" +
            "No conflict-free solution was found."
        );
    }


    // ==================================================
    // GET PARALLEL TASK RESULT
    // ==================================================

    SpaceTimeAStar.ParallelReplanResult
        GetParallelTaskResult(
            Task<SpaceTimeAStar.ParallelReplanResult> task,
            int robotIndex)
    {
        if (
            task == null
        )
        {
            Debug.LogError(
                "PCBS: Parallel task is null " +
                "for Robot " +
                (robotIndex + 1)
            );

            return null;
        }


        if (
            task.IsCanceled
        )
        {
            Debug.LogError(
                "PCBS: Parallel task was cancelled " +
                "for Robot " +
                (robotIndex + 1)
            );

            return null;
        }


        if (
            task.IsFaulted
        )
        {
            string message =
                "Unknown worker exception";


            if (
                task.Exception !=
                    null
            )
            {
                System.Exception baseException =
                    task.Exception
                        .GetBaseException();


                if (
                    baseException !=
                        null
                )
                {
                    message =
                        baseException.Message;
                }
            }


            Debug.LogError(
                "PCBS: Parallel STA* worker faulted" +
                " | Robot " +
                (robotIndex + 1) +
                " | " +
                message
            );


            return null;
        }


        return
            task.Result;
    }


    // ==================================================
    // WORKER TIME TEXT
    // ==================================================

    string GetWorkerTimeText(
        SpaceTimeAStar.ParallelReplanResult result)
    {
        if (
            result ==
                null
        )
        {
            return "failed";
        }


        return
            result.elapsedMilliseconds +
            " ms";
    }


    // ==================================================
    // PREPARE CHILD FOR PARALLEL SEARCH
    // ==================================================

    CBSNode PrepareChild(
        CBSNode parent,
        CBSConflict conflict,
        int constrainedRobot)
    {
        CBSNode child =
            new CBSNode(
                parent
            );


        AddConstraint(
            child,
            conflict,
            constrainedRobot
        );


        return child;
    }


    // ==================================================
    // FINALIZE PARALLEL CHILD
    //
    // Worker result is converted back to SpaceTimeNode
    // objects on Unity's main thread here.
    // ==================================================

    CBSNode FinalizeParallelChild(
        CBSNode child,
        int constrainedRobot,
        SpaceTimeAStar.ParallelReplanResult result,
        string childLabel)
    {
        if (
            result ==
                null
        )
        {
            return null;
        }


        if (
            !result.success
        )
        {
            Debug.LogWarning(
                childLabel +
                " parallel STA* failed" +
                " | Robot " +
                (constrainedRobot + 1) +
                " | Reason = " +
                result.message +
                " | Expanded = " +
                result.expandedStates +
                " | Worker Time = " +
                result.elapsedMilliseconds +
                " ms"
            );


            return null;
        }


        List<SpaceTimeNode> newPath =
            planners[
                constrainedRobot
            ].MaterializeParallelPath(
                result
            );


        if (
            newPath == null ||
            newPath.Count == 0
        )
        {
            return null;
        }


        child.paths[
            constrainedRobot
        ] =
            newPath;


        child.cost =
            CalculateCost(
                child.paths
            );


        if (!fairBenchmarkMode) Debug.Log(
            childLabel +
            " parallel STA* success" +
            " | Robot " +
            (constrainedRobot + 1) +
            " | Timesteps = " +
            (newPath.Count - 1) +
            " | Expanded = " +
            result.expandedStates +
            " | Worker Time = " +
            result.elapsedMilliseconds +
            " ms"
        );


        return child;
    }


    // ==================================================
    // CBS LIMIT CHECK
    // ==================================================

    bool ReachedCBSLimit(
        float searchStartTime)
    {
        if (
            maxCTNodeExpansions > 0 &&
            expandedNodeCount >=
                maxCTNodeExpansions
        )
        {
            Debug.LogError(
                "===== CBS ABORTED =====\n" +
                "Reason: CT node expansion limit reached.\n" +
                "Expanded CT Nodes = " +
                expandedNodeCount +
                "\nLimit = " +
                maxCTNodeExpansions
            );


            return true;
        }


        float elapsedTime =
            Time.realtimeSinceStartup -
            searchStartTime;


        if (
            maxCBSSearchSeconds > 0f &&
            elapsedTime >=
                maxCBSSearchSeconds
        )
        {
            Debug.LogError(
                "===== CBS ABORTED =====\n" +
                "Reason: CBS search timeout.\n" +
                "Elapsed Time = " +
                elapsedTime.ToString("F2") +
                " s\nExpanded CT Nodes = " +
                expandedNodeCount
            );


            return true;
        }


        return false;
    }


    // ==================================================
    // BUILD ROOT
    // ==================================================

    CBSNode BuildRootNode()
    {
        CBSNode root =
            new CBSNode(
                planners.Length
            );


        for (
            int i = 0;
            i < planners.Length;
            i++
        )
        {
            if (
                planners[i].finalPath == null ||
                planners[i].finalPath.Count == 0
            )
            {
                return null;
            }


            root.paths[i] =
                new List<SpaceTimeNode>(
                    planners[i].finalPath
                );
        }


        root.cost =
            CalculateCost(
                root.paths
            );


        if (!fairBenchmarkMode) Debug.Log(
            "===== CBS ROOT NODE ====="
        );


        for (
            int i = 0;
            i < planners.Length;
            i++
        )
        {
            if (!fairBenchmarkMode) Debug.Log(
                "Robot " +
                (i + 1) +
                " | Timesteps = " +
                (root.paths[i].Count - 1) +
                " | Cost = " +
                GetPathCost(
                    root.paths[i]
                )
            );
        }


        if (!fairBenchmarkMode) Debug.Log(
            "Root Total Cost = " +
            root.cost
        );


        return root;
    }


    // ==================================================
    // GET LOWEST-COST CT NODE
    // ==================================================

    CBSNode GetLowestCostNode()
    {
        CBSNode best =
            openList[0];


        int bestIndex =
            0;


        for (
            int i = 1;
            i < openList.Count;
            i++
        )
        {
            if (
                openList[i].cost <
                    best.cost
            )
            {
                best =
                    openList[i];


                bestIndex =
                    i;
            }
        }


        openList.RemoveAt(
            bestIndex
        );


        return best;
    }


    // ==================================================
    // SEQUENTIAL CHILD CREATION
    //
    // Retained so you can turn parallel mode OFF
    // and compare CBS vs PCBS using the same scene.
    // ==================================================

    CBSNode CreateChildSequential(
        CBSNode parent,
        CBSConflict conflict,
        int constrainedRobot)
    {
        CBSNode child =
            new CBSNode(
                parent
            );


        AddConstraint(
            child,
            conflict,
            constrainedRobot
        );


        List<SpaceTimeNode> newPath =
            planners[
                constrainedRobot
            ].ReplanForCBS(

                child.vertexConstraints[
                    constrainedRobot
                ],

                child.edgeConstraints[
                    constrainedRobot
                ]
            );


        if (
            newPath == null ||
            newPath.Count == 0
        )
        {
            return null;
        }


        child.paths[
            constrainedRobot
        ] =
            newPath;


        child.cost =
            CalculateCost(
                child.paths
            );


        return child;
    }


    // ==================================================
    // ADD CONSTRAINT
    // ==================================================

    void AddConstraint(
        CBSNode child,
        CBSConflict conflict,
        int robotIndex)
    {
        // ==================================================
        // VERTEX CONFLICT
        // ==================================================

        if (
            conflict.type ==
                CBSConflictType.Vertex
        )
        {
            child.vertexConstraints[
                robotIndex
            ].Add(

                new SpaceTimeVertexConstraint(
                    conflict.vertexX,
                    conflict.vertexZ,
                    conflict.time
                )
            );


            if (!fairBenchmarkMode) Debug.Log(
                "Added VERTEX constraint" +
                " | Robot " +
                (robotIndex + 1) +
                " | Node=(" +
                conflict.vertexX +
                "," +
                conflict.vertexZ +
                ")" +
                " | t=" +
                conflict.time
            );


            return;
        }


        // ==================================================
        // EDGE CONFLICT
        // ==================================================

        if (
            robotIndex ==
                conflict.robotA
        )
        {
            child.edgeConstraints[
                robotIndex
            ].Add(

                new SpaceTimeEdgeConstraint(
                    conflict.aFromX,
                    conflict.aFromZ,

                    conflict.aToX,
                    conflict.aToZ,

                    conflict.time
                )
            );


            if (!fairBenchmarkMode) Debug.Log(
                "Added EDGE constraint" +
                " | Robot " +
                (robotIndex + 1) +
                " | (" +
                conflict.aFromX +
                "," +
                conflict.aFromZ +
                ") -> (" +
                conflict.aToX +
                "," +
                conflict.aToZ +
                ")" +
                " | t=" +
                conflict.time
            );
        }

        else
        {
            child.edgeConstraints[
                robotIndex
            ].Add(

                new SpaceTimeEdgeConstraint(
                    conflict.bFromX,
                    conflict.bFromZ,

                    conflict.bToX,
                    conflict.bToZ,

                    conflict.time
                )
            );


            if (!fairBenchmarkMode) Debug.Log(
                "Added EDGE constraint" +
                " | Robot " +
                (robotIndex + 1) +
                " | (" +
                conflict.bFromX +
                "," +
                conflict.bFromZ +
                ") -> (" +
                conflict.bToX +
                "," +
                conflict.bToZ +
                ")" +
                " | t=" +
                conflict.time
            );
        }
    }


    // ==================================================
    // FIND FIRST CONFLICT
    // ==================================================

    CBSConflict FindFirstConflict(
        List<List<SpaceTimeNode>> paths)
    {
        int maxTime =
            0;


        foreach (
            List<SpaceTimeNode> path
            in paths
        )
        {
            if (
                path == null ||
                path.Count == 0
            )
            {
                continue;
            }


            maxTime =
                Mathf.Max(
                    maxTime,
                    path.Count - 1
                );
        }


        // Earliest timestep first.

        for (
            int time = 0;
            time <= maxTime;
            time++
        )
        {
            for (
                int i = 0;
                i < paths.Count;
                i++
            )
            {
                for (
                    int j = i + 1;
                    j < paths.Count;
                    j++
                )
                {
                    Node nodeA =
                        GetNodeAtTime(
                            paths[i],
                            time
                        );


                    Node nodeB =
                        GetNodeAtTime(
                            paths[j],
                            time
                        );


                    // ==================================================
                    // VERTEX CONFLICT
                    // ==================================================

                    if (
                        nodeA ==
                            nodeB
                    )
                    {
                        return
                            CBSConflict.CreateVertex(
                                i,
                                j,

                                nodeA.gridX,
                                nodeA.gridZ,

                                time
                            );
                    }


                    if (
                        time >=
                            maxTime
                    )
                    {
                        continue;
                    }


                    Node nextA =
                        GetNodeAtTime(
                            paths[i],
                            time + 1
                        );


                    Node nextB =
                        GetNodeAtTime(
                            paths[j],
                            time + 1
                        );


                    bool robotAMoved =
                        nodeA !=
                            nextA;


                    bool robotBMoved =
                        nodeB !=
                            nextB;


                    // ==================================================
                    // EDGE CONFLICT
                    //
                    // A -> B
                    // B -> A
                    // ==================================================

                    if (
                        robotAMoved &&
                        robotBMoved &&

                        nodeA ==
                            nextB &&

                        nodeB ==
                            nextA
                    )
                    {
                        return
                            CBSConflict.CreateEdge(
                                i,
                                j,

                                nodeA.gridX,
                                nodeA.gridZ,

                                nextA.gridX,
                                nextA.gridZ,

                                nodeB.gridX,
                                nodeB.gridZ,

                                nextB.gridX,
                                nextB.gridZ,

                                time
                            );
                    }
                }
            }
        }


        return null;
    }


    // ==================================================
    // GET NODE AT TIME
    //
    // After arrival, robot remains at its goal.
    // ==================================================

    Node GetNodeAtTime(
        List<SpaceTimeNode> path,
        int time)
    {
        if (
            time <
                path.Count
        )
        {
            return
                path[
                    time
                ].node;
        }


        return
            path[
                path.Count - 1
            ].node;
    }


    // ==================================================
    // COST
    // ==================================================

    int CalculateCost(
        List<List<SpaceTimeNode>> paths)
    {
        int total =
            0;


        foreach (
            List<SpaceTimeNode> path
            in paths
        )
        {
            total +=
                GetPathCost(
                    path
                );
        }


        return total;
    }


    int GetPathCost(
        List<SpaceTimeNode> path)
    {
        if (
            path == null ||
            path.Count == 0
        )
        {
            return 0;
        }


        return
            path[
                path.Count - 1
            ].gCost;
    }


    // ==================================================
    // APPLY SOLUTION
    // ==================================================

    void ApplySolution(
        CBSNode solution)
    {
        if (!fairBenchmarkMode) Debug.Log(
            "Applying final CBS paths..."
        );


        // ==================================================
        // ASSIGN ALL FINAL PATHS FIRST
        // ==================================================

        for (
            int i = 0;
            i < planners.Length;
            i++
        )
        {
            planners[i].finalPath =
                solution.paths[i];


            RobotPath robotPath =
                planners[i]
                    .robot
                    .GetComponent<RobotPath>();


            if (
                robotPath ==
                    null
            )
            {
                Debug.LogError(
                    planners[i].robot.name +
                    " does not have RobotPath."
                );

                continue;
            }


            List<Node> spatialPath =
                new List<Node>();


            foreach (
                SpaceTimeNode state
                in solution.paths[i]
            )
            {
                spatialPath.Add(
                    state.node
                );
            }


            robotPath.path =
                spatialPath;


            robotPath.routeFinalized =
                false;
        }


        // ==================================================
        // MILESTONE 7A
        // RECORD FINAL PCBS PATH DISTANCES
        // ==================================================

        if (SimulationMetrics.Instance != null)
        {
            SimulationMetrics.Instance
                .RecordFinalPCBSPaths(planners);
        }
        else
        {
            Debug.LogWarning(
                "SimulationMetrics not found in scene."
            );
        }


        // ==================================================
        // MILESTONE 7B - START MOVEMENT METRICS
        // ==================================================

        if (SimulationMetrics.Instance != null)
        {
            SimulationMetrics.Instance.StartSimulation();
        }
        else
        {
            Debug.LogError(
                "SimulationMetrics instance not found!"
            );
        }




        // ==================================================
        // RELEASE ALL ROBOTS TOGETHER
        // ==================================================

        for (
            int i = 0;
            i < planners.Length;
            i++
        )
        {
            RobotPath robotPath =
                planners[i]
                    .robot
                    .GetComponent<RobotPath>();


            if (
                robotPath !=
                    null
            )
            {
                robotPath.routeFinalized =
                    true;
            }
        }


        if (!fairBenchmarkMode) Debug.Log(
            "CBS paths finalized. " +
            "Robots may now move."
        );
    }


    // ==================================================
    // PRINT CONFLICT
    // ==================================================

    void PrintConflict(
        CBSConflict conflict)
    {
        if (
            conflict.type ==
                CBSConflictType.Vertex
        )
        {
            if (!fairBenchmarkMode) Debug.Log(
                "VERTEX CONFLICT" +
                " | Robot " +
                (conflict.robotA + 1) +
                " vs Robot " +
                (conflict.robotB + 1) +
                " | Node=(" +
                conflict.vertexX +
                "," +
                conflict.vertexZ +
                ")" +
                " | t=" +
                conflict.time
            );
        }

        else
        {
            if (!fairBenchmarkMode) Debug.Log(
                "EDGE CONFLICT" +
                " | Robot " +
                (conflict.robotA + 1) +
                " vs Robot " +
                (conflict.robotB + 1) +
                " | t=" +
                conflict.time +
                " -> " +
                (conflict.time + 1)
            );
        }
    }
}
