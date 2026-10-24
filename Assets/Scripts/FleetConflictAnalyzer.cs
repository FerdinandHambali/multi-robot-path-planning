using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class FleetConflictAnalyzer : MonoBehaviour
{
    // ==================================================
    // REFERENCES
    // ==================================================

    private List<ImprovedAStar> planners =
        new List<ImprovedAStar>();


    // ==================================================
    // SETTINGS
    // ==================================================

    public bool printIndividualConflicts = false;

    [Header("Fair Timing Experiment")]
    [Tooltip("ON: do not generate console log strings while Select/Replace is being timed.")]
    public bool fairBenchmarkMode = true;

    public int benchmarkInitialConflicts { get; private set; }
    public int benchmarkFinalConflicts { get; private set; }
    public float benchmarkInitialTc { get; private set; }
    public float benchmarkFinalTc { get; private set; }

    // Safety termination only
    public int maximumReplacementIterations = 50;

    private const float TC_EPSILON = 0.0001f;


    // ==================================================
    // TIMED NODE
    // ==================================================

    private class TimedNode
    {
        public Node node;

        public float arrivalTime;

        public float windowStart;
        public float windowEnd;
    }


    // ==================================================
    // TIMED EDGE
    // ==================================================

    private class TimedEdge
    {
        public Node from;
        public Node to;

        public float startTime;
        public float endTime;
    }


    // ==================================================
    // TIMED PATH
    // ==================================================

    private class TimedPath
    {
        public ImprovedAStar planner;

        public List<TimedNode> nodes =
            new List<TimedNode>();

        public List<TimedEdge> edges =
            new List<TimedEdge>();

        public float travelTime;
    }


    // ==================================================
    // CONFLICT RESULT
    // ==================================================

    private class ConflictResult
    {
        public int conflictCount = 0;

        public float tc = 0f;

        public HashSet<ImprovedAStar>
            conflictingPlanners =
            new HashSet<ImprovedAStar>();
    }


    // ==================================================
    // START
    // ==================================================

    IEnumerator Start()
    {
        // Wait one frame so ImprovedAStar scripts
        // finish generating their databases.
        yield return null;


        ImprovedAStar[] foundPlanners =
            FindObjectsByType<ImprovedAStar>(
                FindObjectsSortMode.None
            );


        planners.Clear();
        planners.AddRange(foundPlanners);


        Debug.Log(
            "FleetConflictAnalyzer found " +
            planners.Count +
            " planners."
        );


        // ==================================================
        // INITIALISE ALL ROBOTS
        //
        // Path 1 is the initial route.
        // Robots must NOT move yet.
        // ==================================================

        foreach (
            ImprovedAStar planner
            in planners
        )
        {
            if (planner == null)
                continue;


            if (planner.robotPath != null)
            {
                planner.robotPath.routeFinalized =
                    false;
            }


            if (
                planner.GetPathCount() > 0
            )
            {
                planner.SelectDatabasePath(0);
            }
        }


        // ==================================================
        // SELECT / REPLACE
        // ==================================================

        // ==================================================
        // MILESTONE 4D
        // COMPUTATION TIME
        // ==================================================

        // Add the computation time used by
        // every robot to generate its
        // Path Expert Database.
        double totalPathPlanningMs = 0.0;


        foreach (
            ImprovedAStar planner
            in planners
        )
        {
            if (planner == null)
                continue;


            totalPathPlanningMs +=
                planner.PlanningTimeMilliseconds;
        }


        // Measure Select / Replace separately.
        System.Diagnostics.Stopwatch
            selectReplaceWatch =
                System.Diagnostics.Stopwatch
                    .StartNew();


        // ==================================================
        // SELECT / REPLACE
        // ==================================================

        RunSelectReplace();


        selectReplaceWatch.Stop();

        // Debug summary after timing; these messages are NOT charged to the algorithm.
        Debug.Log("FAIR A* | Initial Conflicts = " + benchmarkInitialConflicts
            + " | Initial Tc = " + benchmarkInitialTc.ToString("F3") + " s");
        Debug.Log("FAIR A* | Final Conflicts = " + benchmarkFinalConflicts
            + " | Final Tc = " + benchmarkFinalTc.ToString("F3") + " s");
        foreach (ImprovedAStar planner in planners)
        {
            if (planner != null && planner.robot != null)
                Debug.Log("FAIR A* | " + planner.robot.name +
                    " selected Path " + planner.GetSelectedPathNumber());
        }


        double selectReplaceMs =
            selectReplaceWatch
                .Elapsed
                .TotalMilliseconds;


        double totalComputationMs =
            totalPathPlanningMs +
            selectReplaceMs;


        Debug.Log(
            "Total Path Planning Time = " +
            totalPathPlanningMs.ToString("F3") +
            " ms"
        );


        Debug.Log(
            "Select / Replace Time = " +
            selectReplaceMs.ToString("F3") +
            " ms"
        );


        Debug.Log(
            "FAIR A* SEARCH WORK (LOG-FREE) = " +
            totalComputationMs.ToString("F3") +
            " ms"
        );


        if (
            SimulationMetrics.Instance != null
        )
        {
            SimulationMetrics.Instance
                .RecordComputationTime(
                    totalComputationMs
                );
        }


        // ==================================================
        // FINAL CON(R)
        // ==================================================

        Debug.Log(
            "================================"
        );

        Debug.Log(
            "FINAL CON(R)"
        );

        Debug.Log(
            "================================"
        );

        AnalyzeCurrentPaths();


        // ==================================================
        // MILESTONE 4B
        // Record distance of FINAL selected routes.
        // ==================================================

        if (SimulationMetrics.Instance != null)
        {
            SimulationMetrics.Instance.RecordFinalPaths(
                planners
            );
        }


        // ==================================================
        // MILESTONE 4A
        // Start actual robot movement timer.
        // ==================================================

        if (SimulationMetrics.Instance != null)
        {
            SimulationMetrics.Instance.StartSimulation();
        }

        foreach (
            ImprovedAStar planner
            in planners
        )
        {
            if (
                planner != null &&
                planner.robotPath != null
            )
            {
                planner.robotPath.routeFinalized =
                    true;
            }
        }


        Debug.Log(
            "Routes finalized. Robots may now move."
        );
    }


    // ==================================================
    // MILESTONE 3B
    //
    // SELECT / REPLACE PROCEDURE
    // ==================================================

    void RunSelectReplace()
    {
        if (!fairBenchmarkMode) Debug.Log(
            "================================"
        );

        if (!fairBenchmarkMode) Debug.Log(
            "SELECT / REPLACE PROCEDURE"
        );

        if (!fairBenchmarkMode) Debug.Log(
            "================================"
        );


        // ==================================================
        // INITIAL CON(R)
        // ==================================================

        ConflictResult currentResult =
            EvaluateFleet(
                null,
                false,
                false
            );

        benchmarkInitialConflicts = currentResult.conflictCount;
        benchmarkInitialTc = currentResult.tc;


        if (!fairBenchmarkMode) Debug.Log(
            "Initial Conflicts = " +
            currentResult.conflictCount
        );


        if (!fairBenchmarkMode) Debug.Log(
            "Initial Tc = " +
            currentResult.tc.ToString("F3") +
            " s"
        );


        int iteration = 0;


        // ==================================================
        // REPLACEMENT LOOP
        // ==================================================

        while (
            currentResult.tc >
                TC_EPSILON &&
            iteration <
                maximumReplacementIterations
        )
        {
            iteration++;


            if (!fairBenchmarkMode) Debug.Log(
                "--------------------------------"
            );


            if (!fairBenchmarkMode) Debug.Log(
                "Replacement Iteration " +
                iteration
            );


            if (!fairBenchmarkMode) Debug.Log(
                "Current Tc = " +
                currentResult.tc
                    .ToString("F3") +
                " s"
            );


            ImprovedAStar bestPlanner =
                null;


            int bestPathIndex = -1;


            float bestTc =
                currentResult.tc;


            // ==================================================
            // ONLY ROBOTS CURRENTLY INVOLVED
            // IN A CONFLICT ARE CONSIDERED
            // ==================================================

            // ==================================================
            // DETERMINISTIC ROBOT ORDER
            //
            // HashSet order is not guaranteed.
            // Sort conflicting robots by name:
            //
            // Robot1, Robot2, Robot3...
            //
            // If two alternatives have the same Tc',
            // the earlier robot keeps priority.
            // ==================================================

            List<ImprovedAStar> orderedConflictingPlanners =
                new List<ImprovedAStar>(
                    currentResult.conflictingPlanners
                );


            orderedConflictingPlanners.Sort(
                (a, b) =>
                {
                    string nameA =
                        a.robot != null
                        ? a.robot.name
                        : a.name;

                    string nameB =
                        b.robot != null
                        ? b.robot.name
                        : b.name;

                    return string.CompareOrdinal(
                        nameA,
                        nameB
                    );
                }
            );


            foreach (
                ImprovedAStar planner
                in orderedConflictingPlanners
            )
            {
                if (planner == null)
                    continue;


                int pathCount =
                    planner.GetPathCount();


                string robotName =
                    planner.robot != null
                    ? planner.robot.name
                    : planner.name;


                if (pathCount <= 1)
                {
                    if (!fairBenchmarkMode) Debug.Log(
                        robotName +
                        " has no alternative paths."
                    );

                    continue;
                }


                int currentPathIndex =
                    planner
                        .GetSelectedPathNumber()
                    - 1;


                // ==================================================
                // TRY EVERY STORED ALTERNATIVE
                // ==================================================

                for (
                    int candidateIndex = 0;
                    candidateIndex <
                        pathCount;
                    candidateIndex++
                )
                {
                    // Ignore current path.
                    if (
                        candidateIndex ==
                        currentPathIndex
                    )
                    {
                        continue;
                    }


                    List<Node> candidatePath =
                        planner.GetDatabasePath(
                            candidateIndex
                        );


                    if (
                        candidatePath == null ||
                        candidatePath.Count == 0
                    )
                    {
                        continue;
                    }


                    // ==================================================
                    // TEMPORARILY TEST CANDIDATE
                    //
                    // Do NOT actually switch the
                    // robot yet.
                    // ==================================================

                    Dictionary<
                        ImprovedAStar,
                        List<Node>
                    > overridePaths =
                        new Dictionary<
                            ImprovedAStar,
                            List<Node>
                        >();


                    overridePaths.Add(
                        planner,
                        candidatePath
                    );


                    ConflictResult
                        candidateResult =
                            EvaluateFleet(
                                overridePaths,
                                false,
                                false
                            );


                    if (!fairBenchmarkMode) Debug.Log(
                        "Considering " +
                        robotName +
                        " Path " +
                        (candidateIndex + 1) +
                        " | Tc' = " +
                        candidateResult.tc
                            .ToString("F3") +
                        " s"
                    );


                    // ==================================================
                    // REPLACEMENT CONDITION
                    //
                    // Tc' < Tc
                    // ==================================================

                    if (
                        candidateResult.tc <
                        bestTc -
                        TC_EPSILON
                    )
                    {
                        bestTc =
                            candidateResult.tc;


                        bestPlanner =
                            planner;


                        bestPathIndex =
                            candidateIndex;
                    }
                }
            }


            // ==================================================
            // NO BETTER STORED PATH EXISTS
            // ==================================================

            if (
                bestPlanner == null ||
                bestPathIndex < 0
            )
            {
                if (!fairBenchmarkMode) Debug.Log(
                    "No stored alternative reduces Tc."
                );


                break;
            }


            // ==================================================
            // ACCEPT BEST REPLACEMENT
            // ==================================================

            int oldPathNumber =
                bestPlanner
                    .GetSelectedPathNumber();


            float oldTc =
                currentResult.tc;


            bestPlanner.SelectDatabasePath(
                bestPathIndex
            );


            string selectedRobotName =
                bestPlanner.robot != null
                ? bestPlanner.robot.name
                : bestPlanner.name;


            if (!fairBenchmarkMode) Debug.Log(
                "***** REPLACEMENT ACCEPTED *****"
            );


            if (!fairBenchmarkMode) Debug.Log(
                selectedRobotName +
                ": Path " +
                oldPathNumber +
                " -> Path " +
                (bestPathIndex + 1)
            );


            if (!fairBenchmarkMode) Debug.Log(
                "Tc: " +
                oldTc.ToString("F3") +
                " -> " +
                bestTc.ToString("F3") +
                " s"
            );


            // ==================================================
            // RECALCULATE FLEET AFTER REPLACEMENT
            // ==================================================

            currentResult =
                EvaluateFleet(
                    null,
                    false,
                    false
                );
        }


        // ==================================================
        // COMPLETE
        // ==================================================

        if (!fairBenchmarkMode) Debug.Log(
            "================================"
        );


        benchmarkFinalConflicts = currentResult.conflictCount;
        benchmarkFinalTc = currentResult.tc;

        if (!fairBenchmarkMode) Debug.Log(
            "SELECT / REPLACE COMPLETE"
        );


        if (!fairBenchmarkMode) Debug.Log(
            "Iterations = " +
            iteration
        );


        if (!fairBenchmarkMode) Debug.Log(
            "Final Conflicts = " +
            currentResult.conflictCount
        );


        if (!fairBenchmarkMode) Debug.Log(
            "Final Tc = " +
            currentResult.tc
                .ToString("F3") +
            " s"
        );


        if (!fairBenchmarkMode) Debug.Log(
            "===== FINAL SELECTED PATHS ====="
        );


        foreach (
            ImprovedAStar planner
            in planners
        )
        {
            if (planner == null)
                continue;


            string robotName =
                planner.robot != null
                ? planner.robot.name
                : planner.name;


            if (!fairBenchmarkMode) Debug.Log(
                robotName +
                " | Path " +
                planner
                    .GetSelectedPathNumber()
            );
        }


        if (!fairBenchmarkMode) Debug.Log(
            "================================"
        );
    }


    // ==================================================
    // NORMAL CON(R) OUTPUT
    // ==================================================

    [ContextMenu("Analyze Current Paths")]
    public void AnalyzeCurrentPaths()
    {
        ConflictResult result =
            EvaluateFleet(
                null,
                true,
                printIndividualConflicts
            );


        Debug.Log(
            "===== CON(R) RESULT ====="
        );


        Debug.Log(
            "Total Conflicts: " +
            result.conflictCount
        );


        Debug.Log(
            "Tc = " +
            result.tc.ToString("F3") +
            " s"
        );


        if (SimulationMetrics.Instance != null)
        {
            SimulationMetrics.Instance
                .RecordConflictScore(
                    result.tc
                );
        }


        Debug.Log(
            "========================="
        );
    }


    // ==================================================
    // EVALUATE FLEET
    //
    // overridePaths allows us to temporarily
    // test one alternative without actually
    // selecting that route.
    // ==================================================

    ConflictResult EvaluateFleet(
        Dictionary<
            ImprovedAStar,
            List<Node>
        > overridePaths,
        bool printPairResults,
        bool printIndividual)
    {
        ConflictResult result =
            new ConflictResult();


        List<TimedPath> timedPaths =
            new List<TimedPath>();


        // ==================================================
        // BUILD TIME COORDINATES
        // ==================================================

        foreach (
            ImprovedAStar planner
            in planners
        )
        {
            if (planner == null)
                continue;


            List<Node> path;


            if (
                overridePaths != null &&
                overridePaths.ContainsKey(
                    planner
                )
            )
            {
                path =
                    overridePaths[planner];
            }
            else
            {
                path =
                    planner.GetSelectedPath();
            }


            if (
                path == null ||
                path.Count == 0
            )
            {
                continue;
            }


            TimedPath timedPath =
                BuildTimedPath(
                    planner,
                    path
                );


            if (
                timedPath.nodes.Count > 0
            )
            {
                timedPaths.Add(
                    timedPath
                );
            }
        }


        // ==================================================
        // PRINT TIME COORDINATES
        // ==================================================

        if (printPairResults)
        {
            Debug.Log(
                "===== CON(R) TIME COORDINATES ====="
            );


            foreach (
                TimedPath timedPath
                in timedPaths
            )
            {
                Debug.Log(
                    GetRobotName(
                        timedPath
                    ) +
                    " | Path " +
                    timedPath.planner
                        .GetSelectedPathNumber() +
                    " | Travel Time: " +
                    timedPath.travelTime
                        .ToString("F2") +
                    " s"
                );
            }


            Debug.Log(
                "===== CON(R) CONFLICT ANALYSIS ====="
            );
        }


        // ==================================================
        // PAIRWISE CONFLICT DETECTION
        // ==================================================

        for (
            int i = 0;
            i < timedPaths.Count;
            i++
        )
        {
            for (
                int j = i + 1;
                j < timedPaths.Count;
                j++
            )
            {
                int pairConflictCount = 0;

                float pairTc = 0f;


                CheckNodeConflicts(
                    timedPaths[i],
                    timedPaths[j],
                    ref pairConflictCount,
                    ref pairTc,
                    printIndividual
                );


                CheckEdgeConflicts(
                    timedPaths[i],
                    timedPaths[j],
                    ref pairConflictCount,
                    ref pairTc,
                    printIndividual
                );


                result.conflictCount +=
                    pairConflictCount;


                result.tc +=
                    pairTc;


                if (
                    pairConflictCount > 0
                )
                {
                    result
                        .conflictingPlanners
                        .Add(
                            timedPaths[i]
                                .planner
                        );


                    result
                        .conflictingPlanners
                        .Add(
                            timedPaths[j]
                                .planner
                        );
                }


                if (printPairResults)
                {
                    Debug.Log(
                        GetRobotName(
                            timedPaths[i]
                        ) +
                        " vs " +
                        GetRobotName(
                            timedPaths[j]
                        ) +
                        " | Conflicts: " +
                        pairConflictCount +
                        " | Tc: " +
                        pairTc.ToString("F3") +
                        " s"
                    );
                }
            }
        }


        return result;
    }


    // ==================================================
    // BUILD TIME COORDINATES
    // ==================================================

    TimedPath BuildTimedPath(
        ImprovedAStar planner,
        List<Node> path)
    {
        TimedPath result =
            new TimedPath();


        result.planner =
            planner;


        if (planner.robot == null)
        {
            return result;
        }


        // ==================================================
        // USE THE ROBOT'S ACTUAL MOVEMENT SPEED
        // ==================================================

        RobotMovement movement =
            planner.robot
                .GetComponent<RobotMovement>();


        if (movement == null)
        {
            Debug.LogError(
                planner.robot.name +
                ": RobotMovement not found."
            );


            return result;
        }


        if (
            movement.moveSpeed <= 0f
        )
        {
            Debug.LogError(
                planner.robot.name +
                ": moveSpeed must be > 0."
            );


            return result;
        }


        float robotSpeed =
            movement.moveSpeed;


        List<float> arrivalTimes =
            new List<float>();


        float currentTime = 0f;


        arrivalTimes.Add(
            currentTime
        );


        // ==================================================
        // ARRIVAL TIMES
        // ==================================================

        for (
            int i = 1;
            i < path.Count;
            i++
        )
        {
            float distance =
                Vector3.Distance(
                    path[i - 1]
                        .worldPosition,
                    path[i]
                        .worldPosition
                );


            float travelTime =
                distance /
                robotSpeed;


            currentTime +=
                travelTime;


            arrivalTimes.Add(
                currentTime
            );
        }


        result.travelTime =
            currentTime;


        // ==================================================
        // NODE TIME WINDOWS
        // ==================================================

        for (
            int i = 0;
            i < path.Count;
            i++
        )
        {
            float previousTravelTime = 0f;

            float nextTravelTime = 0f;


            if (i > 0)
            {
                previousTravelTime =
                    arrivalTimes[i] -
                    arrivalTimes[i - 1];
            }


            if (
                i <
                path.Count - 1
            )
            {
                nextTravelTime =
                    arrivalTimes[i + 1] -
                    arrivalTimes[i];
            }


            if (
                i == 0 &&
                path.Count > 1
            )
            {
                previousTravelTime =
                    nextTravelTime;
            }


            if (
                i ==
                path.Count - 1 &&
                path.Count > 1
            )
            {
                nextTravelTime =
                    previousTravelTime;
            }


            TimedNode timedNode =
                new TimedNode();


            timedNode.node =
                path[i];


            timedNode.arrivalTime =
                arrivalTimes[i];


            timedNode.windowStart =
                Mathf.Max(
                    0f,
                    arrivalTimes[i] -
                    previousTravelTime / 2f
                );


            timedNode.windowEnd =
                arrivalTimes[i] +
                nextTravelTime / 2f;


            result.nodes.Add(
                timedNode
            );
        }


        // ==================================================
        // EDGE TIME WINDOWS
        // ==================================================

        for (
            int i = 0;
            i < path.Count - 1;
            i++
        )
        {
            TimedEdge edge =
                new TimedEdge();


            edge.from =
                path[i];


            edge.to =
                path[i + 1];


            edge.startTime =
                arrivalTimes[i];


            edge.endTime =
                arrivalTimes[i + 1];


            result.edges.Add(
                edge
            );
        }


        return result;
    }


    // ==================================================
    // NODE CONFLICTS
    // ==================================================

    void CheckNodeConflicts(
        TimedPath pathA,
        TimedPath pathB,
        ref int conflictCount,
        ref float tc,
        bool printIndividual)
    {
        foreach (
            TimedNode nodeA
            in pathA.nodes
        )
        {
            foreach (
                TimedNode nodeB
                in pathB.nodes
            )
            {
                if (
                    !SameGridNode(
                        nodeA.node,
                        nodeB.node
                    )
                )
                {
                    continue;
                }


                float overlap =
                    GetOverlap(
                        nodeA.windowStart,
                        nodeA.windowEnd,
                        nodeB.windowStart,
                        nodeB.windowEnd
                    );


                if (overlap <= 0f)
                {
                    continue;
                }


                conflictCount++;

                tc += overlap;


                if (printIndividual)
                {
                    Debug.Log(
                        "NODE CONFLICT | " +
                        GetRobotName(pathA) +
                        " vs " +
                        GetRobotName(pathB) +
                        " | Node (" +
                        nodeA.node.gridX +
                        "," +
                        nodeA.node.gridZ +
                        ")" +
                        " | overlap = " +
                        overlap.ToString("F3") +
                        " s"
                    );
                }
            }
        }
    }


    // ==================================================
    // EDGE CONFLICTS
    // ==================================================

    void CheckEdgeConflicts(
        TimedPath pathA,
        TimedPath pathB,
        ref int conflictCount,
        ref float tc,
        bool printIndividual)
    {
        foreach (
            TimedEdge edgeA
            in pathA.edges
        )
        {
            foreach (
                TimedEdge edgeB
                in pathB.edges
            )
            {
                bool oppositeDirection =
                    SameGridNode(
                        edgeA.from,
                        edgeB.to
                    )
                    &&
                    SameGridNode(
                        edgeA.to,
                        edgeB.from
                    );


                if (!oppositeDirection)
                {
                    continue;
                }


                float overlap =
                    GetOverlap(
                        edgeA.startTime,
                        edgeA.endTime,
                        edgeB.startTime,
                        edgeB.endTime
                    );


                if (overlap <= 0f)
                {
                    continue;
                }


                conflictCount++;

                tc += overlap;


                if (printIndividual)
                {
                    Debug.Log(
                        "EDGE CONFLICT | " +
                        GetRobotName(pathA) +
                        " vs " +
                        GetRobotName(pathB) +
                        " | (" +
                        edgeA.from.gridX +
                        "," +
                        edgeA.from.gridZ +
                        ") -> (" +
                        edgeA.to.gridX +
                        "," +
                        edgeA.to.gridZ +
                        ")" +
                        " | overlap = " +
                        overlap.ToString("F3") +
                        " s"
                    );
                }
            }
        }
    }


    // ==================================================
    // SAME GRID NODE
    // ==================================================

    bool SameGridNode(
        Node a,
        Node b)
    {
        return
            a.gridX == b.gridX &&
            a.gridZ == b.gridZ;
    }


    // ==================================================
    // TIME WINDOW OVERLAP
    // ==================================================

    float GetOverlap(
        float startA,
        float endA,
        float startB,
        float endB)
    {
        float start =
            Mathf.Max(
                startA,
                startB
            );


        float end =
            Mathf.Min(
                endA,
                endB
            );


        return Mathf.Max(
            0f,
            end - start
        );
    }


    // ==================================================
    // ROBOT NAME
    // ==================================================

    string GetRobotName(
        TimedPath path)
    {
        if (
            path.planner != null &&
            path.planner.robot != null
        )
        {
            return
                path.planner.robot.name;
        }


        if (
            path.planner != null
        )
        {
            return
                path.planner.name;
        }


        return "Unknown Robot";
    }
}