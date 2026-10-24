using UnityEngine;
using System.Collections.Generic;

public class SimulationMetrics : MonoBehaviour
{
    public static SimulationMetrics Instance;


    // ==================================================
    // STORED METRICS
    // ==================================================

    private Dictionary<string, float> robotDistances =
        new Dictionary<string, float>();

    private Dictionary<string, float> robotFinishTimes =
        new Dictionary<string, float>();


    private float simulationStartTime;

    private bool simulationStarted = false;

    // Shared absolute movement start for both planners.
    public bool MovementStarted { get { return simulationStarted; } }
    public double MovementStartTime { get { return simulationStartTime; } }

    // Ideal travel durations provided by the common movement executor.
    private Dictionary<string, float> plannedFinishTimes =
        new Dictionary<string, float>();


    // Final metrics
    private float averageDistance = 0f;

    private int collisionCount = 0;

    private float finalTc = 0f;

    private double computationTimeMs = 0.0;


    void Awake()
    {
        Instance = this;
    }


    // ==================================================
    // MILESTONE 4D
    // COMPUTATION TIME
    // ==================================================

    public void RecordPlannedTravelTime(string robotName, float seconds)
    {
        if (!simulationStarted) return;
        plannedFinishTimes[robotName] = seconds;
        Debug.Log(robotName + " Planned Travel Time = " + seconds.ToString("F3") + " s");
    }

    public void RecordComputationTime(
        double milliseconds)
    {
        computationTimeMs =
            milliseconds;


        Debug.Log(
            "Computation Time = " +
            computationTimeMs.ToString("F3") +
            " ms"
        );
    }


    // ==================================================
    // MILESTONE 4B
    // FINAL PATH DISTANCE
    // ==================================================

    public void RecordFinalPaths(
        List<ImprovedAStar> planners)
    {
        robotDistances.Clear();

        averageDistance = 0f;

        float totalDistance = 0f;


        Debug.Log(
            "================================"
        );

        Debug.Log(
            "MILESTONE 4B - TRAVEL DISTANCE"
        );

        Debug.Log(
            "================================"
        );


        foreach (
            ImprovedAStar planner
            in planners
        )
        {
            if (
                planner == null ||
                planner.robot == null
            )
            {
                continue;
            }


            List<Node> path =
                planner.GetSelectedPath();


            if (
                path == null ||
                path.Count == 0
            )
            {
                continue;
            }


            float distance = 0f;


            for (
                int i = 0;
                i < path.Count - 1;
                i++
            )
            {
                distance +=
                    Vector3.Distance(
                        path[i].worldPosition,
                        path[i + 1].worldPosition
                    );
            }


            string robotName =
                planner.robot.name;


            robotDistances[
                robotName
            ] = distance;


            totalDistance +=
                distance;


            Debug.Log(
                robotName +
                " Travel Distance = " +
                distance.ToString("F3")
            );
        }


        if (
            robotDistances.Count > 0
        )
        {
            averageDistance =
                totalDistance /
                robotDistances.Count;


            Debug.Log(
                "Average Travel Distance = " +
                averageDistance.ToString("F3")
            );
        }


        Debug.Log(
            "================================"
        );
    }

    // ==================================================
    // MILESTONE 7A
    // PCBS + STA* FINAL TRAVEL DISTANCE
    // ==================================================

    public void RecordFinalPCBSPaths(
        SpaceTimeAStar[] planners)
    {
        robotDistances.Clear();
        averageDistance = 0f;

        float totalDistance = 0f;

        Debug.Log(
            "================================"
        );

        Debug.Log(
            "PCBS + STA* - TRAVEL DISTANCE"
        );

        Debug.Log(
            "================================"
        );

        foreach (SpaceTimeAStar planner in planners)
        {
            if (
                planner == null ||
                planner.robot == null ||
                planner.finalPath == null ||
                planner.finalPath.Count == 0
            )
            {
                continue;
            }

            List<SpaceTimeNode> path =
                planner.finalPath;

            float distance = 0f;

            for (int i = 0; i < path.Count - 1; i++)
            {
                Node currentNode = path[i].node;
                Node nextNode = path[i + 1].node;

                if (currentNode == null || nextNode == null)
                {
                    Debug.LogError(
                        planner.robot.name +
                        " has an invalid path node."
                    );
                    return;
                }

                distance += Vector3.Distance(
                    currentNode.worldPosition,
                    nextNode.worldPosition
                );
            }

            string robotName = planner.robot.name;

            robotDistances[robotName] = distance;
            totalDistance += distance;

            Debug.Log(
                robotName +
                " Travel Distance = " +
                distance.ToString("F3") +
                " m"
            );
        }

        if (robotDistances.Count > 0)
        {
            averageDistance =
                totalDistance / robotDistances.Count;

            Debug.Log(
                "Average Travel Distance = " +
                averageDistance.ToString("F3") +
                " m"
            );
        }
        else
        {
            Debug.LogError(
                "PCBS: No valid paths for distance measurement."
            );
        }

        Debug.Log(
            "================================"
        );
    }


    // ==================================================
    // MILESTONE 4C
    // CONFLICT SCORE
    // ==================================================

    public void RecordConflictScore(
        float tc)
    {
        finalTc = tc;


        Debug.Log(
            "Conflict Score Tc = " +
            finalTc.ToString("F3") +
            " s"
        );
    }


    // ==================================================
    // MILESTONE 4C
    // REAL-TIME COLLISION COUNT
    // ==================================================

    public void RecordCollision(
        string robotA,
        string robotB)
    {
        if (!simulationStarted)
        {
            return;
        }


        collisionCount++;


        Debug.Log(
            "COLLISION | " +
            robotA +
            " vs " +
            robotB +
            " | Total Collisions = " +
            collisionCount
        );
    }


    // ==================================================
    // START MOVEMENT TIMER
    // ==================================================

    public void StartSimulation()
    {
        simulationStartTime =
            Time.time;


        simulationStarted =
            true;


        robotFinishTimes.Clear();
        plannedFinishTimes.Clear();


        collisionCount =
            0;


        Debug.Log(
            "===== MOVEMENT TIMER STARTED ====="
        );
    }


    // ==================================================
    // ROBOT FINISHED
    // ==================================================

    public void RobotFinished(
        string robotName)
    {
        if (!simulationStarted)
        {
            return;
        }


        // Prevent duplicate finish recording.
        if (
            robotFinishTimes.ContainsKey(
                robotName
            )
        )
        {
            return;
        }


        float travelTime =
            Time.time -
            simulationStartTime;


        robotFinishTimes.Add(
            robotName,
            travelTime
        );


        Debug.Log(
            robotName +
            " Travel Time = " +
            travelTime.ToString("F3") +
            " s"
        );


        // Current experiment has 6 robots.
        if (
            robotFinishTimes.Count == 6
        )
        {
            PrintFinalResults();
        }
    }


    // ==================================================
    // MILESTONE 4E
    // FINAL SIMULATION RESULTS
    // ==================================================

    void PrintFinalResults()
    {
        float totalTravelTime =
            0f;


        float makespan =
            0f;


        foreach (
            KeyValuePair<string, float>
            result
            in robotFinishTimes
        )
        {
            totalTravelTime +=
                result.Value;


            if (
                result.Value >
                makespan
            )
            {
                makespan =
                    result.Value;
            }
        }


        float averageTravelTime =
            totalTravelTime /
            robotFinishTimes.Count;


        // Report planned durations as the PRIMARY comparable movement metrics.
        // Observed arrival frame time remains available separately for debugging.
        float observedAverage = averageTravelTime;
        float observedMakespan = makespan;
        bool allPlanned = plannedFinishTimes.Count == robotFinishTimes.Count;
        if (allPlanned)
        {
            float totalPlanned = 0f;
            float maxPlanned = 0f;
            foreach (KeyValuePair<string, float> pair in plannedFinishTimes)
            {
                totalPlanned += pair.Value;
                if (pair.Value > maxPlanned) maxPlanned = pair.Value;
            }
            averageTravelTime = totalPlanned / plannedFinishTimes.Count;
            makespan = maxPlanned;
        }
        else
        {
            Debug.LogWarning("Some planned travel durations are missing; reporting observed time.");
        }

        // Do not use the same key names as the primary metrics: the existing
        // BenchmarkConsoleSummary should continue reading the planned results.
        Debug.Log("Travel time basis = planned movement (distance/speed + PCBS waits)");
        Debug.Log("Unity-frame mean arrival = " + observedAverage.ToString("F3") + " s");
        Debug.Log("Unity-frame last arrival = " + observedMakespan.ToString("F3") + " s");

        Debug.Log(
            "================================"
        );

        Debug.Log(
            "===== FINAL SIMULATION RESULTS ====="
        );

        Debug.Log(
            "================================"
        );


        Debug.Log(
            "Average Travel Time = " +
            averageTravelTime.ToString("F3") +
            " s"
        );


        Debug.Log(
            "Makespan = " +
            makespan.ToString("F3") +
            " s"
        );


        Debug.Log(
            "Average Travel Distance = " +
            averageDistance.ToString("F3") +
            " Unity units"
        );


        Debug.Log(
            "Collision Count = " +
            collisionCount
        );


        Debug.Log(
            "Conflict Score Tc = " +
            finalTc.ToString("F3") +
            " s"
        );


        Debug.Log(
            "Computation Time = " +
            computationTimeMs.ToString("F3") +
            " ms"
        );


        Debug.Log(
            "================================"
        );
    }
}