using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class SpaceTimeConflictDetector : MonoBehaviour
{
    // ==================================================
    // PLANNERS
    // ==================================================

    private SpaceTimeAStar plannerA;
    private SpaceTimeAStar plannerB;


    // ==================================================
    // START
    // ==================================================

    IEnumerator Start()
    {
        // Get the two STA* planners attached
        // to this same GameObject.

        SpaceTimeAStar[] planners =
            GetComponents<SpaceTimeAStar>();


        if (
            planners == null ||
            planners.Length < 2
        )
        {
            Debug.LogError(
                "ConflictDetector requires at least " +
                "two SpaceTimeAStar components."
            );

            yield break;
        }


        plannerA =
            planners[0];

        plannerB =
            planners[1];


        // ==================================================
        // WAIT UNTIL BOTH STA* SEARCHES FINISH
        // ==================================================

        while (
            plannerA.finalPath == null ||
            plannerA.finalPath.Count == 0 ||

            plannerB.finalPath == null ||
            plannerB.finalPath.Count == 0
        )
        {
            yield return null;
        }


        Debug.Log(
            "===== AUTOMATIC CONFLICT DETECTION ====="
        );


        DetectConflicts();
    }


    // ==================================================
    // DETECT CONFLICTS
    // ==================================================

    void DetectConflicts()
    {
        List<SpaceTimeNode> pathA =
            plannerA.finalPath;

        List<SpaceTimeNode> pathB =
            plannerB.finalPath;


        // ==================================================
        // GOAL OCCUPANCY ASSUMPTION
        //
        // Once a robot reaches its goal,
        // it stays there.
        //
        // Therefore we compare until the longer
        // of the two paths finishes.
        // ==================================================

        int maxTime =
            Mathf.Max(
                pathA.Count,
                pathB.Count
            ) - 1;


        int vertexConflicts = 0;

        int edgeConflicts = 0;


        // ==================================================
        // CHECK EVERY TIMESTEP
        // ==================================================

        for (
            int time = 0;
            time <= maxTime;
            time++
        )
        {
            Node nodeA =
                GetNodeAtTime(
                    pathA,
                    time
                );

            Node nodeB =
                GetNodeAtTime(
                    pathB,
                    time
                );


            // ==================================================
            // VERTEX CONFLICT
            //
            // Robot A and Robot B occupy
            // the same node at the same time.
            // ==================================================

            if (
                nodeA == nodeB
            )
            {
                vertexConflicts++;


                Debug.Log(
                    "VERTEX CONFLICT" +
                    " | t=" +
                    time +
                    " | Node=(" +
                    nodeA.gridX +
                    "," +
                    nodeA.gridZ +
                    ")"
                );
            }


            // ==================================================
            // EDGE CONFLICT
            //
            // Need t + 1, so stop edge checking
            // after the final timestep.
            // ==================================================

            if (
                time >= maxTime
            )
            {
                continue;
            }


            Node nextA =
                GetNodeAtTime(
                    pathA,
                    time + 1
                );


            Node nextB =
                GetNodeAtTime(
                    pathB,
                    time + 1
                );


            bool robotAMoved =
                nodeA != nextA;


            bool robotBMoved =
                nodeB != nextB;


            // ==================================================
            // A -> B
            // B -> A
            //
            // during the same time interval.
            // ==================================================

            if (
                robotAMoved &&
                robotBMoved &&

                nodeA == nextB &&
                nodeB == nextA
            )
            {
                edgeConflicts++;


                Debug.Log(
                    "EDGE CONFLICT" +
                    " | t=" +
                    time +
                    " -> " +
                    (time + 1) +
                    " | (" +
                    nodeA.gridX +
                    "," +
                    nodeA.gridZ +
                    ") <-> (" +
                    nodeB.gridX +
                    "," +
                    nodeB.gridZ +
                    ")"
                );
            }
        }


        // ==================================================
        // RESULT
        // ==================================================

        Debug.Log(
            "===== CONFLICT RESULT ====="
        );


        Debug.Log(
            "Vertex Conflicts = " +
            vertexConflicts
        );


        Debug.Log(
            "Edge Conflicts = " +
            edgeConflicts
        );


        if (
            vertexConflicts == 0 &&
            edgeConflicts == 0
        )
        {
            Debug.Log(
                "NO CONFLICTS FOUND."
            );
        }
        else
        {
            Debug.Log(
                "CONFLICT DETECTED."
            );
        }
    }


    // ==================================================
    // GET ROBOT POSITION AT A TIMESTEP
    // ==================================================

    Node GetNodeAtTime(
        List<SpaceTimeNode> path,
        int time)
    {
        // Robot has not finished its path yet.

        if (
            time < path.Count
        )
        {
            return
                path[time].node;
        }


        // ==================================================
        // ROBOT ALREADY REACHED GOAL
        //
        // Our implementation assumption:
        //
        // remain at final goal forever.
        // ==================================================

        return
            path[
                path.Count - 1
            ].node;
    }
}