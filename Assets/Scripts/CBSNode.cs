using System.Collections.Generic;

public class CBSNode
{
    // ==================================================
    // CONSTRAINTS FOR EACH ROBOT
    // ==================================================

    public List<List<SpaceTimeVertexConstraint>>
        vertexConstraints =
            new List<List<SpaceTimeVertexConstraint>>();


    public List<List<SpaceTimeEdgeConstraint>>
        edgeConstraints =
            new List<List<SpaceTimeEdgeConstraint>>();


    // ==================================================
    // PATH FOR EACH ROBOT
    // ==================================================

    public List<List<SpaceTimeNode>>
        paths =
            new List<List<SpaceTimeNode>>();


    // ==================================================
    // CT NODE COST
    // ==================================================

    public int cost;


    // ==================================================
    // CONSTRUCTOR
    // ==================================================

    public CBSNode(
        int robotCount)
    {
        for (
            int i = 0;
            i < robotCount;
            i++
        )
        {
            vertexConstraints.Add(
                new List<SpaceTimeVertexConstraint>()
            );

            edgeConstraints.Add(
                new List<SpaceTimeEdgeConstraint>()
            );

            paths.Add(
                new List<SpaceTimeNode>()
            );
        }
    }


    // ==================================================
    // COPY CONSTRUCTOR
    //
    // We will use this in the next stage
    // when creating CT children.
    // ==================================================

    public CBSNode(
        CBSNode parent)
    {
        cost =
            parent.cost;


        for (
            int i = 0;
            i < parent.paths.Count;
            i++
        )
        {
            // ==============================
            // COPY VERTEX CONSTRAINTS
            // ==============================

            List<SpaceTimeVertexConstraint>
                vertexCopy =
                    new List<SpaceTimeVertexConstraint>();


            foreach (
                SpaceTimeVertexConstraint constraint
                in parent.vertexConstraints[i]
            )
            {
                vertexCopy.Add(
                    new SpaceTimeVertexConstraint(
                        constraint.gridX,
                        constraint.gridZ,
                        constraint.time
                    )
                );
            }


            vertexConstraints.Add(
                vertexCopy
            );


            // ==============================
            // COPY EDGE CONSTRAINTS
            // ==============================

            List<SpaceTimeEdgeConstraint>
                edgeCopy =
                    new List<SpaceTimeEdgeConstraint>();


            foreach (
                SpaceTimeEdgeConstraint constraint
                in parent.edgeConstraints[i]
            )
            {
                edgeCopy.Add(
                    new SpaceTimeEdgeConstraint(
                        constraint.fromX,
                        constraint.fromZ,
                        constraint.toX,
                        constraint.toZ,
                        constraint.time
                    )
                );
            }


            edgeConstraints.Add(
                edgeCopy
            );


            // ==============================
            // COPY PATH
            // ==============================

            paths.Add(
                new List<SpaceTimeNode>(
                    parent.paths[i]
                )
            );
        }
    }
}