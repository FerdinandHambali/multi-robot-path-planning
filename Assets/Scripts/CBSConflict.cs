public enum CBSConflictType
{
    Vertex,
    Edge
}


public class CBSConflict
{
    public CBSConflictType type;

    public int robotA;
    public int robotB;

    public int time;


    // ==============================
    // VERTEX CONFLICT
    // ==============================

    public int vertexX;
    public int vertexZ;


    // ==============================
    // EDGE CONFLICT
    // ==============================

    public int aFromX;
    public int aFromZ;

    public int aToX;
    public int aToZ;

    public int bFromX;
    public int bFromZ;

    public int bToX;
    public int bToZ;


    // ==================================================
    // CREATE VERTEX CONFLICT
    // ==================================================

    public static CBSConflict CreateVertex(
        int robotA,
        int robotB,
        int x,
        int z,
        int time)
    {
        CBSConflict conflict =
            new CBSConflict();

        conflict.type =
            CBSConflictType.Vertex;

        conflict.robotA =
            robotA;

        conflict.robotB =
            robotB;

        conflict.vertexX =
            x;

        conflict.vertexZ =
            z;

        conflict.time =
            time;

        return conflict;
    }


    // ==================================================
    // CREATE EDGE CONFLICT
    // ==================================================

    public static CBSConflict CreateEdge(
        int robotA,
        int robotB,
        int aFromX,
        int aFromZ,
        int aToX,
        int aToZ,
        int bFromX,
        int bFromZ,
        int bToX,
        int bToZ,
        int time)
    {
        CBSConflict conflict =
            new CBSConflict();

        conflict.type =
            CBSConflictType.Edge;

        conflict.robotA =
            robotA;

        conflict.robotB =
            robotB;

        conflict.aFromX =
            aFromX;

        conflict.aFromZ =
            aFromZ;

        conflict.aToX =
            aToX;

        conflict.aToZ =
            aToZ;

        conflict.bFromX =
            bFromX;

        conflict.bFromZ =
            bFromZ;

        conflict.bToX =
            bToX;

        conflict.bToZ =
            bToZ;

        conflict.time =
            time;

        return conflict;
    }
}