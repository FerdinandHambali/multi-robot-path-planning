using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;

/// <summary>
/// Shared time-based movement executor for Improved A* and PCBS + STA*.
/// Improved A*: each segment duration = distance / moveSpeed.
/// PCBS: follows its reserved timestep schedule, including genuine waits.
/// Both sample position against ONE absolute movement clock, rather than
/// accumulating Time.deltaTime / yielding one extra frame per waypoint.
/// </summary>
public class RobotMovement : MonoBehaviour
{
    private bool hasFinished = false;
    public RobotPath robotPath;

    [Header("Movement Mode")]
    public bool useDiscreteTimeExecution = true;

    [Header("PCBS / STA* Settings")]
    public float timePerStep = 0.1f;

    [Header("Speed Based Settings")]
    public float moveSpeed = 2f;

    private List<Node> activePath;
    private int pathIndex = 0;

    private struct TimedSegment
    {
        public Vector3 from;
        public Vector3 to;
        public double begin;
        public double end;
        public int arrivalPathIndex;

        public TimedSegment(Vector3 a, Vector3 b, double start, double finish, int index)
        {
            from = a;
            to = b;
            begin = start;
            end = finish;
            arrivalPathIndex = index;
        }
    }

    private readonly List<TimedSegment> segments = new List<TimedSegment>();
    private int currentSegment = 0;
    private double scheduleStartTime;
    private double totalScheduledTime;
    private double timeBeforeReplans;

    IEnumerator Start()
    {
        if (robotPath == null)
        {
            Debug.LogError(gameObject.name + ": RobotPath reference is not assigned.");
            yield break;
        }

        // Use the same global movement start as SimulationMetrics.
        while (robotPath.path == null ||
               robotPath.path.Count == 0 ||
               !robotPath.routeFinalized ||
               (SimulationMetrics.Instance != null && !SimulationMetrics.Instance.MovementStarted))
        {
            yield return null;
        }

        if (moveSpeed <= 0f)
        {
            Debug.LogError(gameObject.name + ": moveSpeed must be greater than zero.");
            yield break;
        }
        if (useDiscreteTimeExecution && timePerStep <= 0f)
        {
            Debug.LogError(gameObject.name + ": timePerStep must be greater than zero.");
            yield break;
        }

        activePath = robotPath.path;
        hasFinished = false;
        timeBeforeReplans = 0.0;
        pathIndex = 0;

        // Both planners start from their first actual grid node.
        Node firstNode = activePath[0];
        if (firstNode == null)
        {
            Debug.LogError(gameObject.name + ": first path node is null.");
            yield break;
        }
        transform.position = AtRobotHeight(firstNode.worldPosition);

        BuildSchedule(false);
        if (SimulationMetrics.Instance != null)
            scheduleStartTime = SimulationMetrics.Instance.MovementStartTime;
        else
            scheduleStartTime = Time.timeAsDouble;

        Debug.Log(gameObject.name + " shared movement started | Mode = " +
                  (useDiscreteTimeExecution ? "PCBS timed" : "Improved A* speed") +
                  " | Planned travel time = " + totalScheduledTime.ToString("F3") + " s");

        while (true)
        {
            if (useDiscreteTimeExecution)
            {
                if (robotPath.path != null && robotPath.path != activePath)
                {
                    Debug.LogError(gameObject.name + ": PCBS path changed during execution.");
                    yield break;
                }
            }
            else if (robotPath.path != null && robotPath.path != activePath)
            {
                // Preserve dynamic A* path replacement, and reschedule remaining
                // movement from the current physical position at the current time.
                timeBeforeReplans += Math.Max(0.0, Time.timeAsDouble - scheduleStartTime);
                activePath = robotPath.path;
                pathIndex = 0;
                BuildSchedule(true);
                scheduleStartTime = Time.timeAsDouble;
                Debug.Log(gameObject.name + " switched to new path.");
            }

            double elapsed = Math.Max(0.0, Time.timeAsDouble - scheduleStartTime);

            // Consume completed segments without inserting a frame of delay
            // between two consecutive nodes.
            while (currentSegment < segments.Count &&
                   elapsed >= segments[currentSegment].end)
            {
                TimedSegment finished = segments[currentSegment];
                transform.position = finished.to;
                pathIndex = finished.arrivalPathIndex;
                currentSegment++;
            }

            if (currentSegment >= segments.Count)
            {
                double plannedTime = timeBeforeReplans + totalScheduledTime;
                if (SimulationMetrics.Instance != null)
                    SimulationMetrics.Instance.RecordPlannedTravelTime(gameObject.name, (float)plannedTime);

                FinishRobot();
                yield break;
            }

            TimedSegment segment = segments[currentSegment];
            double duration = segment.end - segment.begin;
            float fraction = duration > 0.0
                ? Mathf.Clamp01((float)((elapsed - segment.begin) / duration))
                : 1f;

            transform.position = Vector3.Lerp(segment.from, segment.to, fraction);
            yield return null;
        }
    }

    private Vector3 AtRobotHeight(Vector3 point)
    {
        return new Vector3(point.x, transform.position.y, point.z);
    }

    private void AddSegment(Vector3 from, Vector3 to, double duration, int arrivalIndex)
    {
        if (duration < 0.0)
            throw new InvalidOperationException("Negative movement duration.");

        double end = totalScheduledTime + duration;
        segments.Add(new TimedSegment(from, to, totalScheduledTime, end, arrivalIndex));
        totalScheduledTime = end;
    }

    private void BuildSchedule(bool resumeFromCurrentPosition)
    {
        segments.Clear();
        currentSegment = 0;
        totalScheduledTime = 0.0;

        if (activePath == null || activePath.Count == 0)
            throw new InvalidOperationException(gameObject.name + ": empty final path.");

        if (!useDiscreteTimeExecution)
        {
            Vector3 previous = transform.position;
            int initialIndex = resumeFromCurrentPosition ? 0 : 1;
            for (int i = initialIndex; i < activePath.Count; i++)
            {
                Vector3 next = AtRobotHeight(activePath[i].worldPosition);
                double duration = Vector3.Distance(previous, next) / moveSpeed;
                AddSegment(previous, next, duration, i);
                previous = next;
            }
            return;
        }

        // PCBS trajectory may repeat a node to encode a wait or reserve
        // multiple time slots for a slow move. Retain this original rule:
        // wait first (if needed), then move within the reserved slots.
        int index = 0;
        Vector3 location = transform.position;
        while (index < activePath.Count - 1)
        {
            Node current = activePath[index];
            int nextDifferent = index + 1;
            while (nextDifferent < activePath.Count &&
                   activePath[nextDifferent].gridX == current.gridX &&
                   activePath[nextDifferent].gridZ == current.gridZ)
                nextDifferent++;

            if (nextDifferent >= activePath.Count)
            {
                int finalWaitSlots = activePath.Count - 1 - index;
                if (finalWaitSlots > 0)
                    AddSegment(location, location, finalWaitSlots * (double)timePerStep,
                               activePath.Count - 1);
                break;
            }

            Vector3 destination = AtRobotHeight(activePath[nextDifferent].worldPosition);
            int slots = nextDifferent - index;
            double physicalTime = Vector3.Distance(location, destination) / moveSpeed;
            // Small epsilon protects exact integer ratios against float noise.
            int requiredSlots = Math.Max(1,
                (int)Math.Ceiling(physicalTime / timePerStep - 0.00001));
            int waitSlots = slots - requiredSlots;
            if (waitSlots < 0)
                throw new InvalidOperationException(
                    gameObject.name + ": PCBS timing mismatch; available=" + slots +
                    ", required=" + requiredSlots + ". Check speed and timestep settings.");

            if (waitSlots > 0)
            {
                AddSegment(location, location, waitSlots * (double)timePerStep, index);
                Debug.Log(gameObject.name + " | PCBS WAIT | Slots = " + waitSlots +
                          " | Duration = " + (waitSlots * timePerStep).ToString("F3") + " s");
            }
            AddSegment(location, destination, requiredSlots * (double)timePerStep,
                       nextDifferent);
            index = nextDifferent;
            location = destination;
        }

        double expected = (activePath.Count - 1) * (double)timePerStep;
        if (Math.Abs(expected - totalScheduledTime) > 0.0001)
            Debug.LogError(gameObject.name + ": scheduled duration mismatch; expected=" +
                           expected.ToString("F4") + ", got=" + totalScheduledTime.ToString("F4"));
    }

    private void FinishRobot()
    {
        if (hasFinished) return;
        hasFinished = true;
        Debug.Log(gameObject.name + " reached the dock!");
        if (SimulationMetrics.Instance != null)
            SimulationMetrics.Instance.RobotFinished(gameObject.name);
    }

    // Physical collision monitoring is UNCHANGED from the original script.
    void OnTriggerEnter(Collider other)
    {
        RobotMovement otherRobot = other.GetComponentInParent<RobotMovement>();
        if (otherRobot == null || otherRobot == this) return;
        if (string.CompareOrdinal(gameObject.name, otherRobot.gameObject.name) >= 0)
            return;

        float centerDistance = Vector3.Distance(transform.position,
                                                 otherRobot.transform.position);
        Debug.Log("COLLISION DEBUG | " + gameObject.name + " vs " +
                  otherRobot.gameObject.name + " | Distance = " +
                  centerDistance.ToString("F3") + " | A Path Index = " + pathIndex +
                  " | B Path Index = " + otherRobot.pathIndex +
                  " | Time = " + Time.time.ToString("F3") +
                  " | A Position = " + transform.position +
                  " | B Position = " + otherRobot.transform.position);
        if (SimulationMetrics.Instance != null)
            SimulationMetrics.Instance.RecordCollision(gameObject.name,
                                                       otherRobot.gameObject.name);
    }
}
