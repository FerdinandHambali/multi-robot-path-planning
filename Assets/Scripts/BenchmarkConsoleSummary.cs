using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using UnityEngine;

/// <summary>
/// OPTIONAL PRESENTATION-ONLY TOOL. Attach to one active GameObject in each
/// simulation scene (the SimulationMetrics GameObject is a good choice).
/// Suppresses normal Debug.Log messages during play, preserves warnings/errors,
/// and prints exactly one combined results block after SimulationMetrics finishes.
/// It does not affect pathfinding, robot movement, or metric computation.
///
/// Designed for the Fair_Planner_Benchmark_Fix versions of the planners and
/// the existing SimulationMetrics final report.
/// </summary>
[DefaultExecutionOrder(-10000)]
public sealed class BenchmarkConsoleSummary : MonoBehaviour
{
    [Header("Clean Console")]
    [Tooltip("Only display the final summary; errors and warnings are preserved.")]
    public bool enableCleanConsole = true;

    [Tooltip("Expected number of robots used for the summary label.")]
    [Min(1)]
    public int expectedRobotCount = 6;

    [Tooltip("Include planning-stage timing and CT node details in the final block.")]
    public bool includePlanningBreakdown = true;

    private static BenchmarkConsoleSummary activeInstance;
    private ILogHandler oldHandler;
    private CaptureHandler captureHandler;

    // Stored from existing logs; no new searches, metrics or timing performed.
    private readonly HashSet<string> finishedRobots = new HashSet<string>();
    private readonly Dictionary<string, int> pathsPerRobot = new Dictionary<string, int>();

    private string plannerName = "Unknown";
    private bool finalResultsStarted;
    private bool summaryPrinted;

    private double averageTravelTime = double.NaN;
    private double makespan = double.NaN;
    private double averageDistance = double.NaN;
    private double computationTime = double.NaN;
    private double conflictScore = double.NaN;
    private int collisions = -1;

    private int initialConflicts = -1;
    private int finalConflicts = -1;
    private int ctNodesExpanded = -1;
    private int solutionCost = -1;

    private double aStarDatabaseMs = double.NaN;
    private double aStarSelectReplaceMs = double.NaN;
    private double initialSTAStarMs = double.NaN;
    private double cbsMainThreadMs = double.NaN;
    private double cbsWorkerMs = double.NaN;
    private double pcbsObservedElapsedMs = double.NaN;

    private void Awake()
    {
        if (!enableCleanConsole) return;

        if (activeInstance != null && activeInstance != this)
        {
            Debug.LogWarning("BenchmarkConsoleSummary: multiple instances found; only the first will be used.");
            return;
        }

        activeInstance = this;
        oldHandler = Debug.unityLogger.logHandler;
        captureHandler = new CaptureHandler(this, oldHandler);
        Debug.unityLogger.logHandler = captureHandler;
    }

    private void OnDisable() { RestoreOriginalHandler(); }
    private void OnDestroy() { RestoreOriginalHandler(); }

    private void RestoreOriginalHandler()
    {
        if (activeInstance != this) return;

        if (captureHandler != null && Debug.unityLogger.logHandler == captureHandler)
            Debug.unityLogger.logHandler = oldHandler;

        captureHandler = null;
        activeInstance = null;
    }

    private sealed class CaptureHandler : ILogHandler
    {
        private readonly BenchmarkConsoleSummary summary;
        private readonly ILogHandler wrapped;

        public CaptureHandler(BenchmarkConsoleSummary owner, ILogHandler previous)
        {
            summary = owner;
            wrapped = previous;
        }

        public void LogFormat(LogType logType, UnityEngine.Object context, string format, params object[] args)
        {
            // Preserve warnings, errors, exceptions, and assertions.
            if (logType != LogType.Log)
            {
                wrapped.LogFormat(logType, context, format, args);
                return;
            }

            string message;
            try { message = string.Format(CultureInfo.InvariantCulture, format, args); }
            catch { message = format ?? string.Empty; }

            // Normal messages are intentionally not forwarded to Unity Console.
            // They are only inspected for the existing benchmark results.
            summary.ReadExistingLog(message);
        }

        public void LogException(Exception exception, UnityEngine.Object context)
        {
            wrapped.LogException(exception, context);
        }

        public void PrintOneSummary(string message)
        {
            // Bypass the filter so the final multiline block prints exactly once.
            wrapped.LogFormat(LogType.Log, null, "{0}", message);
        }
    }

    private void ReadExistingLog(string message)
    {
        if (summaryPrinted || string.IsNullOrEmpty(message)) return;
        message = message.Trim();

        if (message.StartsWith("FAIR A* |", StringComparison.Ordinal) ||
            message.StartsWith("FAIR A* SEARCH WORK", StringComparison.Ordinal))
            plannerName = "Improved A* + Select/Replace";

        if (message.Contains("===== FAIR PCBS COMPUTATION BENCHMARK =====") ||
            message.StartsWith("TOTAL FAIR PCBS SEARCH WORK", StringComparison.Ordinal))
            plannerName = "PCBS + Space-Time A*";

        // Robot counts and database sizes are available even when paths are not printed at the end.
        if (message.EndsWith(" reached the dock!", StringComparison.Ordinal))
        {
            string name = message.Substring(0, message.Length - " reached the dock!".Length).Trim();
            if (name.Length > 0) finishedRobots.Add(name);
        }

        string marker = " | Total Paths: ";
        int markerAt = message.IndexOf(marker, StringComparison.Ordinal);
        if (markerAt > 0)
        {
            string name = message.Substring(0, markerAt).Trim();
            int number;
            if (int.TryParse(message.Substring(markerAt + marker.Length).Trim(), out number))
                pathsPerRobot[name] = number;
        }

        if (message.StartsWith("FAIR A* | Initial Conflicts", StringComparison.Ordinal))
            initialConflicts = ReadIntAfter(message, "Initial Conflicts = ");
        if (message.StartsWith("FAIR A* | Final Conflicts", StringComparison.Ordinal))
            finalConflicts = ReadIntAfter(message, "Final Conflicts = ");

        if (message.StartsWith("Solution Cost = ", StringComparison.Ordinal))
        {
            solutionCost = ReadIntAfter(message, "Solution Cost = ");
            ctNodesExpanded = ReadIntAfter(message, "Expanded CT Nodes = ");
        }

        ReadNumberLine(message, "Total Path Planning Time = ", ref aStarDatabaseMs);
        ReadNumberLine(message, "Select / Replace Time = ", ref aStarSelectReplaceMs);
        ReadNumberLine(message, "Initial STA* Search Work = ", ref initialSTAStarMs);
        ReadNumberLine(message, "CBS Main-Thread Active Work = ", ref cbsMainThreadMs);
        ReadNumberLine(message, "CBS Parallel Worker Work (SUM) = ", ref cbsWorkerMs);
        ReadNumberLine(message, "PCBS Observed Unity Elapsed (contains frame waits) = ", ref pcbsObservedElapsedMs);

        if (message.Contains("===== FINAL SIMULATION RESULTS ====="))
        {
            finalResultsStarted = true;
            return;
        }

        if (!finalResultsStarted) return;

        ReadNumberLine(message, "Average Travel Time = ", ref averageTravelTime);
        ReadNumberLine(message, "Makespan = ", ref makespan);
        ReadNumberLine(message, "Average Travel Distance = ", ref averageDistance);
        ReadNumberLine(message, "Conflict Score Tc = ", ref conflictScore);

        if (message.StartsWith("Collision Count = ", StringComparison.Ordinal))
            collisions = ReadIntAfter(message, "Collision Count = ");

        if (message.StartsWith("Computation Time = ", StringComparison.Ordinal))
        {
            ReadNumberLine(message, "Computation Time = ", ref computationTime);
            PrintFinalSummary();
        }
    }

    private static void ReadNumberLine(string message, string startsWith, ref double destination)
    {
        if (!message.StartsWith(startsWith, StringComparison.Ordinal)) return;

        string tail = message.Substring(startsWith.Length);
        int end = 0;
        while (end < tail.Length &&
               (char.IsDigit(tail[end]) || tail[end] == '-' || tail[end] == '+' || tail[end] == '.'))
            end++;

        double parsed;
        if (end > 0 && double.TryParse(tail.Substring(0, end),
              NumberStyles.Float, CultureInfo.InvariantCulture, out parsed))
            destination = parsed;
    }

    private static int ReadIntAfter(string message, string marker)
    {
        int start = message.IndexOf(marker, StringComparison.Ordinal);
        if (start < 0) return -1;
        start += marker.Length;
        int end = start;
        while (end < message.Length && char.IsDigit(message[end])) end++;
        int result;
        return end > start && int.TryParse(message.Substring(start, end - start), out result)
            ? result : -1;
    }

    private static string Format(double number, string unit)
    {
        return double.IsNaN(number) ? "N/A" :
            number.ToString("F3", CultureInfo.InvariantCulture) + " " + unit;
    }

    private void PrintFinalSummary()
    {
        if (summaryPrinted) return;
        summaryPrinted = true;

        bool isPCBS = plannerName.StartsWith("PCBS", StringComparison.Ordinal);
        StringBuilder text = new StringBuilder(700);
        text.AppendLine("========================================");
        text.AppendLine("          FINAL FYP RESULTS");
        text.AppendLine("========================================");
        text.AppendLine("Algorithm: " + plannerName);
        text.AppendLine("Robots completed: " + finishedRobots.Count + "/" + expectedRobotCount);
        text.AppendLine();
        text.AppendLine("Average travel time: " + Format(averageTravelTime, "s"));
        text.AppendLine("Average travel distance: " + Format(averageDistance, "Unity units"));
        text.AppendLine("Makespan: " + Format(makespan, "s"));
        text.AppendLine("Actual collisions: " + (collisions >= 0 ? collisions.ToString() : "N/A"));
        text.AppendLine("Predicted conflicts: " +
            (isPCBS ? "0 (conflict-free CBS solution)" :
             finalConflicts >= 0 ? finalConflicts.ToString() : "N/A"));
        text.AppendLine("Conflict score Tc: " +
            (isPCBS ? "N/A (not independently calculated)" : Format(conflictScore, "s")));
        text.AppendLine("CT nodes expanded: " +
            (isPCBS && ctNodesExpanded >= 0 ? ctNodesExpanded.ToString() : "N/A (Improved A*)"));
        text.AppendLine();
        text.AppendLine("Total search work: " + Format(computationTime, "ms"));

        if (includePlanningBreakdown)
        {
            if (isPCBS)
            {
                text.AppendLine("  Initial STA* searches: " + Format(initialSTAStarMs, "ms"));
                text.AppendLine("  CBS main-thread work: " + Format(cbsMainThreadMs, "ms"));
                text.AppendLine("  CBS worker work (sum): " + Format(cbsWorkerMs, "ms"));
                text.AppendLine("  Unity elapsed (incl. waits): " + Format(pcbsObservedElapsedMs, "ms"));
                if (solutionCost >= 0) text.AppendLine("  CBS solution cost (x10): " + solutionCost);
            }
            else
            {
                text.AppendLine("  Path database generation: " + Format(aStarDatabaseMs, "ms"));
                text.AppendLine("  Select/Replace: " + Format(aStarSelectReplaceMs, "ms"));
                if (pathsPerRobot.Count > 0)
                {
                    int totalStoredPaths = 0;
                    foreach (int count in pathsPerRobot.Values) totalStoredPaths += count;
                    text.AppendLine("  Database paths stored: " + totalStoredPaths + " across "
                        + pathsPerRobot.Count + " robots");
                }
                if (initialConflicts >= 0)
                    text.AppendLine("  Conflicts before Select/Replace: " + initialConflicts);
            }
        }

        text.AppendLine("========================================");
        if (captureHandler != null) captureHandler.PrintOneSummary(text.ToString());
    }
}
