import { step, dt } from "./main_crane.js"; // Assuming these are defined in simulation.js

export let QLog = [], ReactionLog = [], PerformanceLog = [], UnifiedLog = [];
export let QHeader = null, ReactionHeader = ["loadcell"], PerformanceHeader = ["performance_ms"], unifiedHeader = null;
export let csvtime = 0;

export function logQStep(Q, system) { // Log Q coordinates
  if (!QHeader) {
    const names = system.Qcoordinates.flat().map(q => q.vars);
    QHeader = ["time", ...names];
  }

  QLog.push([csvtime, ...Q.flat()]);
  csvtime = step * dt; // assuming `step` is defined outside and increasing
}

export function logReactions(reactions) { // Log reaction forces
  if (!ReactionHeader) {
    ReactionHeader = reactions.map((_, i) => `reaction_${i}`);
  }
  ReactionLog.push(reactions);
}

export function logPerformance(stepTimeMs) { // Log performance time
  PerformanceLog.push([stepTimeMs]);

}

export function logStep(Q, reactions, performanceMs, system) {
  if (!unifiedHeader) {
    const qNames = system.Qcoordinates.flat().map(q => q.vars);
    const reactionNames = reactions.map((_, i) => `reaction_${i}`);
    unifiedHeader = ["time", ...qNames, ...reactionNames, "step_time_ms"];
    UnifiedLog.push(unifiedHeader);
  }

  const row = [
    csvtime,
    ...Q.flat(),
    ...reactions,
    performanceMs
  ];
  UnifiedLog.push(row);
  csvtime = step * dt; // assuming step, dt defined elsewhere
}

export function getQLogCSV() {
  return [QHeader.join(","), ...QLog.map(row => row.join(","))].join("\n");
}

export function getReactionLogCSV() {
  return [ReactionHeader.join(","), ...ReactionLog.map(row => row.join(","))].join("\n");
}

export function getPerformanceCSV() {
  return [PerformanceHeader.join(","), ...PerformanceLog.map(row => row.join(","))].join("\n");
}

export function getCombinedLogCSV() {
  const N = Math.max(QLog.length, ReactionLog.length, PerformanceLog.length);
  const header = [...QHeader, ...ReactionHeader, ...PerformanceHeader];

  const rows = [];
  for (let i = 0; i < N; i++) {
    const qRow = QLog[i] || Array(QHeader.length).fill("");
    const rRow = ReactionLog[i] || Array(ReactionHeader?.length || 0).fill("");
    const pRow = PerformanceLog[i] || [""];
    rows.push([...qRow, ...rRow, ...pRow]);
  }

  return [header.join(","), ...rows.map(row => row.join(","))].join("\n");
}

export function downloadCSV(filename) {
  if (!filename) {
    const now = new Date();
    const pad = n => n.toString().padStart(2, '0');
    const datetime = `${pad(now.getFullYear() % 100)}${pad(now.getMonth() + 1)}${pad(now.getDate())}_${pad(now.getHours())}${pad(now.getMinutes())}${pad(now.getSeconds())}`;
    filename = `RunTimeLog_${datetime}.csv`;
  }

  const csv = getCombinedLogCSV();
  const blob = new Blob([csv], { type: "text/csv" });
  const link = document.createElement("a");
  link.href = URL.createObjectURL(blob);
  link.download = filename;
  document.body.appendChild(link);
  link.click();
  document.body.removeChild(link);
}


export let playbackData = [];

export async function loadCSVFile(file) {
  const text = await file.text();
  const [headerLine, ...lines] = text.trim().split("\n");
  const headers = headerLine.split(",");
  playbackData = lines.map(line => {
    const values = line.split(",").map(Number);
    return Object.fromEntries(values.map((v, i) => [headers[i], v]));
  });
  console.log("CSV Loaded:", playbackData.length, "steps");
}