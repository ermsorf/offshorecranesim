import { loadSystemConfig, evaluateMatrix } from './configImport.js';
import { runRK4Step } from './RK4.js';

// Simulation parameters
const dt = 0.01; // Match MATLAB dt
const steps = 500;
let system, Q, variableMap;
let results = [];

// Function to initialize and run simulation
async function runSimulation() {
    ({ system, Q, variableMap } = await loadSystemConfig('../src/FrameGen/doublePendulumConfig.json'));

    // Store initial conditions
    results.push([0, Q[0][0], Q[1][0], Q[0][1], Q[1][1]]); // [time, q1, q2, qd1, qd2]

    for (let i = 1; i <= steps; i++) {
        runRK4Step(system, Q, variableMap, dt);
        results.push([i * dt, Q[0][0], Q[1][0], Q[0][1], Q[1][1]]);
    }

    // Save to CSV
    saveResultsToCSV(results);
}

// Function to save results to a CSV file
function saveResultsToCSV(data) {
    let csvContent = "time,q1,q2,qd1,qd2\n" + data.map(e => e.join(",")).join("\n");
    let blob = new Blob([csvContent], { type: "text/csv" });
    let a = document.createElement("a");
    a.href = URL.createObjectURL(blob);
    a.download = "rk4_results.csv";
    a.click();
}

// Run simulation
runSimulation();