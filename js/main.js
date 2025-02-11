import { loadSystemConfig, evaluateMatrix, evaluateExpression } from './loadSystemConfig.js';
import { rk4Step, rk4StepX } from './RK4.js';
import { initializeGraphics, updateGraphics } from './testGL.js';

let systemConfig;
let state = {};
let cartesianState = {};

async function initialize() {
    systemConfig = await loadSystemConfig();
    state = systemConfig.state;
    cartesianState = Array.from({ length: systemConfig.system.B.length/6 }, () => Array(6).fill(0)); 
    initializeGraphics();
    console.log("System Configuration Loaded",systemConfig);
}

async function runRK4Step() {
    if (!systemConfig) {
        console.error("System configuration not loaded.");
        return;
    }

    const { evaluateMatrix, evaluatedMatrices, system } = systemConfig;
    const dt = 0.25;

    // Extract required matrices
    const { Qcoordinates: Q, Mstar, Nstar, Bt, F } = evaluatedMatrices;
    const Fstar = math.multiply(Bt, F);

    // RK4 step for generalized coordinates
    const newState = rk4Step(Q, Mstar, Nstar, Fstar, dt);

    // Update state
    system.Qcoordinates.forEach((qRow, i) => {
        state[qRow[0].vars] = newState[i][0];
        state[qRow[1].vars] = newState[i][1];
        state[qRow[2].vars] = (newState[i][1] - Q[i][1]) / dt;
    });

    // Re-evaluate matrices
    const updatedMatrices = Object.fromEntries(
        Object.entries(system)
          .filter(([key]) => key !== "Qcoordinates" && key !== "initconditions")
          .map(([key, value]) => [key, evaluateMatrix(value, state)])
      );

    // RK4 step for Cartesian coordinates
    const B_new = updatedMatrices["B"];
    const qd_new = newState.map(q => [q[1]]);
    const X_new = rk4StepX(cartesianState, B_new, qd_new, dt);

    // Update Cartesian state
    X_new.forEach((row, i) => {
        cartesianState[i] = row;
    });
    
    console.log(`Step completed. Updated Generalized & Cartesian State.`, cartesianState);
}

async function runSimulation() {
    await initialize();
    console.time("Total RK4 Execution");

    let step = 0;
    const maxSteps = 200;
    const dt = 0.25; // Time step in seconds
    let lastTime = performance.now();

    async function runNextStep() {
        let currentTime = performance.now();
        let elapsed = (currentTime - lastTime) / 1000; // Convert to seconds
        
        if (elapsed >= dt) {
            console.time(`RK4 Step ${step + 1}`);
            await runRK4Step();
            console.timeEnd(`RK4 Step ${step + 1}`);

            step++;
            lastTime = currentTime;
        }

        if (step < maxSteps) {
            requestAnimationFrame(runNextStep);
        } else {
            console.timeEnd("Total RK4 Execution");
        }
    }
    updateGraphics(cartesianState);
    requestAnimationFrame(runNextStep);
}

runSimulation();
    