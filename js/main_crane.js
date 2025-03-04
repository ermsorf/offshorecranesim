import { loadSystemConfig, evaluateMatrix, evaluateExpression } from './configImport.js';
import { runRK4Step } from './RK4.js';
import { getNextPos, getRotationMatrices } from "./positionRotation.js";
import { initgraphics } from './webGL_Crane.js';

let system, Q, variableMap, xState, xState_new, rotations;
let running = false;
let step = 0;
const maxSteps = 2000;
const dt = 0.05; // Time step in seconds
let lastTime = performance.now();

async function initialize() {
    ({ system, Q, variableMap } = await loadSystemConfig('../src/FrameGen/CraneConfig.json'));
    console.log("System Config Loaded", system);
    xState = await evaluateMatrix(system.T, Q, variableMap);
    console.log("Initial X state:", xState);
}   

async function runNextStep() {
    if (!running) return;

    let currentTime = performance.now();
    let elapsed = (currentTime - lastTime) / 1000; // Convert to seconds

    while (elapsed >= dt) {  // Run multiple steps if needed
        console.time(`Step ${step + 1}`);

        await runRK4Step(system, Q, variableMap, dt);
        console.timeEnd(`Step ${step + 1}`);

        const positions = await getNextPos(xState, system, Q, variableMap, dt);

        const rotations = await getRotationMatrices(system, Q, variableMap);

        step++;
        lastTime += dt * 1000; // Move forward in fixed steps

        if (step >= maxSteps) {
            running = false;
            return;
        }

        elapsed = (currentTime - lastTime) / 1000; // Recalculate elapsed time
    }

    requestAnimationFrame(runNextStep);
}


function startSimulation() {    
    if (!running) {
        running = true;
        step = 0;
        lastTime = performance.now();
        requestAnimationFrame(runNextStep);
    }
}

function stopSimulation() {
    running = false;
}

async function resetSimulation() {
    running = false;
    step = 0;
    await initialize();
}

// Create UI buttons
function createUI() {
    const container = document.createElement('div');
    container.style.position = 'absolute';
    container.style.bottom = '10px';
    container.style.left = '10px';
    container.style.zIndex = '100';

    const startButton = document.createElement('button');
    startButton.innerText = 'Start';
    startButton.onclick = startSimulation;
    
    const stopButton = document.createElement('button');
    stopButton.innerText = 'Stop';
    stopButton.onclick = stopSimulation;
    
    const resetButton = document.createElement('button');
    resetButton.innerText = 'Reset';
    resetButton.onclick = resetSimulation;
    
    container.appendChild(startButton);
    container.appendChild(stopButton);
    container.appendChild(resetButton);
    document.body.appendChild(container);
}

initialize().then(() => {
    runRK4Step(system, Q, variableMap, dt);
    createUI();
});

