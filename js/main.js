import { loadSystemConfig, evaluateMatrix, evaluateExpression } from './configImport.js';
import { rk4Step, runRK4Step } from './RK4.js';
import { initializeGraphics } from './testGL.js';
import { getNextPos, getRotationMatrices } from "./positionRotation.js";

let system, Q, variableMap, xState;
let running = false;
let step = 0;
const maxSteps = 10;
const dt = 0.1; // Time step in seconds
let lastTime = performance.now();

async function initialize() {
    ({ system, Q, variableMap } = await loadSystemConfig('../src/FrameGen/doublePendulumConfig.json'));
    console.log("System Config Loaded", system)

    xState = evaluateMatrix(system.T, Q, variableMap);
    console.log("Initial X state:", xState);
}   

await initialize()
await runRK4Step(system, Q, variableMap, dt)

let xState_new = await getNextPos(xState, system,Q,variableMap,dt)
console.log("xState new", xState_new)

let rotations = await getRotationMatrices(system, Q, variableMap)
console.log("Rotations", rotations)





    /*
    qState = await systemConfig.Q;
    xState = evaluateMatrix(systemConfig.system.T, qState)
    console.log("X state:", xState)
    console.log("System Configuration Loaded", systemConfig);
    console.log(evaluateMatrix(systemConfig.system.Qcoordinates, qState))
}



function runNextStep() {
    if (!running) return;
    let currentTime = performance.now();
    let elapsed = (currentTime - lastTime) / 1000; // Convert to seconds
    
    if (elapsed >= dt) {
        console.time(`RK4 Step ${step + 1}`);
        runRK4Step(systemConfig, qState, dt).then(() => {
            console.timeEnd(`RK4 Step ${step + 1}`);
        });
        step++;
        lastTime = currentTime;
    }
    
    if (step < maxSteps) {
        requestAnimationFrame(runNextStep);
    } else {
        console.timeEnd("Total RK4 Execution");
        running = false;
    }
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

function resetSimulation() {
    running = false;
    step = 0;
    initialize();
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
    runRK4Step(systemConfig, qState, dt);
    createUI();
});

*/