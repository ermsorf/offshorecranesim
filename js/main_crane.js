import { loadSystemConfig, evaluateMatrix, evaluateExpression } from './configimport.js';
import { runRK4Step } from './RK4.js';
// import { getNextPos, getRotationMatrices } from "./positionRotation.js";
import { initializeScene, updateTransformListValues } from './webGL_crane.js';
import * as THREE from '../libs/three/three.module.js';
import GUI from '../libs/js/lil-gui.module.min.js';

const graphicsDiv = document.getElementById("graphics");
let system, Q, variableMap, xState, xState_new, rotations;
let running = false;
let step = 0;
const maxSteps = 20000;
const dt = 0.02; // Time step in seconds
let lastTime = performance.now();


let renderer, scene, camera, controls, water;
let transformValues = {};
let controlForces = {
    BoomRotationZ: 0,
    TrolleyTranslationX: 0,
    Theta1RotationZ: 0,
    Phi1RotationX: 0,
    Lambda1TranslationZ: 0,
    Theta2RotationZ: 0,
    Phi2RotationX: 0,
    Lambda2TranslationZ: 0,
    Theta3RotationZ: 0,
    Phi3RotationX: 0,
    Lambda3TranslationZ: 0
};

async function initialize() {
    // await console.log("Q before loadSystemConfig", Q);
    ({ system, Q, variableMap } = await loadSystemConfig('../src/FrameGen/CraneDemo.json'));
    await console.log("System Config Loaded", system);
    //await console.log("Q after loadSystemConfig", Q);

    ({ renderer, scene, camera, controls, water } = initializeScene(graphicsDiv));
}   

function animate() {
    requestAnimationFrame(animate);
    const time = performance.now();
    //updateTransformsWithTrig(time);
    updateTransformListValues(transformValues);
    controls.update();
    water.material.uniforms['time'].value += 1.0 / 60.0;
    renderer.render(scene, camera);
}

initialize().then(() => {
    // console.log("Q in initialize().then()", Q);
    runRK4Step(system, Q, variableMap, dt, controlForces);
    createUI();
    animate();
});



function runNextStep() {
    if (!running) return;

    let currentTime = performance.now();
    let elapsed = (currentTime - lastTime) / 1000; // Convert to seconds
    
    while (elapsed >= dt) {  // Run multiple steps if needed
        // console.log("Q before runRK4Step", Q);
        runRK4Step(system, Q, variableMap, dt, controlForces);
        // console.log("Q after runRK4Step", Q)
        // console.log("getVar('cr')", getVar('cr'));
        // console.log("getVar('trolley')", getVar('trolley'));
        transformValues = { "BoomRotationZ": getVar("cr"),
                            "TrolleyTranslationX": getVar("trolley"),
                            "Theta1RotationZ": getVar("theta1"),
                            "Phi1RotationX": getVar("phi1"),
                            "Lambda1TranslationZ": getVar("lambda1"),
                            "Theta2RotationZ": getVar("theta2"),
                            "Phi2RotationX": getVar("phi2"),
                            "Lambda2TranslationZ": getVar("lambda2"),
                            "Theta3RotationZ": getVar("theta3"),
                            "Phi3RotationX": getVar("phi3"),
                            "Lambda3TranslationZ": getVar("lambda3")
                        };


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

async function singleStep() {
    if (!running) {
        // console.log("Executing Single Step...");
        let currentTime = performance.now();
        let elapsed = (currentTime - lastTime) / 1000; // Convert to seconds

        if (elapsed >= dt) {
            await runRK4Step(system, Q, variableMap, dt);
            //console.log("Single Step Q:", Q);
            //console.log("getVar('cr')", getVar('cr'));
            //console.log("getVar('trolley')", getVar('trolley'));

            step++;
            lastTime += dt * 1000; // Move forward in fixed steps
        }
    }
}


// Create UI buttons


function createUI() {
    const gui = new GUI();
    let controllers = {}; // Store GUI controllers for updates

    // Simulation controls
    const simulationFolder = gui.addFolder('Simulation Controls');
    simulationFolder.add({ start: startSimulation }, 'start');
    simulationFolder.add({ stop: stopSimulation }, 'stop');
    simulationFolder.add({ reset: resetSimulation }, 'reset');
    simulationFolder.add({ step: singleStep }, 'step');
    simulationFolder.open();

    // Transform controls folder
    const transformFolder = gui.addFolder('Transform Controls');

    function addControl(folder, prop, min, max) {
        controllers[prop] = folder.add(controlForces, prop, min, max)
            .onChange((value) => {
                controlForces[prop] = value; // Update value live
            });
    }

    addControl(transformFolder, 'BoomRotationZ', -1000000, 1000000);
    addControl(transformFolder, 'TrolleyTranslationX', -50000, 50000);

    // Add a reset button
    transformFolder.add({ reset: () => {
        Object.keys(controlForces).forEach((key) => {
            controlForces[key] = 0;
            controllers[key].setValue(0); // Reset GUI sliders visually
        });
    }}, 'reset').name('Reset Inputs');

    transformFolder.open();
}

function getVar(name) {
    const { i, j } = variableMap[name];
    return Q[i][j];
}