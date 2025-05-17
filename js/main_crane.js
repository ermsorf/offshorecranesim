import * as THREE from "../libs/three/three.module.js";
import {loadSystemConfig, evaluateMatrix, evaluateExpression,} from "./configimport.js";
import { runRK4Step } from "./RK4.js";
import { initializeScene, updateTransformListValues, createOrUpdateWire } from "./webGL_crane.js";
import { createUI } from "./createUI.js";
import { sceneObjects } from "../libs/three/loaders/modelLoader.js";
import { logPerformance } from "./csv.js";
// Set parameters
const configFilePath = "../src/FrameGen/Crane1EL.json"; // Path to the system configuration file generated in MATLAB
export const dt = 0.010 ; // Time step in seconds
export const maxTime = 50, maxSteps = 50000; // Maximum simulation time and steps 


// Define variables global scope
const graphicsDiv = document.getElementById("graphics");
export let system, Q, variableMap, trigMap;
export let running = false, step = 0, lastTime = 0;
let renderer, scene, camera, controls, water;
let transformValues = {};
export let controlForces = {
  BoomRotationZ: 0,
  TrolleyTranslationX: 0,
  LambdaTranslationZ: 0
};



// ###################################################################################

async function initialize() { // Loads system and initializes scene
  ({ system, Q, variableMap, trigMap } = await loadSystemConfig(configFilePath)); // Load system configuration
  ({ renderer, scene, camera, controls, water } = initializeScene(graphicsDiv));  // Initialize webGL scene from webGL_crane.js, 
}

initialize().then(() => { // Starts website
  runRK4Step(system, Q, variableMap, trigMap, dt, controlForces); // Run one step to initialize Q
  createUI(); // Create UI elements
  animate(); // Start webGL animation loop
  console.log("sceneObjects", sceneObjects);

});

function animate() {
  requestAnimationFrame(animate);
  const time = performance.now();
  updateTransformListValues(transformValues); // Update coordinates to bodies loaded in webGL_scene, transformValues updated in runNextStep()
  controls.update(); // Update camera controls
  water.material.uniforms["time"].value += 1.0 / 60.0; 
  renderer.render(scene, camera);
}


async function runNextStep() {
  if (!running) return;

  let currentTime = performance.now();
  let elapsed = (currentTime - lastTime) / 1000;

  while (elapsed >= dt) {
    if (!running) break; // Check running inside loop!

    let start = performance.now();
    await runRK4Step(system, Q, variableMap, trigMap, dt, controlForces);
    let end = performance.now();

    logPerformance(end - start);
    console.log("Step:", step, "Time:", end-start, "ms");

    step++;
    lastTime += dt * 1000;
    if (step >= maxSteps || step * dt >= maxTime) {
      running = false;
      console.log("Simulation stopped after", step, "steps");
      break;
    }

    currentTime = performance.now();
    elapsed = (currentTime - lastTime) / 1000;

      transformValues = {
      BoomRotationZ: getVar("cr"),
      TrolleyTranslationX: getVar("trolley"),
      ContainerRotationZ: getVar("container"),
  };
  if (system.info.wiresegments >= 1) {
      Object.assign(transformValues, {
          Theta1RotationZ: getVar("theta1"),
          Phi1RotationX: getVar("phi1"),
          Lambda1TranslationZ: getVar("lambda1"),
      });
  }
  if (system.info.wiresegments >= 2) {
      Object.assign(transformValues, {
          Theta2RotationZ: getVar("theta2"),
          Phi2RotationX: getVar("phi2"),
          Lambda2TranslationZ: getVar("lambda2"),
      });
  }
  if (system.info.wiresegments >= 3) {
      Object.assign(transformValues, {
          Theta3RotationZ: getVar("theta3"),
          Phi3RotationX: getVar("phi3"),
          Lambda3TranslationZ: getVar("lambda3"),
      });
  }
  // Update wires 
  const bodyNames = Array.from({length: system.info.wiresegments }, (_, i) => `Theta${i + 1}`).concat('Container');
  const points = bodyNames.map(name => sceneObjects[name].getWorldPosition(new THREE.Vector3()));
  createOrUpdateWire(points, scene);
    // Yield control to UI for one frame to keep it responsive
    await new Promise(requestAnimationFrame);
  }


  if (running) {
    requestAnimationFrame(runNextStep);
  }
}

// Functions for buttons to start and stop simulation
export function startSimulation() {
  if (!running) {
    running = true;
    step = 0;
    lastTime = performance.now();
    requestAnimationFrame(runNextStep);
  }
}

export function stopSimulation() {
  running = false;
}

export async function singleStep() {
  if (!running) {
    let currentTime = performance.now();
    let elapsed = (currentTime - lastTime) / 1000; 

    if (elapsed >= dt) {
      await runRK4Step(system, Q, variableMap, trigMap, dt, controlForces);

      step++;
      lastTime += dt * 1000; 
    }
  }
}

export function getVar(name) {
  if (!variableMap[name]) {
    console.warn(`Variable "${name}" not found in variableMap.`);
  }
  const { i, j } = variableMap[name];
  return Q[i][j];
}

export function setVar(name, value) {
  const { i, j } = variableMap[name];
  Q[i][j] = value;
}

