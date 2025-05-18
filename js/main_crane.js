import * as THREE from "../libs/three/three.module.js";
import {loadSystemConfig, evaluateMatrix, evaluateExpression,} from "./configimport.js";
import { runRK4Step } from "./RK4.js";
import { initializeScene, updateTransformListValues, createOrUpdateWire } from "./webGL_crane.js";
import { createUI, playbackMode } from "./createUI.js";
import { sceneObjects } from "../libs/three/loaders/modelLoader.js";
import { logPerformance, playbackData } from "./csv.js";

// Set parameters
const configFilePath = "../src/FrameGen/Crane1EL.json"; // Path to the system configuration file generated in MATLAB
export const dt = 0.010 ; // Time step in seconds
export const maxTime = 50, maxSteps = 50000; // Maximum simulation time and steps 

 
// Define variables global scope
const graphicsDiv = document.getElementById("graphics");
export let system, Q, variableMap, trigMap;
export let running= false, step = 0, lastTime = 0;
let renderer, scene, camera, controls, water;
export let playbackIndex = 0;
let transformValues = {};
export let controlForces = {
  BoomRotationZ: 0,
  TrolleyTranslationX: 0,
  LambdaTranslationZ: 0
};
let allowedKeys;


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
  allowedKeys = new Set(system.Qcoordinates.flat().map(entry => entry.vars)
);

});

function animate() {
  requestAnimationFrame(animate);
  const time = performance.now();
  updateTransformListValues(transformValues); // Update coordinates to bodies loaded in webGL_scene, transformValues updated in runNextStep()
  controls.update(); // Update camera controls
  water.material.uniforms["time"].value += 1.0 / 60.0; 
  renderer.render(scene, camera);
}




let playbackStartTime = null;

async function runNextStep() {
  if (!running) return;

  const now = performance.now();

  if (playbackMode) {
    if (playbackStartTime === null) playbackStartTime = now;

    const elapsedPlaybackTime = (now - playbackStartTime) / 1000;

    while (
      playbackIndex < playbackData.length &&
      playbackData[playbackIndex].time <= elapsedPlaybackTime
    ) {
      const data = playbackData[playbackIndex++];
      for (const key in data) {
        if (!allowedKeys.has(key)) continue;
        setVar(key, data[key]);
      }
    }

    if (playbackIndex >= playbackData.length) {
      running = false;
      console.log("Playback finished");
      return;
    }
  } else {
    let elapsed = (now - lastTime) / 1000;
    while (elapsed >= dt) {
      if (!running) break;
      const start = performance.now();
      await runRK4Step(system, Q, variableMap, trigMap, dt, controlForces);
      const end = performance.now();
      logPerformance(end - start); // Log performance time
      console.log("Step:", step, "Time:", (step*dt).toFixed(3), "Elapsed:", end-start, "ms");
      step++;
      lastTime += dt * 1000;
      elapsed -= dt;
      if (end - start > 400) {
        console.warn("Step took too long:", end - start, "ms");
        stopSimulation();
        break
      }
    }
  }

  // Update transforms
  transformValues = { BoomRotationZ: getVar("cr"), TrolleyTranslationX: getVar("trolley"), ContainerRotationZ: getVar("container") };
  if (system.info.wiresegments >= 1) Object.assign(transformValues, {
    Theta1RotationZ: getVar("theta1"), Phi1RotationX: getVar("phi1"), Lambda1TranslationZ: getVar("lambda1")
  });
  if (system.info.wiresegments >= 2) Object.assign(transformValues, {
    Theta2RotationZ: getVar("theta2"), Phi2RotationX: getVar("phi2"), Lambda2TranslationZ: getVar("lambda2")
  });
  if (system.info.wiresegments >= 3) Object.assign(transformValues, {
    Theta3RotationZ: getVar("theta3"), Phi3RotationX: getVar("phi3"), Lambda3TranslationZ: getVar("lambda3")
  });

  const bodyNames = Array.from({ length: system.info.wiresegments }, (_, i) => `Theta${i + 1}`).concat('Container');
  const points = bodyNames.map(name => sceneObjects[name].getWorldPosition(new THREE.Vector3()));
  createOrUpdateWire(points, scene);

  if (running) requestAnimationFrame(runNextStep);
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

export function setRunning(value){
  running = value;
}
export function setStep(value){
  step = value;
}
export function setPlaybackIndex(value){
  playbackIndex = value;
}