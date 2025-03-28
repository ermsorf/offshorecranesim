import {
    loadSystemConfig,
    evaluateMatrix,
    evaluateExpression,
  } from "./configimport.js";
import { runRK4Step } from "./RK4.js";
import { initializeScene, updateTransformListValues } from "./webGL_crane.js";
import * as THREE from "../libs/three/three.module.js";
import GUI from "../libs/js/lil-gui.module.min.js"; 

const graphicsDiv = document.getElementById("graphics");
let system, Q, variableMap, trigMap;
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
  LambdaExtensionZ: 0
};

async function initialize() {
  ({ system, Q, variableMap, trigMap } = await loadSystemConfig(
    "../src/FrameGen/Crane3LinkExtending.json",    // Path to the system configuration file
  ));

  ({ renderer, scene, camera, controls, water } = initializeScene(graphicsDiv)); // Initialize graphics scene
}

initialize().then(() => {
  // console.log("Q in initialize().then()", Q);
  runRK4Step(system, Q, variableMap, trigMap, dt, controlForces);
  createUI();
  // scene.traverse((object) => {
  //     console.log(object.name, object);
  // });
  animate();
});

function animate() {
  requestAnimationFrame(animate);
  const time = performance.now();
  updateTransformListValues(transformValues);
  controls.update();
  water.material.uniforms["time"].value += 1.0 / 60.0;
  renderer.render(scene, camera);
}


function runNextStep() {
  if (!running) return;

  let currentTime = performance.now(); 
  let elapsed = (currentTime - lastTime) / 1000; 

  while (elapsed >= dt) { // Ensure catch up if lagging

    console.time("runRK4Step");
    runRK4Step(system, Q, variableMap, trigMap, dt, controlForces);
    console.timeEnd("runRK4Step");

    transformValues = {
      BoomRotationZ: getVar("cr"),
      TrolleyTranslationX: getVar("trolley"),
      Theta1RotationZ: getVar("theta1"),
      Phi1RotationX: getVar("phi1"),
      Lambda1TranslationZ: getVar("lambda1"),
      Theta2RotationZ: getVar("theta2"),
      Phi2RotationX: getVar("phi2"),
      Lambda2TranslationZ: getVar("lambda2"),
      Theta3RotationZ: getVar("theta3"),
      Phi3RotationX: getVar("phi3"),
      Lambda3TranslationZ: getVar("lambda3"),
      containerRotationZ: getVar("theta4"),
      // Phi4RotationX: getVar("phi4"),
      // Lambda4TranslationZ: getVar("lambda4"),
      // Theta5RotationZ: getVar("theta5"),
      // Phi5RotationX: getVar("phi5"),
      // Lambda5TranslationZ: getVar("lambda5"),
      // Theta6RotationZ: getVar("theta6"),
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

} // END runNextStep



// Create UI buttons

function createUI() {
  const gui = new GUI();
  gui.domElement.style.position = "absolute";
  gui.domElement.style.top = "0px";
  gui.domElement.style.right = "0px";
  gui.domElement.style.height = '100vh'; // Full vertical height
  gui.domElement.style.overflowY = 'auto'; // Enable scrolling if needed

  // Simulation controls
  const simulationFolder = gui.addFolder("RK4 Controls");
  simulationFolder.add({ Start: startSimulation }, "Start");
  simulationFolder.add({ Stop: stopSimulation }, "Stop");
  simulationFolder.add({ Step: singleStep }, "Step");
  simulationFolder.open();

  // Camera controls
  const cameraFolder = gui.addFolder("Camera");
  const cameraActions = {
    Global: () => console.warn("Global camera switch not implemented"),
    Cabin: () => console.warn("Cabin camera switch not implemented"),
    Trolley: () => console.warn("Trolley camera switch not implemented"),
  };
  cameraFolder.add(cameraActions, "Global");
  cameraFolder.add(cameraActions, "Cabin");
  cameraFolder.add(cameraActions, "Trolley");

  // Joystick controls folder
  const joystickFolder = gui.addFolder("Joystick Controls");
  const joystickContainer = document.createElement("div");
  joystickContainer.style.position = "relative";
  joystickContainer.style.width = "150px";
  joystickContainer.style.height = "150px";
  joystickContainer.style.margin = "10px auto";
  joystickContainer.style.background = "#424242";
  joystickContainer.style.borderRadius = "50%";
  joystickFolder.domElement.appendChild(joystickContainer);

  const joystick = nipplejs.create({
    zone: joystickContainer,
    mode: "static",
    position: { left: "50%", top: "50%" },
    color: "#27D0FF",
  }); 

  joystick.on("move", (evt, data) => {
    if (data && data.force !== undefined && data.angle) {
      const angle = data.angle.radian;
      const force = data.force;
      controlForces.BoomRotationZ = -Math.cos(angle) * force * 10000;
      controlForces.TrolleyTranslationX = Math.sin(angle) * force * 1000;
    }
  });

  joystick.on("end", () => {
    controlForces.BoomRotationZ = 0;
    controlForces.TrolleyTranslationX = 0;
  });
}


// Functions for buttons to start and stop simulation
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

async function singleStep() {
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
  const { i, j } = variableMap[name];
  return Q[i][j];
}

export function setVar(name, value) {
  const { i, j } = variableMap[name];
  Q[i][j] = value;
}