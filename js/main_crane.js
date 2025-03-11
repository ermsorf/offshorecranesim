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
    TrolleyTranslationX: 0
};

async function initialize() {
    // await console.log("Q before loadSystemConfig", Q);
    ({ system, Q, variableMap } = await loadSystemConfig('../src/FrameGen/CraneDemo3Wires.json'));
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
    // scene.traverse((object) => {
    //     console.log(object.name, object);
    // });
    animate();
});



function runNextStep() {
    if (!running) return;

    let currentTime = performance.now();
    let elapsed = (currentTime - lastTime) / 1000; // Convert to seconds
    
    while (elapsed >= dt) {  // Run multiple steps if needed
        // console.log("Q before runRK4Step", Q);
        // console.time("runRK4Step");
        runRK4Step(system, Q, variableMap, dt, controlForces);
        // console.timeEnd("runRK4Step");
        // console.log("Q after runRK4Step", Q)
        // console.log("getVar('cr')", getVar('cr'));
        // console.log("getVar('trolley')", getVar('trolley'));

        transformValues = { "BoomRotationZ": getVar("cr"),
                            "TrolleyTranslationX": getVar("trolley"),
                            "Theta1RotationZ": getVar("theta1"),
                            "Phi1RotationX": getVar("phi1"),
                            //"Lambda1TranslationZ": getVar("lambda1"),
                            "Theta2RotationZ": getVar("theta2"),
                            "Phi2RotationX": getVar("phi2"),
                            //"Lambda2TranslationZ": getVar("lambda2"),
                            "Theta3RotationZ": getVar("theta3"),
                            "Phi3RotationX": getVar("phi3")
                            //"Lambda3TranslationZ": getVar("lambda3")
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

    // Simulation controls
    const simulationFolder = gui.addFolder('RK4 Controls');
    simulationFolder.add({ Start: startSimulation }, 'Start');
    simulationFolder.add({ Stop: stopSimulation }, 'Stop');
    simulationFolder.add({ Step: singleStep }, 'Step');
    simulationFolder.open();

    // Camera controls
    const cameraFolder = gui.addFolder('Camera');
    const cameraActions = {
        Global: () => console.warn("Global camera switch not implemented"),
        Cabin: () => console.warn("Cabin camera switch not implemented"),
        Trolley: () => console.warn("Trolley camera switch not implemented"),
    };
    cameraFolder.add(cameraActions, 'Global');
    cameraFolder.add(cameraActions, 'Cabin');
    cameraFolder.add(cameraActions, 'Trolley');

    // Joystick controls folder
    const joystickFolder = gui.addFolder('Joystick Controls');
    const joystickContainer = document.createElement('div');
    joystickContainer.style.position = 'relative';
    joystickContainer.style.width = '150px';
    joystickContainer.style.height = '150px';
    joystickContainer.style.margin = '10px auto';
    joystickContainer.style.background = '#424242';
    joystickContainer.style.borderRadius = '50%';
    joystickFolder.domElement.appendChild(joystickContainer);

    const joystick = nipplejs.create({
        zone: joystickContainer,
        mode: 'static',
        position: { left: '50%', top: '50%' },
        color: '#27D0FF'
    });

    joystick.on('move', (evt, data) => {
        if (data && data.force !== undefined && data.angle) {
            const angle = data.angle.radian;
            const force = data.force;
            controlForces.BoomRotationZ = - Math.cos(angle) * force * 100000;
            controlForces.TrolleyTranslationX = Math.sin(angle) * force * 15000;
        }
    });

    joystick.on('end', () => {
        controlForces.BoomRotationZ = 0;
        controlForces.TrolleyTranslationX = 0;
    }); 
}


function getVar(name) {
    const { i, j } = variableMap[name];
    return Q[i][j];
}

function setVar(name, value) {
    const { i, j } = variableMap[name];
    Q[i][j] = value;
}


function updateSpline() {
    const p1 = new THREE.Vector3(), p2 = new THREE.Vector3(), p3 = new THREE.Vector3();
    body1.getWorldPosition(p1);
    body2.getWorldPosition(p2);
    body3.getWorldPosition(p3);

    curve.points = [p1, p2, p3];

    tubeMesh.geometry.dispose();  // Free memory
    tubeMesh.geometry = new THREE.TubeGeometry(curve, 20, 0.1, 8, false);
}