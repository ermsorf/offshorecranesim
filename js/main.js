
// Load the system config JSON file
fetch('../src/matlabprototyping/config.json')
  .then(response => response.json())
  .then(system => {
      const h = 0.01; // Time step
      let state = { x: 1.5, y: 2.0, z: Math.PI / 4 }; // Initial conditions

      function evaluateExpression(expr, vars) {
          let result = expr;
          vars.forEach(varName => {
              result = result.replace(new RegExp(`\\b${varName}\\b`, 'g'), state[varName]);
          });
          return eval(result);
      }

      function evaluateMatrix(matrix) {
          return matrix.map(row =>
              row.map(entry => evaluateExpression(entry.expr, entry.vars))
          );
      }

      // Process all matrices dynamically
      let evaluatedMatrices = {};
      Object.keys(system).forEach(key => {
          if (key !== "variables") {
              evaluatedMatrices[key] = evaluateMatrix(system[key]);
          }
      });

      console.log(evaluatedMatrices); // Matrices with numerical values
  });

import * as THREE from '../libs/three/three.module.js';
import { OrbitControls } from '../libs/three/controls/OrbitControls.js';
import GUI from '../libs/three/lil-gui.module.min.js'; 
import { loadModel, sceneObjects } from './modelLoader.js';

// Setting up the scene
const scene = new THREE.Scene();
scene.background = new THREE.Color('white');

// GUI
const gui = new GUI();

// Get the div where we will attach the renderer
const graphicsDiv = document.getElementById("graphics");

// Set up the camera
const camera = new THREE.PerspectiveCamera(
    75, 
    graphicsDiv.clientWidth / graphicsDiv.clientHeight, 
    0.1, 
    1000000
);
camera.position.set(0, 10, 10);
camera.lookAt(0, 0, 0);

// Set up the renderer
const renderer = new THREE.WebGLRenderer();
renderer.setSize(graphicsDiv.clientWidth, graphicsDiv.clientHeight);
graphicsDiv.appendChild(renderer.domElement);

// Camera controls
const controls = new OrbitControls(camera, renderer.domElement);
controls.update();

// Lighting
const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
directionalLight.position.set(50, 50, 50).normalize();
scene.add(directionalLight);

// -------------------------------
// Global variable for dynamic line connection
// This array will hold the names of the bodies we want to connect.
let dynamicLineBodyNames = [];

// -------------------------------
// Function to draw (and update) the line between bodies
// This function creates the line and sets the global dynamicLineBodyNames.
function drawLineBetweenBodies(bodyNames, lineColor = 0xff0000) {
    dynamicLineBodyNames = bodyNames; // Save for dynamic updates
    const points = bodyNames.map(name => {
        const obj = sceneObjects[name];
        if (!obj) {
            console.warn(`Object "${name}" not found.`);
            return new THREE.Vector3();
        }
        return obj.position.clone();
    });
    const geometry = new THREE.BufferGeometry().setFromPoints(points);
    const material = new THREE.LineBasicMaterial({ color: lineColor });
    
    // Remove previous line if it exists
    if (window.myLine) {
        scene.remove(window.myLine);
    }
    
    window.myLine = new THREE.Line(geometry, material);
    scene.add(window.myLine);
}

// Function to update the line geometry in real time
function updateLinePositions() {
    if (window.myLine && dynamicLineBodyNames.length > 0) {
        const newPoints = dynamicLineBodyNames.map(name => {
            const obj = sceneObjects[name];
            return obj ? obj.position.clone() : new THREE.Vector3();
        });
        window.myLine.geometry.setFromPoints(newPoints);
    }
}

// -------------------------------
// Load Models
// -------------------------------

// Load Base Object (Part1) - No Parent
loadModel('Part1', '../models/crane_assembly.obj', '../models/crane_assembly.mtl', { x: 0, y: 0, z: 0 }, scene, gui, null, {
    position: { x: [0, 0], y: [0, 0], z: [0, 0] },
    rotation: { x: [0, 0], y: [-Math.PI / 2, Math.PI / 2], z: [0, 0] }
});

// Load Part 1 (Child of Part1)
loadModel('Part2', '../models/crane_assembly.obj', '../models/crane_assembly.mtl', { x: 1, y: 0, z: 0 }, scene, gui, 'Part1', {
    position: { x: [-2, 2], y: [-1, 1], z: [-2, 2] },
    rotation: { x: [0, 0], y: [0, 0], z: [0, 0] }
});

// Load Part3 (Child of Part2)
loadModel('Part3', '../models/frame.obj', '../models/frame.mtl', { x: 0, y: 0, z: 10 }, scene, gui, 'Part2', {
    position: { x: [-20, -10], y: [-10, 10], z: [-20, 20] },
    rotation: { x: [-Math.PI / 2, Math.PI / 2], y: [-Math.PI / 2, Math.PI / 2], z: [-Math.PI / 2, Math.PI / 2] }
});

// Load Part4 (Child of Part3)
loadModel('Part4', '../models/frame.obj', '../models/frame.mtl', { x: 0, y: -5, z: 0 }, scene, gui, 'Part3', {
    position: { x: [-20, -10], y: [-10, 10], z: [-20, 20] },
    rotation: { x: [-Math.PI / 2, Math.PI / 2], y: [-Math.PI / 2, Math.PI / 2], z: [-Math.PI / 2, Math.PI / 2] }
});

// Load Part5 (Child of Part4)
loadModel('Part5', '../models/frame.obj', '../models/frame.mtl', { x: 0, y: -5, z: 0 }, scene, gui, 'Part4', {
    position: { x: [-20, -10], y: [-10, 10], z: [-20, 20] },
    rotation: { x: [-Math.PI / 2, Math.PI / 2], y: [-Math.PI / 2, Math.PI / 2], z: [-Math.PI / 2, Math.PI / 2] }
});

// Load Part6 (Child of Part5)
loadModel('Part6', '../models/frame.obj', '../models/frame.mtl', { x: 0, y: -5, z: 0 }, scene, gui, 'Part5', {
    position: { x: [-20, -10], y: [-10, 10], z: [-20, 20] },
    rotation: { x: [-Math.PI / 2, Math.PI / 2], y: [-Math.PI / 2, Math.PI / 2], z: [-Math.PI / 2, Math.PI / 2] }
});

// -------------------------------
// Sort GUI Folders by Name
// -------------------------------
function sortGuiFoldersByName(gui) {
    // Get the GUI's DOM element (this is where lil-gui places its folders)
    const guiContainer = gui.domElement;
    
    // Query all folder elements; the class name may vary depending on your lil-gui version.
    const folders = Array.from(guiContainer.querySelectorAll('.lil-gui-folder'));
    
    // Sort the folders by the folder title text.
    folders.sort((a, b) => {
        const titleA = a.querySelector('.title')?.textContent.trim() || '';
        const titleB = b.querySelector('.title')?.textContent.trim() || '';
        return titleA.localeCompare(titleB, undefined, { numeric: true });
    });
    
    // Remove the folders from the container and re-append them in sorted order.
    folders.forEach(folder => {
        guiContainer.appendChild(folder);
    });
}

setTimeout(() => {
    sortGuiFoldersByName(gui);
}, 2);

setTimeout(() => {
    sortGuiFoldersByName(gui);
}, 2000); // adjust delay as needed to ensure all folders are added

// -------------------------------
// Draw the dynamic line
// -------------------------------
// Draw a red line connecting Part3 -> Part4 -> Part5.
// We call it once after a delay to ensure the models are loaded.
setTimeout(() => {
    drawLineBetweenBodies(['Part3', 'Part4', 'Part5']);
}, 3000);

// -------------------------------
// Animation Loop
// -------------------------------
function animate() {
    requestAnimationFrame(animate);
    
    controls.update();

    // Update dynamic line positions in real time
    updateLinePositions();
    
    renderer.render(scene, camera);
}

animate();

// Adjust canvas size on window resize
window.addEventListener("resize", () => {
    renderer.setSize(graphicsDiv.clientWidth, graphicsDiv.clientHeight);
    camera.aspect = graphicsDiv.clientWidth / graphicsDiv.clientHeight;
    camera.updateProjectionMatrix();
});
