import * as THREE from '../libs/three/three.module.js';
import { OrbitControls } from '../libs/three/controls/OrbitControls.js';
import { loadModel, sceneObjects } from '../libs/three/loaders/modelLoader.js';
import GUI from '../libs/js/lil-gui.module.min.js'; 


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
loadModel('Tower', '../assets/models/tower.obj', '../assets/materials/tower.mtl', { x: 0, y: 0, z: 0 }, scene, gui, null, {
    position: { x: [0, 0], y: [0, 0], z: [0, 0] },
    rotation: { x: [0, 0], y:[0, 0] , z: [0,0] }
});

// Load Part 2 (Child of Part1)
loadModel('Boom', '../assets/models/boom.obj', '../assets/materials/boom.mtl', { x: 0, y: 0, z: 0 }, scene, gui, 'Tower', {
    position: { x: [0, 0], y: [0, 0], z: [0, 0] },
    rotation: { x: [0, 0], y: [0, 0], z: [-Math.PI / 2, Math.PI / 2] }
});

// Load Part3 (Child of Part2)
loadModel('Trolley', '../assets/models/trolley.obj', '../assets/materials/trolley.mtl', { x: 0, y: 0, z: 0 }, scene, gui, 'Boom', {
    position: { x: [0, 20], y: [0, 0], z: [0, 0] },
    rotation: { x: [0, 0], y: [0, 0], z: [0, 0] }
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
// Animation Loop
// -------------------------------
function animate() {
    requestAnimationFrame(animate);
    
    controls.update();


    renderer.render(scene, camera);
}

animate();

// Adjust canvas size on window resize
window.addEventListener("resize", () => {
    renderer.setSize(graphicsDiv.clientWidth, graphicsDiv.clientHeight);
    camera.aspect = graphicsDiv.clientWidth / graphicsDiv.clientHeight;
    camera.updateProjectionMatrix();
});
