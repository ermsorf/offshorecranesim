import * as THREE from '../libs/three/three.module.js';
import { OBJLoader } from '../libs/three/loaders/OBJLoader.js';
import { MTLLoader } from '../libs/three/loaders/MTLLoader.js';
import { OrbitControls } from '../libs/three/controls/OrbitControls.js';
import GUI from '../libs/three/lil-gui.module.min.js'; 

// Setting up the scene
const scene = new THREE.Scene();
scene.background = new THREE.Color('gray');

// GUI
const gui = new GUI();
const obj = {
    rotationY: 0 // This will be controlled by the GUI slider
};
gui.add(obj, 'rotationY', 0, Math.PI * 2).name("Rotate Y");

// Get the div where we will attach the renderer
const graphicsDiv = document.getElementById("graphics");

// Set up the camera
const camera = new THREE.PerspectiveCamera(75, graphicsDiv.clientWidth / graphicsDiv.clientHeight, 0.1, 1000000);
camera.position.set(0, 1000, 40000); 
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

// Global variable to store the loaded model
let loadedObject = null;

// Load the model using MTLLoader and OBJLoader
const mtlLoader = new MTLLoader();
mtlLoader.load('../crane_model.mtl', (materials) => {
    materials.preload();

    const objLoader = new OBJLoader();
    objLoader.setMaterials(materials);
    objLoader.load('../crane_model.obj', (object) => {
        loadedObject = object; // Store reference globally
        object.position.set(0, 0, 0);
        scene.add(object);
        console.log("Model loaded successfully.");
    }, undefined, (error) => {
        console.error("Error loading OBJ file:", error);
    });
}, undefined, (error) => {
    console.error("Error loading MTL file:", error);
});

// Animation loop
function animate() {
    requestAnimationFrame(animate);

    if (loadedObject) {
        loadedObject.rotation.y = obj.rotationY; // Apply rotation from GUI slider
    }

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
