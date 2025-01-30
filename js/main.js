import * as THREE from '../libs/three/three.module.js';
import { OrbitControls } from '../libs/three/controls/OrbitControls.js';
import GUI from '../libs/three/lil-gui.module.min.js'; 

import { loadModel, sceneObjects } from './modelLoader.js';

// Setting up the scene
const scene = new THREE.Scene();
scene.background = new THREE.Color('white');

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
camera.position.set(0, 10,10) 
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



loadModel('PartOne', '../models/crane_assembly.obj', '../models/crane_assembly.mtl', { x: -2, y: 0, z: 0 }, scene, gui);


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
