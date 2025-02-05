import * as THREE from '../libs/three/three.module.js';
import { OBJLoader } from '../libs/three/loaders/OBJLoader.js'
import { GLTFLoader } from '../libs/three/loaders/GLTFLoader.js';
import { STLLoader } from '../libs/three/loaders/STLLoader.js';
import { TrackballControls } from '../libs/three/controls/TrackballControls.js';
import { OrbitControls } from '../libs/three/controls/OrbitControls.js';


const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(70, window.innerWidth / 2 / window.innerHeight, 0.1, 1000);
const renderer = new THREE.WebGLRenderer();


const graphicsDiv = document.getElementById("graphics");
renderer.setSize(graphicsDiv.clientWidth, graphicsDiv.clientHeight);
graphicsDiv.appendChild(renderer.domElement);
// --------------------
renderer.setClearColor(0xDDDDDD, 1.0);
renderer.shadowMap.enabled = true;

camera.up.set(0,0,1)
camera.position.set(100, 100, 100);


var light = new THREE.SpotLight(0xffffff);   //create light
  light.position.set(0, 0, 300);
  light.castShadow = true;              // Allow light to cast shadows
  light.intensity = 200000;                  // Set light intensity
  light.distance = 500                // Set light travel distance
  light.shadow.mapSize.width = 4096;  // Default is 512
  light.shadow.mapSize.height = 4096; // Default is 512
  light.shadow.radius = 1; // Softens the shadow edges
  scene.add(light)


camera.up.set(0,0,1)
camera.position.set(50, 50, 50);
camera.lookAt(0, 0, 0);


const controls = new OrbitControls(camera, renderer.domElement); // Initialize OrbitControls
controls.enableDamping = true; // Smooth damping for controls
controls.dampingFactor = 0.25; // Controls damping factor (smoothness)
controls.screenSpacePanning = true; // Allow panning along the screen axes
controls.enableRotate = true;

const light = new THREE.SpotLight(0xffffff);   //create light
  light.position.set(100, 100, 300);
  light.castShadow = true;              // Allow light to cast shadows
  light.intensity = 200000;                  // Set light intensity
  light.distance = 500                // Set light travel distance
  light.shadow.mapSize.width = 4096;  
  light.shadow.mapSize.height = 4096; 
  light.shadow.radius = 1; // Softens the shadow edges
  scene.add(light)


scene.add(plane)

// Animation loop
function animate() {
    requestAnimationFrame(animate);
    renderer.render(scene, camera);
    
}
animate();

// Adjust on resize
window.addEventListener("resize", () => {
    renderer.setSize(graphicsDiv.clientWidth, graphicsDiv.clientHeight);
    camera.aspect = graphicsDiv.clientWidth / graphicsDiv.clientHeight;
    camera.updateProjectionMatrix();
});