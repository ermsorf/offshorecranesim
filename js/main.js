import * as THREE from '../libs/three/three.module.js';
import { OBJLoader } from '../libs/three/loaders/OBJLoader.js';
import { MTLLoader } from '../libs/three/loaders/MTLLoader.js';
import { TrackballControls } from '../libs/three/controls/TrackballControls.js'

const scene = new THREE.Scene();
scene.background = new THREE.Color('gray');

const graphicsDiv = document.getElementById("graphics");
const camera = new THREE.PerspectiveCamera(75, graphicsDiv.clientWidth / graphicsDiv.clientHeight, 0.1, 1000000);
camera.up.set
// Move the camera further back and also higher up

// Make the camera look at the center of the scene
camera.lookAt(0, 1000, 0);

const renderer = new THREE.WebGLRenderer();
renderer.setSize(graphicsDiv.clientWidth, graphicsDiv.clientHeight);
graphicsDiv.appendChild(renderer.domElement);

const controls = new TrackballControls( camera, renderer.domElement );
//controls.update() must be called after any manual changes to the camera's transform
camera.position.set( 0, 20, 100 );
controls.update();

// Add lighting to the scene
const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
directionalLight.position.set(50, 50, 50).normalize();
scene.add(directionalLight);

// Load the model using MTLLoader and OBJLoader
const mtlLoader = new MTLLoader();
mtlLoader.load('../crane_model.mtl', (materials) => {
  materials.preload();

  const objLoader = new OBJLoader();
  objLoader.setMaterials(materials);
  objLoader.load('../crane_model.obj', (object) => {
    object.position.set(0, 0, 0); // Adjust if necessary
    scene.add(object);
    console.log("Model loaded successfully.");

    // Initial render
    renderer.render(scene, camera);
  }, undefined, (error) => {
    console.error("Error loading OBJ file: ", error);
  });
}, undefined, (error) => {
  console.error("Error loading MTL file: ", error);
});

// Function to animate and render the scene
function animate() {
	requestAnimationFrame( animate );
	// required if controls.enableDamping or controls.autoRotate are set to true
	controls.update();
	renderer.render( scene, camera );

}
animate();

// Adjust on resize
window.addEventListener("resize", () => {
  renderer.setSize(graphicsDiv.clientWidth, graphicsDiv.clientHeight);
  camera.aspect = graphicsDiv.clientWidth / graphicsDiv.clientHeight;
  camera.updateProjectionMatrix();
});
