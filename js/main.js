import * as THREE from '../libs/three/three.module.js';

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(70, window.innerWidth / 2 / window.innerHeight, 0.1, 1000);
const renderer = new THREE.WebGLRenderer();

const graphicsDiv = document.getElementById("graphics");
renderer.setSize(graphicsDiv.clientWidth, graphicsDiv.clientHeight);
graphicsDiv.appendChild(renderer.domElement);
// --------------------
renderer.setClearColor(0xDDDDDD, 1.0);

camera.position.set(500, 500, 500);
camera.lookAt(0, 0, 0);
camera.eulerOrder = “XZY”;


// Adding objects to the scene:
const BoxGeometry = new THREE.BoxGeometry(100, 100, 100, 10, 10, 10);
const BoxMaterial = new THREE.MeshBasicMaterial({color: 0x00ff00});
const cube = new THREE.Mesh(BoxGeometry, BoxMaterial);
cube.position.set(0, 0, 100);  // Raise the cube above the plane
scene.add(cube);

const planeGeometry = new THREE.PlaneGeometry(200, 200);  // Make the plane larger to see the effect better
const planeMaterial = new THREE.MeshBasicMaterial({color: 0xAAAAAA});
const plane = new THREE.Mesh(planeGeometry, planeMaterial);
plane.rotation.set(0, 0, 0);  // Rotate the plane to lay flat along XZ plane
plane.position.set(0, 0, 0); 
scene.add(plane);

var light = new THREE.SpotLight(0xffffff);   //create light
  light.position.set(0, 0, 300);
  light.castShadow = true;              // Allow light to cast shadows
  light.intensity = 1;                  // Set light intensity
  light.distance = 1000;                // Set light travel distance


// Animation loop
function animate() {
    requestAnimationFrame(animate);
    cube.rotation.x += 0.005;
    cube.rotation.y += 0.005;
    renderer.render(scene, camera);
}
animate();

// Adjust on resize
window.addEventListener("resize", () => {
    renderer.setSize(graphicsDiv.clientWidth, graphicsDiv.clientHeight);
    camera.aspect = graphicsDiv.clientWidth / graphicsDiv.clientHeight;
    camera.updateProjectionMatrix();
});