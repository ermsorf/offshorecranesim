import * as THREE from '../libs/js/Three.js';
import * as OrbitControls from '../libs/js//controls/OrbitControls.js';

// Create scene, camera, renderer
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(75, window.innerWidth / 2 / window.innerHeight, 0.1, 1000);
const renderer = new THREE.WebGLRenderer();


// Add OrbitControls
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

// Fit to graphicsDiv specified in index
const graphicsDiv = document.getElementById("graphics");
renderer.setSize(graphicsDiv.clientWidth, graphicsDiv.clientHeight);
graphicsDiv.appendChild(renderer.domElement);

// Renderer settings
renderer.shadowMap.enabled = true;


const BoxGeometry = new THREE.BoxGeometry();
const BoxMaterial = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
const cube = new THREE.Mesh(BoxGeometry, BoxMaterial)


const planeGeometry = new THREE.PlaneGeometry(10,10);
const planeMaterial = new THREE.MeshBasicMaterial({ color: 0xAAAAAA })
const plane = new THREE.Mesh(planeGeometry, planeMaterial);
plane.rotation.set(Math.PI/2,0,0)

scene.add(plane);
scene.add(cube);

// Position the camera
camera.position.set(0, 2, 5);
camera.lookAt(0,0,0)

// Animation loop
function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}
animate();

// Adjust on resize
window.addEventListener("resize", () => {
    renderer.setSize(graphicsDiv.clientWidth, graphicsDiv.clientHeight);
    camera.aspect = graphicsDiv.clientWidth / graphicsDiv.clientHeight;
    camera.updateProjectionMatrix();
});