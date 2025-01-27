import * as THREE from '../libs/three/three.module.js';
import { OrbitControls } from '../libs/three/controls/OrbitControls.js';

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(75, window.innerWidth / 2 / window.innerHeight, 0.1, 1000);
const renderer = new THREE.WebGLRenderer();

const graphicsDiv = document.getElementById("graphics");
renderer.setSize(graphicsDiv.clientWidth, graphicsDiv.clientHeight);
graphicsDiv.appendChild(renderer.domElement);

const BoxGeometry = new THREE.BoxGeometry();
const BoxMaterial = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
const cube = new THREE.Mesh(BoxGeometry, BoxMaterial)
scene.add(cube);

camera.position.z = 2;


function animate() {
    requestAnimationFrame(animate);
    cube.rotation.x += 0.01;
    cube.rotation.y += 0.01;
    renderer.render(scene, camera);
}
animate();

// Adjust on resize
window.addEventListener("resize", () => {
    renderer.setSize(graphicsDiv.clientWidth, graphicsDiv.clientHeight);
    camera.aspect = graphicsDiv.clientWidth / graphicsDiv.clientHeight;
    camera.updateProjectionMatrix();
});