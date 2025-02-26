import * as THREE from '../libs/three/three.module.js';
import { OrbitControls } from '../libs/three/controls/OrbitControls.js';
import { loadModel, sceneObjects } from '../libs/three/loaders/modelLoader.js';
import GUI from '../libs/js/lil-gui.module.min.js'; 
import { Water } from '../libs/three/Water.js';

// Setting up the scene
const scene = new THREE.Scene();
scene.background = new THREE.Color('white');
// Set the scene's up vector to Z-up
scene.up.set(0, 1, 0);

// GUI
const gui = new GUI();

// Get the div where we will attach the renderer
const graphicsDiv = document.getElementById("graphics");

// Set up the camera
const camera = new THREE.PerspectiveCamera(
    90, 
    graphicsDiv.clientWidth / graphicsDiv.clientHeight, 
    0.1, 
    1000000
);

camera.position.set(0, 50, 50);
// Set the camera's up vector to Z-up
camera.up.set(0, 1,0);
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
// Load Models
// -------------------------------

// Load Base Object (Part1) - No Parent
loadModel('Tower', '../assets/models/tower.obj', '../assets/materials/tower.mtl', { x: 0, y: 15, z: 0 }, scene, gui, null, {
    position: { x: [0, 0], y: [0, 0], z: [0, 0] },
    rotation: { x: [-Math.PI/2, -Math.PI/2], y:[0, 0] , z: [0, 0] }
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

// Load Part4 (Child of Part2)
//loadModel('Trolley', '../assets/models/Block.obj', '../assets/materials/trolley.mtl', { x: 0, y: 0, z: 0 }, scene, gui, 'Boom', {
//    position: { x: [0, 20], y: [0, 0], z: [0, 0] },
//    rotation: { x: [0, 0], y: [0, 0], z: [0, 0] }
//});

// -------------------------------
// Sort GUI Folders by Name
// -------------------------------
function sortGuiFoldersByName(gui) {
    const guiContainer = gui.domElement;
    const folders = Array.from(guiContainer.querySelectorAll('.lil-gui-folder'));
    folders.sort((a, b) => {
        const titleA = a.querySelector('.title')?.textContent.trim() || '';
        const titleB = b.querySelector('.title')?.textContent.trim() || '';
        return titleA.localeCompare(titleB, undefined, { numeric: true });
    });
    folders.forEach(folder => {
        guiContainer.appendChild(folder);
    });
}

setTimeout(() => {
    sortGuiFoldersByName(gui);
}, 2);

setTimeout(() => {
    sortGuiFoldersByName(gui);
}, 2000);

// Create the water geometry (which by default is in the XY plane)
const waterGeometry = new THREE.PlaneGeometry(10000, 10000);

// Load the water normals texture (adjust path as needed)
const waterNormals = new THREE.TextureLoader().load('../libs/three/WaterTexture.jpg', function (texture) {
    texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
    texture.flipY = false; // keep the normals in the expected orientation
    console.log('Water normals loaded:', texture);
});

// Create the Water object as usual
const water = new Water(waterGeometry, {
    textureWidth: 512,
    textureHeight: 512,
    waterNormals: waterNormals,
    alpha: 1.0,
    // Keep the sun direction as if Y were up for the shader calculations
    sunDirection: new THREE.Vector3(0, 1, 0),
    sunColor: 0xffffff,
    waterColor: 0x00B4D8,
    distortionScale: 3.7,
    fog: scene.fog !== undefined
});

// The shader expects the water to lie in the XZ plane,
// so rotate it by -90° around X:
water.rotation.x = -Math.PI / 2;

// Create a container for the water mesh that will rotate it into the Z‑up system.
// Since our scene uses Z‑up (horizontal = XY plane), we need to rotate the water back.
const waterContainer = new THREE.Object3D();
// Rotate the container by +90° around X so that the water surface lies in the XY plane.
waterContainer.rotation.x = 0;
waterContainer.add(water);
scene.add(waterContainer);

// -------------------------------
// Animation Loop
// -------------------------------
function animate() {
    requestAnimationFrame(animate);
    
    controls.update();
    // Update water's time uniform to animate the water surface
    water.material.uniforms['time'].value += 1.0 / 60.0;

    renderer.render(scene, camera);
}

animate();

// Adjust canvas size on window resize
window.addEventListener("resize", () => {
    renderer.setSize(graphicsDiv.clientWidth, graphicsDiv.clientHeight);
    camera.aspect = graphicsDiv.clientWidth / graphicsDiv.clientHeight;
    camera.updateProjectionMatrix();
});
