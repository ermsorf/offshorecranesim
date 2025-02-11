import * as THREE from '../libs/three/three.module.js';
import { OrbitControls } from '../libs/three/controls/OrbitControls.js';

let scene, camera, renderer, controls;
let cylinders = [];

export function initializeGraphics(containerId = 'graphics') {
    // Create scene
    scene = new THREE.Scene();

    // Set up camera
    camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 100);
    camera.position.set(0, 2, 5); // Adjusted for better view

    // Set up renderer
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setClearColor(0x202020); // Dark background for visibility

    // Attach to HTML container
    let container = document.getElementById(containerId);
    if (!container) {
        container = document.createElement('div');
        container.id = containerId;
        document.body.appendChild(container);
    }
    container.appendChild(renderer.domElement);

    // OrbitControls for camera movement
    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true; // Smooth controls
    controls.dampingFactor = 0.05;
    controls.screenSpacePanning = false;
    controls.minDistance = 1;
    controls.maxDistance = 10;

    // Lighting
    const light = new THREE.DirectionalLight(0xffffff, 1);
    light.position.set(2, 2, 5);
    scene.add(light);

    // Create two cylinders
    const material = new THREE.MeshStandardMaterial({ color: 0x00ff00 });
    cylinders = []; // Reset array to avoid duplicates

    for (let i = 0; i < 2; i++) {
        const geometry = new THREE.CylinderGeometry(0.1, 0.1, 1, 32);
        const cylinder = new THREE.Mesh(geometry, material);
        cylinder.position.set(i * 0.5 - 0.25, 0, 0); // Adjusted spacing
        scene.add(cylinder);
        cylinders.push(cylinder);
    }

    // Handle window resizing
    window.addEventListener('resize', () => {
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(window.innerWidth, window.innerHeight);
    });

    animate();
}

function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}

export function updateGraphics(cartesianState) {
    console.log("Updating graphics", cartesianState);
    cartesianState.forEach((state, i) => {
        if (cylinders[i]) {
            cylinders[i].position.set(state[0], state[1], state[2]);
            cylinders[i].rotation.set(state[3], state[4], state[5]); // Assuming Euler angles
        }
    });
}
