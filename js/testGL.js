import * as THREE from '../libs/three/three.module.js';
import { OrbitControls } from '../libs/three/controls/OrbitControls.js';

let scene, camera, renderer, controls;

function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}


export function initializeGraphics(containerId = 'graphics') {
    // Create scene
    scene = new THREE.Scene();

    // Set up camera
    camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 100);
    camera.up.set(0,0,1)
    camera.position.set(2, 5, 5); 

    // Set up renderer
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setClearColor(0x909090); // Dark background for visibility

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
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.screenSpacePanning = true; // Allows panning in the Z-up coordinate system
    controls.minDistance = 1;
    controls.maxDistance = 10;

    // Lighting
    const light = new THREE.DirectionalLight(0xffffff, 1);
    light.position.set(5, 5, 5); 
    scene.add(light);

    // Handle window resizing
    window.addEventListener('resize', () => {
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(window.innerWidth, window.innerHeight);
    });

    animate();
}
