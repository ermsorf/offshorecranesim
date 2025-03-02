import * as THREE from '../libs/three/three.module.js';
import { OrbitControls } from '../libs/three/controls/OrbitControls.js';

let scene, camera, renderer, pendulumObjects = [];

export function initThreeJS() {
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0xeeeeee); // Black background
    const plane = new THREE.Mesh(new THREE.PlaneGeometry(10,10,1,1), new THREE.MeshStandardMaterial({color: 0xff}))
    plane.position.set(0,0, -5)
    scene.add(plane)
    

    camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
    camera.up.set(0,0,1)
    camera.position.set(0, 2, 5);
    
    renderer = new THREE.WebGLRenderer();
    renderer.setSize(window.innerWidth, window.innerHeight);
    document.body.appendChild(renderer.domElement);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;

    const light = new THREE.AmbientLight(0xffffff, 0.8);
    scene.add(light);

    // Create pendulum rods and masses
    createPendulum();

    window.addEventListener('resize', onWindowResize, false);
    animate();
}

function createPendulum() {
    const rodMaterial = new THREE.MeshBasicMaterial({ color: 0x0077ff });
    const massMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });

    for (let i = 0; i < 2; i++) {
        let rod = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, 2), rodMaterial);
        rod.position.y = -1;
        scene.add(rod);

        let mass = new THREE.Mesh(new THREE.SphereGeometry(0.2), massMaterial);
        mass.position.y = -2;
        scene.add(mass);

        pendulumObjects.push({ rod, mass });
    }
}

function onWindowResize() {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
}

function animate() {
    requestAnimationFrame(animate);
    renderer.render(scene, camera);
}
export function updatePendulum(positions, rotations) {
    for (let i = 0; i < pendulumObjects.length; i++) {
        if (positions[i]) {
            let [px, py, pz] = positions[i]; // Extract x, y, z values
            objects[i].position.set(px, py, pz);
        }
        if (rotations[i]) {
            let matrix = new THREE.Matrix4().fromArray(rotations[i]);
            objects[i].setRotationFromMatrix(matrix);
        }
    }
}   