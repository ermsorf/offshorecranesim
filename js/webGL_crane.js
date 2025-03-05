import * as THREE from '../libs/three/three.module.js';
import { OrbitControls } from '../libs/three/controls/OrbitControls.js';
import { loadModel } from '../libs/three/loaders/modelLoader.js';


let scene, camera, renderer, objects = [];

export function initgraphics() {
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

    
    
    
    loadModel(crane, '../assets/models/')

    window.addEventListener('resize', onWindowResize, false);
    animate();
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



export function updateStates(positions, rotations) {
    for (let i = 0; i < objects.length; i++) {
        if (positions[i]) {
            let [px, py, pz] = positions[i]; // Extract x, y, z values
            let [mx, my, mz] = positions[i + 1]
            
            objects[i].rod.position.set(px, py, pz);
            objects[i].mass.position.set(mx, my, mz);
        }
        if (rotations[i]) {
            let matrix = new THREE.Matrix4().fromArray(rotations[i]);
            objects[i].rod.setRotationFromMatrix(matrix);
        }
    }
}   







