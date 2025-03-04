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

    // Create pendulum rods
    createPendulum();

    window.addEventListener('resize', onWindowResize, false);
    animate();
}

function createPendulum(count = 1) {
    const rodMaterial = new THREE.MeshBasicMaterial({ color: 0x0077ff });
    // Create the cylinder geometry, then shift it so its top is at y = 0.
    const rodGeometry = new THREE.CylinderGeometry(0.05, 0.05, 2);
    rodGeometry.translate(0, -1, 0);
  
    // First pendulum
    const pivot1 = new THREE.Group();
    pivot1.position.set(0, 0, 0);
    scene.add(pivot1);
  
    const rod1 = new THREE.Mesh(rodGeometry, rodMaterial);
    rod1.rotation.z = Math.PI / 2;
    pivot1.add(rod1);
  
    pivot1.userData.initialRotation = new THREE.Matrix4().copy(pivot1.matrixWorld);
    pendulumObjects.push(pivot1);
  
    if (count === 2) {
      // Second pendulum: attach its pivot at the end of rod1.
      const pivot2 = new THREE.Group();
      // Since rod1 extends 2 units from the pivot (after rotation), position pivot2 at (2, 0, 0).
      pivot2.position.set(2, 0, 0);
      pivot1.add(pivot2);
  
      // Use a clone of the original geometry for the second rod.
      const rod2 = new THREE.Mesh(rodGeometry.clone(), rodMaterial);
      rod2.rotation.z = Math.PI / 2;
      pivot2.add(rod2);
  
      pivot2.userData.initialRotation = new THREE.Matrix4().copy(pivot2.matrixWorld);
      pendulumObjects.push(pivot2);
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
            let [px, py, pz] = positions[i];
            pendulumObjects[i].position.set(px, py, pz);
        }

        if (rotations[i]) {
            let rotationMatrix = new THREE.Matrix4().fromArray(rotations[i].flat());
            pendulumObjects[i].setRotationFromMatrix(rotationMatrix);
        }
    }
}
