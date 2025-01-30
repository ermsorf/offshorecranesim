import * as THREE from '../libs/three/three.module.js';
import { OrbitControls } from '../libs/three/controls/OrbitControls.js';
import GUI from '../libs/three/lil-gui.module.min.js'; 


// Setting up the scene
const scene = new THREE.Scene();
scene.background = new THREE.Color('white');

// GUI
const gui = new GUI();
const obj = {
    rotationY: 0 // This will be controlled by the GUI slider
};
gui.add(obj, 'rotationY', 0, Math.PI * 2).name("Rotate Y");

// Get the div where we will attach the renderer
const graphicsDiv = document.getElementById("graphics");

// Set up the camera
const camera = new THREE.PerspectiveCamera(75, graphicsDiv.clientWidth / graphicsDiv.clientHeight, 0.1, 1000000);
camera.position.set(0, 10,10) 
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




function loadModel(objPath, mtlPath, position = { x: 0, y: 0, z: 0 }, scene) {
    const mtlLoader = new MTLLoader();
    
    mtlLoader.load(mtlPath, (materials) => {
        materials.preload();

        const objLoader = new OBJLoader();
        objLoader.setMaterials(materials);

        objLoader.load(objPath, (object) => {
            object.position.set(position.x, position.y, position.z);
            scene.add(object);
        }, 
        (xhr) => console.log(`Loading ${objPath}: ${(xhr.loaded / xhr.total) * 100}% loaded`), 
        (error) => console.error(`Error loading ${objPath}:`, error));
    });
}

// Global variable to store the loaded model
let loadedObject = null;

loadModel('../models/crane_assembly.obj', '../models/crane_assembly.mtl', { x: -0, y: 0, z: 0 }, scene);



// Animation loop
function animate() {
    requestAnimationFrame(animate);

    if (loadedObject) {
        loadedObject.rotation.y = obj.rotationY; // Apply rotation from GUI slider
    }
    
    scene.add(curveObject);

    controls.update();
    renderer.render(scene, camera);
}

//Create a closed wavey loop
const curve = new THREE.CatmullRomCurve3( [
	new THREE.Vector3( -obj.rotationY * 10000, 0, 10000 ),
	new THREE.Vector3( -50000, 50000, 50000 ),
	new THREE.Vector3( 0, 0, 0 ),
	new THREE.Vector3( 50000, -50000, 50000 ),
	new THREE.Vector3( 100000, 0, 100000 )
] );

const points = curve.getPoints( 50000 );
const geometry = new THREE.BufferGeometry().setFromPoints( points );

const material = new THREE.LineBasicMaterial( { color: 0xff0000 } );

// Create the final object to add to the scene
const curveObject = new THREE.Line( geometry, material );


animate();

// Adjust canvas size on window resize
window.addEventListener("resize", () => {
    renderer.setSize(graphicsDiv.clientWidth, graphicsDiv.clientHeight);
    camera.aspect = graphicsDiv.clientWidth / graphicsDiv.clientHeight;
    camera.updateProjectionMatrix();
});
