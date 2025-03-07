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




export function loadModels(scene) {
    loadModel('Tower', '../assets/models/tower.obj', '../assets/materials/tower.mtl', { x: 0, y: 40, z: 0 }, scene, null, {
        position: {},
        rotation: { x: [-Math.PI / 2, -Math.PI / 2] }
    });
    
    loadModel('Platform', '../assets/models/Platform.obj', '../assets/materials/Platform.mtl', { x: 0, y: 40, z: 0 }, scene, null, {
        position: {},
        rotation: { x: [-Math.PI / 2, -Math.PI / 2] }
    });

    loadModel('Boom', '../assets/models/boom.obj', '../assets/materials/boom.mtl', { x: 0, y: 0, z: 0 }, scene, 'Tower', {
        position: {},
        rotation: { z: [-Math.PI / 2, Math.PI / 2] }
    });

    loadModel('Trolley', '../assets/models/trolley.obj', '../assets/materials/trolley.mtl', { x: 0, y: 0, z: 0 }, scene, 'Boom', {
        position: { x: [0, 30] },
        rotation: {}
    });

}



export function createWater(scene) {
    const waterGeometry = new THREE.PlaneGeometry(10000, 10000);
    const waterNormals = new THREE.TextureLoader().load('../libs/three/WaterTexture.jpg', texture => {
        texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
        texture.flipY = false;
    });
    
    const water = new Water(waterGeometry, {
        textureWidth: 512,
        textureHeight: 512,
        waterNormals: waterNormals,
        alpha: 1.0,
        sunDirection: new THREE.Vector3(1, 0, 0),
        sunColor: 0xffffff,
        waterColor: 0x00B4D8,
        distortionScale: 3.7,
        fog: scene.fog !== undefined,
    });

    water.rotation.x = -Math.PI / 2;
    const waterContainer = new THREE.Object3D();
    waterContainer.add(water);
    scene.add(waterContainer);
    return water;
}

export function updateTransformsWithTrig(time) {
    const newValues = {};
    sceneTransformList.forEach(key => {
        const match = key.match(/^([A-Za-z0-9]+)(Translation|Rotation)([XYZ])$/);
        if (match) {
            const transformType = match[2];
            let newVal = (transformType === 'Translation')
                ? Math.sin(time * 0.0001) * 10 + 10
                : Math.cos(time * 0.0001) * (Math.PI / 4);
            newValues[key] = newVal;
        }
    });
    updateTransformListValues(newValues);
}

export function updateTransformListValues(newValues) {
    for (const key in newValues) {
        const newVal = newValues[key];
        const match = key.match(/^([A-Za-z0-9]+)(Translation|Rotation)([XYZ])$/);
        if (match) {
            const objectName = match[1];
            const transformType = match[2];
            const axis = match[3].toLowerCase();
            const obj = sceneObjects[objectName];
            if (!obj) {
                console.warn(`Object "${objectName}" not found in sceneObjects.`);
                continue;
            }
            obj[transformType === "Translation" ? "position" : "rotation"][axis] = newVal;
        } else {
            console.warn(`Key "${key}" does not match the expected format.`);
        }
    }
}   







