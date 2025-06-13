import * as THREE from '../libs/three/three.module.js';
import { OrbitControls } from '../libs/three/controls/OrbitControls.js';
import { loadModel, sceneObjects, sceneTransformList } from '../libs/three/loaders/modelLoader.js';
import GUI from '../libs/js/lil-gui.module.min.js';
import { Water } from '../libs/three/Water.js';
import { Sky } from '../libs/three/Sky.js';
import { system } from './main_crane.js';

export function initializeScene(graphicsDiv) {
    const scene = new THREE.Scene();
    scene.background = new THREE.Color('#87CEEB');
    scene.up.set(0, 1, 0);


    const camera = new THREE.PerspectiveCamera(90, graphicsDiv.clientWidth / graphicsDiv.clientHeight, 0.1, 1000000);
    camera.position.set(25, 75, -25);
    camera.up.set(0, 1, 0);
    camera.lookAt(0, 50, 0);

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(graphicsDiv.clientWidth, graphicsDiv.clientHeight);
    graphicsDiv.appendChild(renderer.domElement);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.update();

    const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
    scene.add(ambientLight);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
    directionalLight.position.set(50, 50, 50).normalize();
    scene.add(directionalLight);

    loadModels(scene);
    const water = createWater(scene);


    window.addEventListener("resize", () => {
        renderer.setSize(graphicsDiv.clientWidth, graphicsDiv.clientHeight);
        camera.aspect = graphicsDiv.clientWidth / graphicsDiv.clientHeight;
        camera.updateProjectionMatrix();
    });

    // === Sky setup ===
    const sky = new Sky();
    sky.scale.setScalar(10000);
    scene.add(sky);
    const sun = new THREE.Vector3();
    const phi = THREE.MathUtils.degToRad(70 - 10); // elevation angle
    const theta = THREE.MathUtils.degToRad(180);   // azimuth angle
    sun.setFromSphericalCoords(1, phi, theta);
    sky.material.uniforms['sunPosition'].value.copy(sun);

 

    setTimeout(() => {
        console.log("Scene Transform List:", sceneTransformList);
    }, 3000);

    return { renderer, scene, camera, controls, water };
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
    

    // Wire 1
    if (system.info.wiresegments >= 1) {
    loadModel('Theta1', '../assets/models/wireball.obj', '../assets/materials/wireball.mtl', { x: 9, y: 0, z: 18 }, scene, 'Trolley', {
        position: {},
        rotation: { z: [-Math.PI/2, Math.PI/2] }
    });
    loadModel('Phi1', '../assets/models/wireball.obj', '../assets/materials/wireball.mtl', { x: 0, y: 0, z: 0 }, scene, 'Theta1', {
        position: {},
        rotation: { x: [-Math.PI/2, Math.PI/2] }
    });
    loadModel('Lambda1', '../assets/models/wireball.obj', '../assets/materials/wireball.mtl', { x: 0, y: 0, z: -5 }, scene, 'Phi1', {
        position: { z: [0,1] },
        rotation: {}
    });
    }
    // Wire 2
    if (system.info.wiresegments >= 2) {
    loadModel('Theta2', '../assets/models/wireball.obj', '../assets/materials/wireball.mtl', { x: 0, y: 0, z: 0 }, scene, 'Lambda1', {
        position: {},
        rotation: { z: [-Math.PI/2, Math.PI/2] }
    });
    loadModel('Phi2', '../assets/models/wireball.obj', '../assets/materials/wireball.mtl', { x: 0, y: 0, z: 0 }, scene, 'Theta2', {
        position: {},
        rotation: { x: [-Math.PI/2, Math.PI/2] }
    });
    loadModel('Lambda2', '../assets/models/wireball.obj', '../assets/materials/wireball.mtl', { x: 0, y: 0, z: -5 }, scene, 'Phi2', {
        position: { z: [0,1] },
        rotation: {}
    });
    }
    if (system.info.wiresegments >= 3) {
    // Wire 3
    loadModel('Theta3', '../assets/models/wireball.obj', '../assets/materials/wireball.mtl', { x: 0, y: 0, z: 0 }, scene, 'Lambda2', {
        position: {},
        rotation: { z: [-Math.PI/2, Math.PI/2] }
    });
    loadModel('Phi3', '../assets/models/wireball.obj', '../assets/materials/wireball.mtl', { x: 0, y: 0, z: 0 }, scene, 'Theta3', {
        position: {},
        rotation: { x: [-Math.PI/2, Math.PI/2] }
    });
    loadModel('Lambda3', '../assets/models/wireball.obj', '../assets/materials/wireball.mtl', { x: 0, y: 0, z: -5 }, scene, 'Phi3', {
        position: { z: [0,1] },
        rotation: {}
    });
    }

    loadModel('Container', '../assets/models/container.obj', '../assets/materials/container.mtl', { x: 0, y: 0, z: -3 }, scene, `Lambda${system.info.wiresegments}`, {
        position: {},
        rotation: { z: [-Math.PI/2, Math.PI/2] }
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

let wireLine = null;

export function createOrUpdateWire(points, scene) {
    if (!wireLine) {
        const geometry = new THREE.BufferGeometry().setFromPoints(points);
        const material = new THREE.LineBasicMaterial({ color: 0x000000 });
        wireLine = new THREE.Line(geometry, material);
        scene.add(wireLine);
    } else {
        wireLine.geometry.setFromPoints(points);
        wireLine.geometry.attributes.position.needsUpdate = true;
    }
}