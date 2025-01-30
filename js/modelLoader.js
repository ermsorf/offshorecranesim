import { OBJLoader } from '../libs/three/loaders/OBJLoader.js';
import { MTLLoader } from '../libs/three/loaders/MTLLoader.js';
import * as THREE from '../libs/three/three.module.js';



export const sceneObjects = {}; // Store models by name

export function loadModel(name, objPath, mtlPath, position = { x: 0, y: 0, z: 0 }, scene, gui) {
    const mtlLoader = new MTLLoader();
    
    mtlLoader.load(mtlPath, (materials) => {
        materials.preload();

        const objLoader = new OBJLoader();
        objLoader.setMaterials(materials);

        objLoader.load(objPath, (object) => {
            object.position.set(position.x, position.y, position.z);
            scene.add(object);
            
            sceneObjects[name] = object; // Store object by name
            console.log(`Loaded: ${name}`, object);

            // Add lil-gui controls for position and rotation
            const folder = gui.addFolder(name);
            folder.add(object.position, 'x', -5, 5, 0.1).name('Position X');
            folder.add(object.position, 'y', -5, 5, 0.1).name('Position Y');
            folder.add(object.position, 'z', -5, 5, 0.1).name('Position Z');

            folder.add(object.rotation, 'x', -Math.PI, Math.PI, 0.01).name('Rotation X');
            folder.add(object.rotation, 'y', -Math.PI, Math.PI, 0.01).name('Rotation Y');
            folder.add(object.rotation, 'z', -Math.PI, Math.PI, 0.01).name('Rotation Z');
        }, 
        (xhr) => console.log(`Loading ${name}: ${(xhr.loaded / xhr.total) * 100}% loaded`), 
        (error) => console.error(`Error loading ${name}:`, error));
    });
}