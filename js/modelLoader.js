import { OBJLoader } from '../libs/three/loaders/OBJLoader.js';
import { MTLLoader } from '../libs/three/loaders/MTLLoader.js';
import * as THREE from '../libs/three/three.module.js';



export const sceneObjects = {}; // Store models by name


export function loadModel(name, objPath, mtlPath, position, scene, gui, parentName = null, transformLimits = {}) {
    const mtlLoader = new MTLLoader();

    mtlLoader.load(mtlPath, (materials) => {
        materials.preload();

        const objLoader = new OBJLoader();
        objLoader.setMaterials(materials);

        objLoader.load(objPath, (object) => {
            object.position.set(position.x, position.y, position.z);

            // Attach to parent if specified
            if (parentName && sceneObjects[parentName]) {
                sceneObjects[parentName].add(object);
            } else {
                scene.add(object);
            }

            sceneObjects[name] = object;

            // Set up GUI controls with specified limits
            const folder = gui.addFolder(name);

            // Default limits if not provided
            const posLimits = transformLimits.position || { x: [-5, 5], y: [-5, 5], z: [-5, 5] };
            const rotLimits = transformLimits.rotation || { x: [-Math.PI, Math.PI], y: [-Math.PI, Math.PI], z: [-Math.PI, Math.PI] };

            // Add position folder only if at least one axis is movable
            if (posLimits.x[0] !== posLimits.x[1] || posLimits.y[0] !== posLimits.y[1] || posLimits.z[0] !== posLimits.z[1]) {
                const positionFolder = folder.addFolder('Translation');
                if (posLimits.x[0] !== posLimits.x[1]) positionFolder.add(object.position, 'x', posLimits.x[0], posLimits.x[1], 0.1).name('X');
                if (posLimits.y[0] !== posLimits.y[1]) positionFolder.add(object.position, 'y', posLimits.y[0], posLimits.y[1], 0.1).name('Y');
                if (posLimits.z[0] !== posLimits.z[1]) positionFolder.add(object.position, 'z', posLimits.z[0], posLimits.z[1], 0.1).name('Z');
            }

            // Add rotation folder only if at least one axis is movable
            if (rotLimits.x[0] !== rotLimits.x[1] || rotLimits.y[0] !== rotLimits.y[1] || rotLimits.z[0] !== rotLimits.z[1]) {
                const rotationFolder = folder.addFolder('Rotation');
                if (rotLimits.x[0] !== rotLimits.x[1]) rotationFolder.add(object.rotation, 'x', rotLimits.x[0], rotLimits.x[1], 0.01).name('Rotation X');
                if (rotLimits.y[0] !== rotLimits.y[1]) rotationFolder.add(object.rotation, 'y', rotLimits.y[0], rotLimits.y[1], 0.01).name('Rotation Y');
                if (rotLimits.z[0] !== rotLimits.z[1]) rotationFolder.add(object.rotation, 'z', rotLimits.z[0], rotLimits.z[1], 0.01).name('Rotation Z');
            }
        });
    });
}

