import { OBJLoader } from './OBJLoader.js';
import { MTLLoader } from './MTLLoader.js';
import * as THREE from '../three.module.js';

// Global dictionaries to store models and their load Promises
export const sceneObjects = {};
export const sceneObjectPromises = {};

/**
 * loadModel - Loads an OBJ model (with its MTL file), positions it in the scene,
 * attaches it to a parent (if specified), sets up GUI controls, and returns a Promise.
 *
 * @param {string} name - Unique name for the model.
 * @param {string} objPath - Path to the OBJ file.
 * @param {string} mtlPath - Path to the MTL file.
 * @param {Object} position - Object with x, y, z properties to set the initial position.
 * @param {THREE.Scene} scene - The Three.js scene where the model will be added.
 * @param {GUI} gui - GUI instance for adding transformation controls.
 * @param {string|null} parentName - (Optional) Name of the parent model.
 * @param {Object} transformLimits - (Optional) Limits for translation and rotation.
 *
 * @returns {Promise} Resolves with the loaded object.
 */
export function loadModel(name, objPath, mtlPath, position, scene, gui, parentName = null, transformLimits = {}) {
  // Create a new Promise for this model load.
  const modelPromise = new Promise((resolve, reject) => {
    const mtlLoader = new MTLLoader();

    // Load the MTL file.
    mtlLoader.load(mtlPath, (materials) => {
      // Prepare materials for the OBJ loader.
      materials.preload();

      const objLoader = new OBJLoader();
      objLoader.setMaterials(materials);

      // Load the OBJ file.
      objLoader.load(
        objPath,
        (object) => {
          // Set the initial position of the object.
          object.position.set(position.x, position.y, position.z);

          // If a parent is specified, wait for its Promise to resolve and attach the object.
          if (parentName) {
            if (sceneObjectPromises[parentName]) {
              sceneObjectPromises[parentName].then((parentObject) => {
                parentObject.add(object);
              });
            } else {
              console.warn(`Parent "${parentName}" not found. Adding "${name}" directly to the scene.`);
              scene.add(object);
            }
          } else {
            // No parent provided; add the object directly to the scene.
            scene.add(object);
          }

          // Store the loaded object in our global dictionary.
          sceneObjects[name] = object;

          // Create a GUI folder for this model.
          const folder = gui.addFolder(name);

          // Use provided transform limits or fall back to default limits.
          const posLimits = transformLimits.position || { x: [-5, 5], y: [-5, 5], z: [-5, 5] };
          const rotLimits = transformLimits.rotation || { x: [-Math.PI, Math.PI], y: [-Math.PI, Math.PI], z: [-Math.PI, Math.PI] };

          // Add translation controls if at least one axis is movable.
          if (posLimits.x[0] !== posLimits.x[1] || posLimits.y[0] !== posLimits.y[1] || posLimits.z[0] !== posLimits.z[1]) {
            const translationFolder = folder.addFolder('Translation');
            if (posLimits.x[0] !== posLimits.x[1])
              translationFolder.add(object.position, 'x', posLimits.x[0], posLimits.x[1], 0.1).name('X');
            if (posLimits.y[0] !== posLimits.y[1])
              translationFolder.add(object.position, 'y', posLimits.y[0], posLimits.y[1], 0.1).name('Y');
            if (posLimits.z[0] !== posLimits.z[1])
              translationFolder.add(object.position, 'z', posLimits.z[0], posLimits.z[1], 0.1).name('Z');
          }

          // Add rotation controls if at least one axis is movable.
          if (rotLimits.x[0] !== rotLimits.x[1] || rotLimits.y[0] !== rotLimits.y[1] || rotLimits.z[0] !== rotLimits.z[1]) {
            const rotationFolder = folder.addFolder('Rotation');
            if (rotLimits.x[0] !== rotLimits.x[1])
              rotationFolder.add(object.rotation, 'x', rotLimits.x[0], rotLimits.x[1], 0.01).name('Rotation X');
            if (rotLimits.y[0] !== rotLimits.y[1])
              rotationFolder.add(object.rotation, 'y', rotLimits.y[0], rotLimits.y[1], 0.01).name('Rotation Y');
            if (rotLimits.z[0] !== rotLimits.z[1])
              rotationFolder.add(object.rotation, 'z', rotLimits.z[0], rotLimits.z[1], 0.01).name('Rotation Z');
          }

          // Resolve the Promise with the loaded object.
          resolve(object);
        },
        // onProgress callback (optional)
        undefined,
        // onError callback: reject the Promise if an error occurs.
        reject
      );
    },
    // onError for MTL loading: reject the Promise.
    undefined,
    reject);
  });

  // Store the Promise for this model so that children can wait for it.
  sceneObjectPromises[name] = modelPromise;
  return modelPromise;
}
