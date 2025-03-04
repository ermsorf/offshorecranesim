import { OBJLoader } from './OBJLoader.js';
import { MTLLoader } from './MTLLoader.js';
import * as THREE from '../three.module.js';

// Global dictionaries to store models and their load Promises
export const sceneObjects = {};
export const sceneObjectPromises = {};

// Global list that will store transform control keys (e.g. "BodyOneTranslationX")
export const sceneTransformList = [];

/**
 * loadModel - Loads an OBJ model (with its MTL file), positions it in the scene,
 * attaches it to a parent (if specified), applies any fixed transform limits, and
 * creates entries in a global transform list that reflect which translation/rotation axes are configurable.
 *
 * Instead of using GUI sliders, this version creates a list of transform control keys.
 *
 * @param {string} name - Unique name for the model.
 * @param {string} objPath - Path to the OBJ file.
 * @param {string} mtlPath - Path to the MTL file.
 * @param {Object} position - Object with x, y, z properties for the initial position.
 * @param {THREE.Scene} scene - The Three.js scene where the model will be added.
 * @param {string|null} parentName - (Optional) Name of the parent model.
 * @param {Object} transformLimits - (Optional) Limits for translation and rotation.
 *   Expected structure:
 *     {
 *       position: { x: [min, max], y: [min, max], z: [min, max] },
 *       rotation: { x: [min, max], y: [min, max], z: [min, max] }
 *     }
 *   Only axes with provided arrays and where min !== max will be considered “active.”
 *
 * @returns {Promise} Resolves with the loaded object.
 */
export function loadModel(name, objPath, mtlPath, position, scene, parentName = null, transformLimits = {}) {
  const modelPromise = new Promise((resolve, reject) => {
    const mtlLoader = new MTLLoader();

    // Load the MTL file.
    mtlLoader.load(mtlPath, (materials) => {
      materials.preload();

      const objLoader = new OBJLoader();
      objLoader.setMaterials(materials);

      // Load the OBJ file.
      objLoader.load(
        objPath,
        (object) => {
          // Set the initial position.
          object.position.set(position.x, position.y, position.z);

          // Attach to a parent if specified.
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
            scene.add(object);
          }

          // Store the loaded object.
          sceneObjects[name] = object;

          // Apply fixed transform limits (if provided).
          const posLimits = transformLimits.position || {};
          if (posLimits.x && Array.isArray(posLimits.x) && posLimits.x.length === 2 && posLimits.x[0] === posLimits.x[1]) {
            object.position.x = posLimits.x[0];
          }
          if (posLimits.y && Array.isArray(posLimits.y) && posLimits.y.length === 2 && posLimits.y[0] === posLimits.y[1]) {
            object.position.y = posLimits.y[0];
          }
          if (posLimits.z && Array.isArray(posLimits.z) && posLimits.z.length === 2 && posLimits.z[0] === posLimits.z[1]) {
            object.position.z = posLimits.z[0];
          }

          const rotLimits = transformLimits.rotation || {};
          if (rotLimits.x && Array.isArray(rotLimits.x) && rotLimits.x.length === 2 && rotLimits.x[0] === rotLimits.x[1]) {
            object.rotation.x = rotLimits.x[0];
          }
          if (rotLimits.y && Array.isArray(rotLimits.y) && rotLimits.y.length === 2 && rotLimits.y[0] === rotLimits.y[1]) {
            object.rotation.y = rotLimits.y[0];
          }
          if (rotLimits.z && Array.isArray(rotLimits.z) && rotLimits.z.length === 2 && rotLimits.z[0] === rotLimits.z[1]) {
            object.rotation.z = rotLimits.z[0];
          }


          // Build the transform list entry for this model.
          // For translation, check each axis if a valid array is provided and if the limits differ.
          if (posLimits.x && Array.isArray(posLimits.x) && posLimits.x.length === 2 && posLimits.x[0] !== posLimits.x[1]) {
            sceneTransformList.push(`${name}TranslationX`);
          }
          if (posLimits.y && Array.isArray(posLimits.y) && posLimits.y.length === 2 && posLimits.y[0] !== posLimits.y[1]) {
            sceneTransformList.push(`${name}TranslationY`);
          }
          if (posLimits.z && Array.isArray(posLimits.z) && posLimits.z.length === 2 && posLimits.z[0] !== posLimits.z[1]) {
            sceneTransformList.push(`${name}TranslationZ`);
          }
          // For rotation, do the same.
          if (rotLimits.x && Array.isArray(rotLimits.x) && rotLimits.x.length === 2 && rotLimits.x[0] !== rotLimits.x[1]) {
            sceneTransformList.push(`${name}RotationX`);
          }
          if (rotLimits.y && Array.isArray(rotLimits.y) && rotLimits.y.length === 2 && rotLimits.y[0] !== rotLimits.y[1]) {
            sceneTransformList.push(`${name}RotationY`);
          }
          if (rotLimits.z && Array.isArray(rotLimits.z) && rotLimits.z.length === 2 && rotLimits.z[0] !== rotLimits.z[1]) {
            sceneTransformList.push(`${name}RotationZ`);
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
