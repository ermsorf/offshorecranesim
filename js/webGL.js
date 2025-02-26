import * as THREE from '../libs/three/three.module.js';
import { OrbitControls } from '../libs/three/controls/OrbitControls.js';
import { loadModel, sceneObjects } from '../libs/three/loaders/modelLoader.js';
import { sceneTransformList } from '../libs/three/loaders/modelLoader.js';
import GUI from '../libs/js/lil-gui.module.min.js'; 
import { Water } from '../libs/three/Water.js';

// Setting up the scene
const scene = new THREE.Scene();
scene.background = new THREE.Color('white');
// Set the scene's up vector to Z-up
scene.up.set(0, 1, 0);

// GUI
const gui = new GUI();

// Get the div where we will attach the renderer
const graphicsDiv = document.getElementById("graphics");

// Set up the camera
const camera = new THREE.PerspectiveCamera(
    90, 
    graphicsDiv.clientWidth / graphicsDiv.clientHeight, 
    0.1, 
    1000000
);

camera.position.set(0, 50, 50);
// Set the camera's up vector to Z-up
camera.up.set(0, 1,0);
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


// -------------------------------
// Load Models
// -------------------------------

// Load Base Object (Tower) - No Parent
loadModel(
    'Tower',
    '../assets/models/tower.obj',
    '../assets/materials/tower.mtl',
    { x: 0, y: 15, z: 0 },
    scene,
    null, // No parent
    {
      position: { },
      rotation: { x: [-Math.PI/2, -Math.PI/2]}
    }
  );
  
  // Load Part 2 (Child of Tower) - Boom
  loadModel(
    'Boom',
    '../assets/models/boom.obj',
    '../assets/materials/boom.mtl',
    { x: 0, y: 0, z: 0 },
    scene,
    'Tower', // Parent is Tower
    {
      position: {  },
      rotation: { z: [-Math.PI/2, Math.PI/2] }
    }
  );
  
  // Load Part3 (Child of Boom) - Trolley
  loadModel(
    'Trolley',
    '../assets/models/trolley.obj',
    '../assets/materials/trolley.mtl',
    { x: 0, y: 0, z: 0 },
    scene,
    'Boom', // Parent is Boom
    {
      position: { x: [0, 20] },
      rotation: {  }
    }
  );
  

  


/**
 * updateTransformsWithTrig - Updates each active transform (from sceneTransformList)
 * using a trigonometric function based on time.
 *
 * For translations, it uses Math.sin(time) * amplitude.
 * For rotations, it uses Math.cos(time) * amplitude (in radians).
 *
 * @param {number} time - The current time (for example, from performance.now()).
 */

function updateTransformsWithTrig(time) {
    const newValues = {};
    sceneTransformList.forEach(key => {
      // Expected key format: <ObjectName><Translation|Rotation><Axis>
      const match = key.match(/^([A-Za-z0-9]+)(Translation|Rotation)([XYZ])$/);
      if (match) {
        const transformType = match[2];
        let newVal = 0;
        if (transformType === 'Translation') {
          // For translation, use sine (amplitude = 10 units, adjust as needed)
          newVal = Math.sin(time * 0.001) * 10;
        } else if (transformType === 'Rotation') {
          // For rotation, use cosine (amplitude = PI/4 radians, i.e., 45°)
          newVal = Math.cos(time * 0.001) * (Math.PI / 4);
        }
        newValues[key] = newVal;
      }
    });
    updateTransformListValues(newValues);
  }
  

/**
 * updateTransformListValues - Updates the scene objects based on a mapping of transform keys to new values.
 * Expected key format: "<ModelName><Translation|Rotation><Axis>", e.g. "TrolleyTranslationX" or "BoomRotationZ".
 * This function parses the key and updates the corresponding property on the model.
 */
function updateTransformListValues(newValues) {
    for (const key in newValues) {
      const newVal = newValues[key];
      // Match keys like "TrolleyTranslationX" or "BoomRotationZ"
      const match = key.match(/^([A-Za-z0-9]+)(Translation|Rotation)([XYZ])$/);
      if (match) {
        const objectName = match[1];
        const transformType = match[2];
        const axis = match[3].toLowerCase();
        // Ensure that sceneObjects is available; import it if needed
        const obj = sceneObjects[objectName];
        if (!obj) {
          console.warn(`Object "${objectName}" not found in sceneObjects.`);
          continue;
        }
        if (transformType === "Translation") {
          obj.position[axis] = newVal;
        } else if (transformType === "Rotation") {
          obj.rotation[axis] = newVal;
        }
      } else {
        console.warn(`Key "${key}" does not match the expected format.`);
      }
    }
  }
  



// -------------------------------
// Sort GUI Folders by Name
// -------------------------------
function sortGuiFoldersByName(gui) {
    const guiContainer = gui.domElement;
    const folders = Array.from(guiContainer.querySelectorAll('.lil-gui-folder'));
    folders.sort((a, b) => {
        const titleA = a.querySelector('.title')?.textContent.trim() || '';
        const titleB = b.querySelector('.title')?.textContent.trim() || '';
        return titleA.localeCompare(titleB, undefined, { numeric: true });
    });
    folders.forEach(folder => {
        guiContainer.appendChild(folder);
    });
}

setTimeout(() => {
    sortGuiFoldersByName(gui);
}, 2);

setTimeout(() => {
    sortGuiFoldersByName(gui);
}, 2000);

// Create the water geometry (which by default is in the XY plane)
const waterGeometry = new THREE.PlaneGeometry(10000, 10000);

// Load the water normals texture (adjust path as needed)
const waterNormals = new THREE.TextureLoader().load('../libs/three/WaterTexture.jpg', function (texture) {
    texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
    texture.flipY = false; // keep the normals in the expected orientation
    console.log('Water normals loaded:', texture);
});

// Create the Water object as usual
const water = new Water(waterGeometry, {
    textureWidth: 512,
    textureHeight: 512,
    waterNormals: waterNormals,
    alpha: 1.0,
    // Keep the sun direction as if Y were up for the shader calculations
    sunDirection: new THREE.Vector3(0, 1, 0),
    sunColor: 0xffffff,
    waterColor: 0x00B4D8,
    distortionScale: 3.7,
    fog: scene.fog !== undefined
});

// The shader expects the water to lie in the XZ plane,
// so rotate it by -90° around X:
water.rotation.x = -Math.PI / 2;

// Create a container for the water mesh that will rotate it into the Z‑up system.
// Since our scene uses Z‑up (horizontal = XY plane), we need to rotate the water back.
const waterContainer = new THREE.Object3D();
// Rotate the container by +90° around X so that the water surface lies in the XY plane.
waterContainer.rotation.x = 0;
waterContainer.add(water);
scene.add(waterContainer);

// -------------------------------
// Animation Loop
// -------------------------------
function animate() {
    requestAnimationFrame(animate);
    const time = performance.now();
    updateTransformsWithTrig(time);
    controls.update();
    // Update water's time uniform to animate the water surface
    water.material.uniforms['time'].value += 1.0 / 60.0;

    renderer.render(scene, camera);
}


animate();

// Adjust canvas size on window resize
window.addEventListener("resize", () => {
    renderer.setSize(graphicsDiv.clientWidth, graphicsDiv.clientHeight);
    camera.aspect = graphicsDiv.clientWidth / graphicsDiv.clientHeight;
    camera.updateProjectionMatrix();
});

setTimeout(() => {
    console.log("Scene Transform List:", sceneTransformList);
  }, 3000);