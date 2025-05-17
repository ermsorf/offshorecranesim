import { DefineLoad } from './DefineLoad.js';
import { ListOfObjects } from './ListOfObjects.js';

// ----- Vector and Matrix Helper Functions -----
function vecAdd(a, b) {
  return a.map((val, i) => val + b[i]);
}

function vecSubtract(a, b) {
  return a.map((val, i) => val - b[i]);
}

function vecScale(a, scalar) {
  return a.map(val => val * scalar);
}

function vecDot(a, b) {
  return a.reduce((sum, val, i) => sum + val * b[i], 0);
}

function vecNorm(a) {
  return Math.sqrt(vecDot(a, a));
}

function vecNormalize(a) {
  const n = vecNorm(a);
  return n > 1e-6 ? vecScale(a, 1 / n) : a;
}

function vecCross(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0]
  ];
}

// Multiply two 3x3 matrices
function multiplyMatrices(A, B) {
  const result = [];
  for (let i = 0; i < 3; i++) {
    result[i] = [];
    for (let j = 0; j < 3; j++) {
      result[i][j] = 0;
      for (let k = 0; k < 3; k++) {
        result[i][j] += A[i][k] * B[k][j];
      }
    }
  }
  return result;
}

// Multiply a 3x3 matrix by a 3-element vector
function multiplyMatrixVector(M, v) {
  return [
    M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2],
    M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2],
    M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2]
  ];
}

function deg2rad(deg) {
  return deg * (Math.PI / 180);
}

// ----- Rotation Matrix Functions -----
function computeRotationMatrix(rotation) {
  // rotation: [yaw, pitch, roll] in degrees (Tait-Bryan angles)
  const yaw = deg2rad(rotation[0]);
  const pitch = deg2rad(rotation[1]);
  const roll = deg2rad(rotation[2]);

  const Rz = [
    [Math.cos(yaw), -Math.sin(yaw), 0],
    [Math.sin(yaw),  Math.cos(yaw), 0],
    [0, 0, 1]
  ];
  const Ry = [
    [Math.cos(pitch), 0, Math.sin(pitch)],
    [0, 1, 0],
    [-Math.sin(pitch), 0, Math.cos(pitch)]
  ];
  const Rx = [
    [1, 0, 0],
    [0, Math.cos(roll), -Math.sin(roll)],
    [0, Math.sin(roll),  Math.cos(roll)]
  ];
  return multiplyMatrices(multiplyMatrices(Rz, Ry), Rx);
}

function ensureRotationMatrix(shape) {
  // If the shape's rotation is a 1x3 vector (in degrees), convert it to a 3x3 matrix.
  if (shape.rotation) {
    if (Array.isArray(shape.rotation) && shape.rotation.length === 3 && typeof shape.rotation[0] === 'number') {
      shape.rotation = computeRotationMatrix(shape.rotation);
    } else if (
      Array.isArray(shape.rotation) &&
      shape.rotation.length === 3 &&
      !Array.isArray(shape.rotation[0])
    ) {
      throw new Error('Rotation field has invalid dimensions.');
    }
  } else {
    shape.rotation = [
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, 1]
    ];
  }
  return shape;
}

// ----- Collision and Distance Functions -----

function SATCollision(box1, box2) {
  const A = box1.rotation;
  const B = box2.rotation;
  const T = vecSubtract(box2.center, box1.center);
  const axes = [];
  // Face normals for box1
  for (let i = 0; i < 3; i++) {
    axes.push([A[0][i], A[1][i], A[2][i]]);
  }
  // Face normals for box2
  for (let i = 0; i < 3; i++) {
    axes.push([B[0][i], B[1][i], B[2][i]]);
  }
  // Cross product axes
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      let cp = vecCross([A[0][i], A[1][i], A[2][i]], [B[0][j], B[1][j], B[2][j]]);
      if (vecNorm(cp) > 1e-6) {
        cp = vecNormalize(cp);
      }
      axes.push(cp);
    }
  }
  
  let minOverlap = Infinity;
  let collisionAxis = [0, 0, 0];
  for (let i = 0; i < axes.length; i++) {
    const axis = axes[i];
    if (vecNorm(axis) < 1e-6) continue;
    const rA =
      (box1.extents[0] / 2) * Math.abs(vecDot(axis, [A[0][0], A[1][0], A[2][0]])) +
      (box1.extents[1] / 2) * Math.abs(vecDot(axis, [A[0][1], A[1][1], A[2][1]])) +
      (box1.extents[2] / 2) * Math.abs(vecDot(axis, [A[0][2], A[1][2], A[2][2]]));
    const rB =
      (box2.extents[0] / 2) * Math.abs(vecDot(axis, [B[0][0], B[1][0], B[2][0]])) +
      (box2.extents[1] / 2) * Math.abs(vecDot(axis, [B[0][1], B[1][1], B[2][1]])) +
      (box2.extents[2] / 2) * Math.abs(vecDot(axis, [B[0][2], B[1][2], B[2][2]]));
    const overlap = rA + rB - Math.abs(vecDot(T, axis));
    if (overlap < 0) {
      return { collision: false, mtv: [0, 0, 0] };
    } else if (overlap < minOverlap) {
      minOverlap = overlap;
      collisionAxis = axis;
    }
  }
  if (vecDot(T, collisionAxis) < 0) {
    collisionAxis = vecScale(collisionAxis, -1);
  }
  return { collision: true, mtv: vecScale(collisionAxis, minOverlap) };
}

function computeContactPointBoxBox(box1, box2, mtv) {
  const { minDist, cp1, cp2 } = analyticalDistanceBoxBox(box1, box2);
  if (!cp1 || !cp2) return box1.center;
  return vecScale(vecAdd(cp1, cp2), 0.5);
}

function analyticalDistanceBoxBox(box1, box2) {
  const vertices1 = getBoxVertices(box1);
  const vertices2 = getBoxVertices(box2);
  return analyticalDistance(box1, box2, vertices1, vertices2);
}

function analyticalDistance(box1, box2, vertices1, vertices2) {
  let minDist = Infinity, cp1 = null, cp2 = null;
  for (let i = 0; i < vertices1.length; i++) {
    const v = vertices1[i];
    const cp = closestPointOnBox(box2, v);
    const d = vecNorm(vecSubtract(v, cp));
    if (d < minDist) { minDist = d; cp1 = v; cp2 = cp; }
  }
  for (let i = 0; i < vertices2.length; i++) {
    const v = vertices2[i];
    const cp = closestPointOnBox(box1, v);
    const d = vecNorm(vecSubtract(v, cp));
    if (d < minDist) { minDist = d; cp1 = cp; cp2 = v; }
  }
  return { minDist, cp1, cp2 };
}

function getBoxVertices(box) {
  const hx = box.extents[0] / 2;
  const hy = box.extents[1] / 2;
  const hz = box.extents[2] / 2;
  const localVerts = [
    [-hx, -hy, -hz],
    [hx, -hy, -hz],
    [hx, hy, -hz],
    [-hx, hy, -hz],
    [-hx, -hy, hz],
    [hx, -hy, hz],
    [hx, hy, hz],
    [-hx, hy, hz]
  ];
  return localVerts.map(v => {
    const rotated = multiplyMatrixVector(box.rotation, v);
    return vecAdd(rotated, box.center);
  });
}

function closestPointOnBox(box, point) {
  const invRotation = mathTranspose(box.rotation);
  const localPoint = multiplyMatrixVector(invRotation, vecSubtract(point, box.center));
  const halfExt = box.extents.map(e => e / 2);
  const cpLocal = localPoint.map((val, i) => Math.min(Math.max(val, -halfExt[i]), halfExt[i]));
  return vecAdd(multiplyMatrixVector(box.rotation, cpLocal), box.center);
}

function mathTranspose(M) {
  return [
    [M[0][0], M[1][0], M[2][0]],
    [M[0][1], M[1][1], M[2][1]],
    [M[0][2], M[1][2], M[2][2]]
  ];
}

function distanceBoxSphere(box, sph) {
  const cp1 = closestPointOnBox(box, sph.center);
  const d = vecNorm(vecSubtract(sph.center, cp1));
  const minDist = Math.max(0, d - sph.extents[0]);
  const dir = d > 1e-6 ? vecNormalize(vecSubtract(sph.center, cp1)) : [1, 0, 0];
  const cp2 = vecSubtract(sph.center, vecScale(dir, sph.extents[0]));
  return { minDist, cp1, cp2 };
}

function closestPointOnCylinder(cyl, point) {
  const invRotation = mathTranspose(cyl.rotation);
  const localPoint = multiplyMatrixVector(invRotation, vecSubtract(point, cyl.center));
  const cpSide = localPoint.slice();
  cpSide[2] = Math.min(Math.max(localPoint[2], -cyl.extents[1] / 2), cyl.extents[1] / 2);
  const r = Math.sqrt(localPoint[0] ** 2 + localPoint[1] ** 2);
  if (r > 1e-6) {
    cpSide[0] = (cyl.extents[0] / r) * localPoint[0];
    cpSide[1] = (cyl.extents[0] / r) * localPoint[1];
  } else {
    cpSide[0] = cyl.extents[0];
    cpSide[1] = 0;
  }
  let cpCap;
  if (localPoint[2] > cyl.extents[1] / 2) {
    cpCap = localPoint.slice();
    cpCap[2] = cyl.extents[1] / 2;
  } else if (localPoint[2] < -cyl.extents[1] / 2) {
    cpCap = localPoint.slice();
    cpCap[2] = -cyl.extents[1] / 2;
  } else {
    cpCap = cpSide;
  }
  const r_xy = Math.sqrt(cpCap[0] ** 2 + cpCap[1] ** 2);
  if (r_xy > cyl.extents[0]) {
    cpCap[0] = (cyl.extents[0] / r_xy) * cpCap[0];
    cpCap[1] = (cyl.extents[0] / r_xy) * cpCap[1];
  }
  const cpLocal = vecNorm(vecSubtract(localPoint, cpSide)) <= vecNorm(vecSubtract(localPoint, cpCap))
    ? cpSide : cpCap;
  return vecAdd(multiplyMatrixVector(cyl.rotation, cpLocal), cyl.center);
}



function distanceBoxCylinder(box, cyl) {
  const vertices = getBoxVertices(box);
  const halfExt = box.extents.map(e => e / 2);
  const faceCentersLocal = [
    [halfExt[0], 0, 0],
    [-halfExt[0], 0, 0],
    [0, halfExt[1], 0],
    [0, -halfExt[1], 0],
    [0, 0, halfExt[2]],
    [0, 0, -halfExt[2]]
  ];
  const faceCenters = faceCentersLocal.map(fc => vecAdd(multiplyMatrixVector(box.rotation, fc), box.center));
  const boxCandidates = vertices.concat(faceCenters);
  
  const r = cyl.extents[0];
  const numAng = 12;
  const angSamples = [];
  for (let j = 0; j < numAng; j++) {
    angSamples.push(2 * Math.PI * j / numAng);
  }
  const cylSideCandidates = angSamples.map(angle => {
    const localCandidate = [r * Math.cos(angle), r * Math.sin(angle), 0];
    return vecAdd(multiplyMatrixVector(cyl.rotation, localCandidate), cyl.center);
  });
  
  let minDist = Infinity, cpBox = null, cpCyl = null;
  boxCandidates.forEach(v => {
    const cp = closestPointOnCylinder(cyl, v);
    const d = vecNorm(vecSubtract(v, cp));
    if (d < minDist) { minDist = d; cpBox = v; cpCyl = cp; }
  });
  cylSideCandidates.forEach(v => {
    const cp = closestPointOnBox(box, v);
    const d = vecNorm(vecSubtract(v, cp));
    if (d < minDist) { minDist = d; cpBox = cp; cpCyl = v; }
  });
  return { minDist, cpBox, cpCyl };
}


function detectCollisionBoxCylinder(box, cyl) {
  const { minDist, cpBox, cpCyl } = distanceBoxCylinder(box, cyl);
  const collision = (minDist <= 1e-6);
  let contactPoint = [];
  if (collision) {
    contactPoint = vecScale(vecAdd(cpBox, cpCyl), 0.5);
  }
  return { collision, contactPoint };
}

function detectCollision(box, shape2) {
  if (shape2.type === 'box') {
    const { collision, mtv } = SATCollision(box, shape2);
    if (collision) {
      const contactPoint = computeContactPointBoxBox(box, shape2, mtv);
      return { collision: true, contactPoint };
    } else {
      return { collision: false, contactPoint: null };
    }
  } else if (shape2.type === 'sphere') {
    const cp_box = closestPointOnBox(box, shape2.center);
    const d = vecNorm(vecSubtract(shape2.center, cp_box));
    const collision = (d <= shape2.extents[0]);
    let contactPoint = [];
    if (collision) {
      const dir = vecNormalize(vecSubtract(shape2.center, cp_box));
      contactPoint = vecSubtract(shape2.center, vecScale(dir, shape2.extents[0]));
    }
    return { collision, contactPoint };
  } else if (shape2.type === 'cylinder') {
    return detectCollisionBoxCylinder(box, shape2);
  } else {
    throw new Error('Collision detection for shape type not supported: ' + shape2.type);
  }
}


function computeDistance(box, shape2) {
  if (shape2.type === 'box') {
    return analyticalDistanceBoxBox(box, shape2);
  } else if (shape2.type === 'sphere') {
    const { minDist, cpBox, cpSphere } = distanceBoxSphere(box, shape2);
    return { minDist, cp1: cpBox, cp2: cpSphere };
  } else if (shape2.type === 'cylinder') {
    const { minDist, cpBox, cpCyl } = distanceBoxCylinder(box, shape2);
    return { minDist, cp1: cpBox, cp2: cpCyl };
  } else {
    throw new Error('Distance computation for shape type not supported: ' + shape2.type);
  }
}


// ----- Main Function: ComputeCollisionsForPendulumLoad -----
// This function builds dynamic load states from the pendulum data,
// loads stationary objects, performs collision detection for each time step,
// and returns the collision results.
export function ComputeCollisionsForPendulumLoad(pendulumPointsAndRotations) {
  // Load the dynamic load
  const baseLoad = DefineLoad();
  const nSteps = pendulumPointsAndRotations.t.length;
  
  // Build dynamic load states by updating center, rotation, and time.
  // MATLAB computes the global center by adding the pivot to each coordinate.
  const loadStates = [];
  for (let i = 0; i < nSteps; i++) {
    const globalCenter = [
      pendulumPointsAndRotations.xGlobal[i],   // Three.js X
      pendulumPointsAndRotations.zGlobal[i],   // Three.js Y (up)
      -pendulumPointsAndRotations.yGlobal[i]   // Three.js Z (forward)
    ];
    // Deep clone baseLoad (using JSON clone)
    const loadState = JSON.parse(JSON.stringify(baseLoad));
    loadState.center = globalCenter;
    // Assume R is an array of 3x3 matrices (one per time step)
    loadState.rotation = pendulumPointsAndRotations.R[i];
    loadState.time = pendulumPointsAndRotations.t[i];
    loadStates.push(loadState);
  }
  
  // Load stationary objects.
  const objectsArray = ListOfObjects();
  const stationaryObjects = [];
  for (let i = 0; i < objectsArray.length; i++) {
   let obj = objectsArray[i];
   obj = ensureRotationMatrix(obj);
  // Convert object center from MATLAB coordinates to Three.js coordinates:
  // MATLAB: [X, Y, Z]  → Three.js: [Y, Z, X]
   obj.center = [
     obj.center[0], // MATLAB Y becomes Three.js x
     obj.center[2], // MATLAB Z becomes Three.js y
     obj.center[1]  // MATLAB X becomes Three.js z
  ];
  stationaryObjects.push(obj);
}
  
  // Collision Detection Loop: For each time step and each stationary object,
  // determine if a collision occurs or compute the minimum distance and witness points.
  const allResults = [];
  for (let tIdx = 0; tIdx < nSteps; tIdx++) {
    const currentLoad = loadStates[tIdx];
    const timeResults = [];
    for (let i = 0; i < stationaryObjects.length; i++) {
      const obj = stationaryObjects[i];
      const colResult = detectCollision(currentLoad, obj);
      if (colResult.collision) {
        const result = {
          objectName: obj.name,
          collision: true,
          collisionPoint: colResult.contactPoint,
          minDistance: 0,
          witnessPoints: []
        };
        timeResults.push(result);
      } else {
        const { minDist, cp1, cp2 } = computeDistance(currentLoad, obj);
        const result = {
          objectName: obj.name,
          collision: false,
          collisionPoint: [],
          minDistance: minDist,
          witnessPoints: [cp1, cp2]
        };
        timeResults.push(result);
      }
    }
    allResults.push({
      time: currentLoad.time,
      collisions: timeResults
    });
  }
  
  return allResults;
}
