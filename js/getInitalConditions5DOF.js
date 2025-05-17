import * as THREE from '../libs/three/three.module.js';
// import { sceneObjects } from '../libs/three/loaders/modelLoader.js';

export function getInitalConditions5DOF(rawDt, lastState = {}, sceneObjects) {
  const dt = Math.max(1e-6, Math.min(rawDt, 0.1));

  const trolleyObj   = sceneObjects.Theta1;
  const containerObj = sceneObjects.Container;

  if (!trolleyObj || !containerObj) {
    console.warn('getInitalConditions5DOF: waiting for sceneObjects to load');
    return {
      q: [0, 0, 0, 0, 0],
      q_dot: [0, 0, 0, 0, 0],
      lastState,
      wireLength: 0,
    };
  }

  const trolleyPos = new THREE.Vector3();
  const containerPos = new THREE.Vector3();
  trolleyObj.getWorldPosition(trolleyPos);
  containerObj.getWorldPosition(containerPos);

  const x_t = trolleyPos.x;
  const y_t = trolleyPos.y;
  const z_t = trolleyPos.z;

  const x_c = containerPos.x;
  const y_c = containerPos.y;
  const z_c = containerPos.z;

  // Generalized coordinates
  let theta1 = Math.atan2(z_t,x_t); // boom yaw
  let lambda = Math.hypot(x_t, z_t); // radial distance
  const dx = x_c - x_t;
  const dy = y_c - y_t;
  const dz = z_c - z_t;





// Rotate (dx, dz) into the local frame of the trolley (rotate by -theta1)
const cos1 = Math.cos(theta1);
const sin1 = Math.sin(theta1);

const local_dx = cos1 * dx - sin1 * dz;
const local_dz = sin1 * dx + cos1 * dz;

let phi = Math.atan2(local_dx, local_dz); // theta2 in trolley's rotated frame





  let theta = Math.atan2(Math.hypot(dx, dz), -dy); // flipped polar
  let theta4 = 0; // container yaw = constant for now

  // Unwrap boom yaw to grow positively from 0 to ∞
  let continuousYaw = lastState.continuousYaw ?? ((theta1 >= 0) ? theta1 : theta1 + 2 * Math.PI);
  if (lastState.q) {
    const lastRawYaw = lastState.q[0];
    const delta = wrapAngle(theta1 - lastRawYaw);
    continuousYaw = lastState.continuousYaw + delta;
    if (continuousYaw < 0) continuousYaw += 2 * Math.PI;
    theta1 = continuousYaw;
  } else {
    theta1 = continuousYaw;
  }

  const q = [theta1, lambda, phi, theta, theta4];

  // Finite-difference derivative
  let q_dot = [0, 0, 0, 0, 0];
  if (lastState.q) {
    const Δq = [
      wrapAngle(theta1 - lastState.q[0]),
      lambda - lastState.q[1],
      wrapAngle(phi - lastState.q[2]),
      wrapAngle(theta - lastState.q[3]),
      0
    ];
    q_dot = Δq.map(Δ => Δ / (dt));
  }

  const wireLength = Math.hypot(dx, dy, dz);

  // console.log('q', q);
  // console.log('q_dot', q_dot);
  return {
    q,
    q_dot,
    lastState: { q, trolleyPos, containerPos, continuousYaw },
    wireLength,
  };
}

function wrapAngle(angle) {
  return ((angle + Math.PI) % (2 * Math.PI)) - Math.PI;
}
