// collisionWorker.js
import { runRK4Step } from './RK4.js';
import { ComputeCollisionsForPendulumLoad } from './collisionDetection.js';

const craneHeight = 60; // your fixed jib‐boom height

// ─── Math‐helpers ──────────────────────────────────────────────────────────
function rotationZ(θ) {
  const c = Math.cos(θ), s = Math.sin(θ);
  return [
    [ c, -s, 0 ],
    [ s,  c, 0 ],
    [ 0,  0, 1 ]
  ];
}

function rotationY(θ) {
  const c = Math.cos(θ), s = Math.sin(θ);
  return [
    [  c, 0, s ],
    [   0, 1, 0 ],
    [ -s, 0, c ]
  ];
}

function multiplyMatrices(A, B) {
  const R = [];
  for (let i = 0; i < 3; i++) {
    R[i] = [];
    for (let j = 0; j < 3; j++) {
      let sum = 0;
      for (let k = 0; k < 3; k++) {
        sum += A[i][k] * B[k][j];
      }
      R[i][j] = sum;
    }
  }
  return R;
}

// ─── Single‐slot buffer for the latest frame data ─────────────────────────
let pendingData = null;

// Overwrite pendingData on every message; never queue up old ones
onmessage = (e) => {
  pendingData = e.data;
};

// ─── Main worker loop: pick up the latest data, run simulation, post back ─
;(async function mainLoop() {
  while (true) {
    // Wait until we have new data
    if (!pendingData) {
      await new Promise(r => setTimeout(r, 1));
      continue;
    }

    // Grab and clear the slot so new messages can come in
    const data = pendingData;
    pendingData = null;

    // Unpack exactly what main_crane.js is sending:
    // { type:'step', dt, state:{q,q_dot}, wireLength, system, variableMap }
    const {
      dt,
      state:       { q, q_dot },
      wireLength,
      system,
      variableMap,
      trigMap,
    } = data;

    // Build the 5×3 state matrix [position, velocity, unused]
    //  [ q[0], q_dot[0], 0 ],
    //  [ q[1], q_dot[1], 0 ],
    //  [ q[2], q_dot[2], 0 ],
    //  [ q[3], q_dot[3], 0 ],
    //  [ q[4], q_dot[4], 0 ]
    //];



    console.log('Before Q5DOF creation:', q);

    let  PreQ5DOF = new Array (
      [ q[0], q_dot[0], 0 ],
      [ q[1], q_dot[1], 0 ],
      [ q[2], q_dot[2], 0 ],
      [ q[3], q_dot[3], 0 ],
      [ q[4], q_dot[4], 0 ]
  );

// Check q

console.log('q(0)',q[0])

const Q5DOF = JSON.parse(JSON.stringify(PreQ5DOF));
//console.log('PreQ5DOF',PreQ5DOF)
//console.log('Q5DOF', Q5DOF)


    // Prepare integration constants
    const controlForces = { BoomRotationZ: 0, TrolleyTranslationX: 0 };
    const constants     = { g: 9.81, l: wireLength };

    // Run up to 500 RK4 steps, but abort early if new data arrives
    const coordsHistory = [];
    const totalSteps    = 500;
    for (let step = 0; step < totalSteps; step++) {
      if (pendingData) break;  // newer frame waiting—stop this run
      await runRK4Step(system, Q5DOF, variableMap, trigMap, dt, controlForces, constants);
      coordsHistory.push(Q5DOF.map(row => row[0]));
      //console.log(Q5DOF)
      //console.log(dt)
      //console.log(JSON.stringify(Q5DOF));
    }

    // Downsample the trajectory to n=20
    const total = coordsHistory.length;
    const n     = 20;
    const sampled = Array.from({ length: n }, (_, i) => {
      const idx = Math.floor(i * total / n);
      return coordsHistory[idx] || coordsHistory[total - 1];
    });

    //console.log(sampled)

    // Rebuild global XYZ + rotation matrices for each sample
    const xGlobal = [], yGlobal = [], zGlobal = [], Rfull = [], t = [];
    sampled.forEach((s, i) => {
      const [θ1, R, θ2, θ3] = s;
      const L = wireLength;

      // Cartesian mapping
      //console.log('R',R);
      console.log(θ1);
      const x = R * Math.cos(θ1)
              - L * Math.sin(θ3) * (Math.cos(θ1)*Math.sin(θ2) + Math.cos(θ2)*Math.sin(θ1));
      const y = R * Math.sin(θ1)
              + L * Math.sin(θ3) * (Math.cos(θ1)*Math.cos(θ2) - Math.sin(θ1)*Math.sin(θ2));
      const z = -L * Math.cos(θ3)  + craneHeight;

      xGlobal.push(x);
      yGlobal.push(y);
      zGlobal.push(z);

      let M = rotationZ(θ1);
      M = multiplyMatrices(M, rotationZ(θ2));
      M = multiplyMatrices(M, rotationY(θ3));
      Rfull.push(M);

      t.push((i + 1) * dt * (total / n));
    });

    // Compute collisions and send the result back
    const simulationResult = { xGlobal, yGlobal, zGlobal, R: Rfull, t };
    const collisionResults = ComputeCollisionsForPendulumLoad(simulationResult);
    postMessage({ collisionResults, simulationResult });
  }
})();
