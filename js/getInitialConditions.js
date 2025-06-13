import * as THREE from '../libs/three/three.module.js';

export function getCollisionPoints(
  dt,
  lastContainerPos,      // <-- matches use below
  currentContainerPos,   // <-- matches use below
  lastContainerRot,
  currentContainerRot,
  lastTrolleyPos,
  currentTrolleyPos
) {
    // Compute average trolley position for the current frame.
    const trolleyAvgPos_current = new THREE.Vector3()
        .addVectors(lastTrolleyPos, currentTrolleyPos)
        .multiplyScalar(0.5);
    
    // For the previous frame, we assume that lastTrolleyPos represents the previous position.
    const trolleyAvgPos_last = lastTrolleyPos.clone();

    // Current direction vector from trolley to load:
    const directionVec_current = currentContainerPos.clone().sub(trolleyAvgPos_current);
    const spherical_current   = toSphericalFlipped(directionVec_current);
    
    // Wire length (distance):
    const wireLength = directionVec_current.length();

    // Previous direction vector:
    const directionVec_last = lastContainerPos.clone().sub(trolleyAvgPos_last);
    const spherical_last    = toSphericalFlipped(directionVec_last);
    
    // Angular velocities:
    const thetadot = (spherical_current.theta - spherical_last.theta) / dt;
    const phidot  = (spherical_current.phi   - spherical_last.phi)   / dt;
    
    // Yaw ψ and its rate:
    const psi    = currentContainerRot.x;
    const psidot = (currentContainerRot.x - lastContainerRot.x) / dt;

    // Trolley in cylindrical coords:
    const rTrolley_current = Math.hypot(currentTrolleyPos.x, currentTrolleyPos.z);
    const rTrolley_last    = Math.hypot(lastTrolleyPos.x,    lastTrolleyPos.z);
    const rDotTrolley      = (rTrolley_current - rTrolley_last) / dt;

    const alpha_current = Math.atan2(currentTrolleyPos.x, currentTrolleyPos.z);
    const alpha_last    = Math.atan2(lastTrolleyPos.x,    lastTrolleyPos.z);
    let   alphaDot      = (alpha_current - alpha_last) / dt;

    // Wrap-around correction:
    if (alphaDot >  Math.PI / dt) alphaDot -= 2 * Math.PI / dt;
    if (alphaDot < -Math.PI / dt) alphaDot += 2 * Math.PI / dt;

    // Debug prints (optional)
    //console.log('dt:', dt);
    //console.log('lastContainerPos:', lastContainerPos.toArray());
    //console.log('currentContainerPos:', currentContainerPos.toArray());
    //console.log('lastContainerRot:', lastContainerRot);
    //console.log('currentContainerRot:', currentContainerRot);
    //console.log('lastTrolleyPos:', lastTrolleyPos.toArray());
    //console.log('currentTrolleyPos:', currentTrolleyPos.toArray());
    //console.log({ wireLength, thetadot, phidot, psidot, rDotTrolley, alphaDot });

    return {
        theta:      spherical_current.theta,
        thetadot:   thetadot,
        phi:        spherical_current.phi,
        phidot:     phidot,
        psi:        psi,
        psidot:     psidot,
        pivot:      trolleyAvgPos_current,  // if your solver needs the pivot too
        rTrolley:   rTrolley_current,
        rDotTrolley:rDotTrolley,
        alpha:      alpha_current,
        alphaDot:   alphaDot,
        wireLength: wireLength,
    };
}

function toSphericalFlipped(v) {
    const x = v.x, y = v.y, z = v.z;
    const radius = Math.hypot(x, y, z);
    const theta  = Math.atan2(x, z);                  // around Y
    const phi    = Math.atan2(Math.hypot(x, z), -y);  // from –Y
    return { radius, theta, phi };
}
