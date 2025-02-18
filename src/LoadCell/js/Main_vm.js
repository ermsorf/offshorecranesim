const math = require('mathjs');
const Frame = require('./Frame'); // Ensure to import the Frame class

function main() {
    const theta = math.symbols(['theta1', 'theta2', 'theta3', 'theta4']);
    const thetadot = math.symbols(['thetadot1', 'thetadot2', 'thetadot3', 'thetadot4']);
    const thetaddot = math.symbols(['thetaddot1', 'thetaddot2', 'thetaddot3', 'thetaddot4']);
    const l1 = math.symbol('l1');
    const l2 = math.symbol('l2');

    const frames = [];

    for (let i = 0; i < 4; i++) {
        frames.push(new Frame({
            framenumber: i + 1,
            rotationaxis: [3],
            rotationvar: theta[i],
            mass: 2,
            Qcoordinates: [theta[i], thetadot[i], thetaddot[i]],
            cm2joint: [l1, 0, 0],
            joint2cm: [0, l2, 0]
        }));
    }

    ReactionForces(frames);
}

main();