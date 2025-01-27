

let theta = []; let thetadot = []; let thetaddot = [];
for (let i = 1; i <= 5; i++) {
    theta.push(nerdamer(`theta${i}`));
    thetadot.push(nerdamer(`thetadot${i}`));
    thetaddot.push(nerdamer(`thetaddot${i}`));      
}

let L = [];
for (let j = 1; j <= 5; j++) {
    L.push(nerdamer(`L${j}`));    
}

let frames = [];
for (let n = 1; n<=3; n++) {
    frames[n] = new Frame();
    frames[n].setProperties({framenumber: Number(n)})
}

frames[1].setProperties({
    rotationaxis: 3, 
    rotationvar: theta[1],
    cm2joint: [0,0,0],
    joint2cm: [length[1]/2,0,0],
    Qcoordinates: [theta[1],thetadot[1],thetaddot[1]]
})


