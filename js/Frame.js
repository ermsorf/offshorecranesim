var nerdamer = require('../libs/math/all.js');

class Frame {
  constructor(options = {}) {
    this.framenumber = options.framenumber || 0;
    this.rotationaxis = options.rotationaxis || 0;
    this.rotationvar = options.rotationvar || 0;

    this.cm2joint = options.cm2joint || [0, 0, 0];
    this.joint2cm = options.joint2cm || [0, 0, 0];

    this.Qcoordinates = options.Qcoordinates || [];

    this.Ematrix = [];
    this.Edotmatrix = [];
    this.Omatrix = [];
    this.Bmatrix = [];

    this.mass = options.mass || 0;
    this.Jmatrix = options.Jmatrix || nerdamer('eye(3)');
    this.Mmatrix = [];
    this.Dmatrix = [];
  }

  setProperties(properties) {
    for (const [key, value] of Object.entries(properties)) {
      if (this.hasOwnProperty(key)) {
        this[key] = value;
      } else {
        throw new Error(`Unknown property: ${key}`);
      }
    }
  }

  makeEr() {
    const axis = this.rotationaxis;
    const theta = this.rotationvar;

    if (![1, 2, 3].includes(axis)) {
      throw new Error('Axis must be 1 (x-axis), 2 (y-axis), or 3 (z-axis).');
    }

    const c = nerdamer(`cos(${theta})`);
    const s = nerdamer(`sin(${theta})`);
    let Er = nerdamer.imatrix();

    switch (axis) {
      case 1:
        Er = nerdamer.matset(1, 1, c);
        Er = Er.matset(1, 2, -s);
        Er = Er.matset(2, 1, s);
        Er = Er.matset(2, 2, c);
        break;
      case 2:
        Er = Er.matset(0, 0, c);
        Er = Er.matset(0, 2, s);
        Er = Er.matset(2, 0, -s);
        Er = Er.matset(2, 2, c);
        break;
      case 3:
        Er = Er.matset(0, 0, c);
        Er = Er.matset(0, 1, -s);
        Er = Er.matset(1, 0, s);
        Er = Er.matset(1, 1, c);
        break;
      default:
        break;
    }

    return Er;
  }

}

let theta = []; let thetadot = []; let thetaddot = [];
for (let i = 1; i <= 5; i++) {
  theta.push(nerdamer(`theta${i}`));
  thetadot.push(nerdamer(`thetadot${i}`));
  thetaddot.push(nerdamer(`thetaddot${i}`));
}


let L = [];
for (let j = 0; j <= 5; j++) {
  L.push(nerdamer(`L${j}`));
}

let frames = [];
for (let n = 0; n<=3; n++) {
  frames[n] = new Frame();
  frames[n].setProperties({framenumber: Number(n)})
}

frames[0].setProperties({
  rotationaxis: 3,
  rotationvar: theta[1],
  cm2joint: [0,0,0],
  joint2cm: [L[1]/2,0,0],
  Qcoordinates: [theta[1],thetadot[1],thetaddot[1]]
})


console.log(frames[0].makeEr(frames).text())
