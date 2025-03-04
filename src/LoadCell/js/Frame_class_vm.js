
theta = 2
class Frame {
  constructor(properties = {}) {
    this.framenumber = 2;
    this.rotationaxis = 3;
    this.rotationvar = theta;
    this.cm2joint = [0, 0, 0];
    this.joint2cm = [0, 0, 0];
    this.Qcoordinates = null;
    this.Ematrix = null;
    this.Edotmatrix = null;
    this.Omatrix = null;
    this.Bmatrix = null;
    this.mass = null;
    this.Jmatrix = null;
    this.Mmatrix = null;
    this.Dmatrix = null;

    this.setProperties(properties);
  }

  setProperties(properties) {
    Object.keys(properties).forEach((key) => {
      if (Object.prototype.hasOwnProperty.call(this, key)) {
        this[key] = properties[key];
      } else {
        throw new Error(`Unknown property: ${key}`);
      }
    });
  }

  displayInfo() {
    console.log(`Frame ${this.framenumber}`);
    console.log(`Rotation Axis: ${this.rotationaxis}`);
    console.log(`Mass: ${this.mass}`);
    console.log(`Q Coordinates: ${JSON.stringify(this.Qcoordinates)}`);
  }

  static skew(vector) {
    if (vector.length !== 3) {
      throw new Error("Input vector must be 3D.");
    }
    return [
      [0, -vector[2], vector[1]],
      [vector[2], 0, -vector[0]],
      [-vector[1], vector[0], 0],
    ];
  }

  static unskew(matrix) {
    return [matrix[2][1], matrix[0][2], matrix[1][0]];
  }

  makeEr() {
    const axis = this.rotationaxis; // Rotation axes
    const theta = this.rotationvar; // Rotation variables

    let Er = math.identity(4).toArray(); // Start with identity matrix (4x4)

    axis.forEach((axisDirection, i) => {
        const theta_i = theta[i];
        const c = math.cos(theta_i); // Cosine of the angle
        const s = math.sin(theta_i); // Sine of the angle

        let rotationMatrix = math.identity(4).toArray(); // Create a 4x4 identity matrix
        switch (axisDirection) {
            case 1: // Rotation about x-axis
                rotationMatrix[1][1] = c;
                rotationMatrix[1][2] = -s;
                rotationMatrix[2][1] = s;
                rotationMatrix[2][2] = c;
                break;
            case 2: // Rotation about y-axis
                rotationMatrix[0][0] = c;
                rotationMatrix[0][2] = s;
                rotationMatrix[2][0] = -s;
                rotationMatrix[2][2] = c;
                break;
            case 3: // Rotation about z-axis
                rotationMatrix[0][0] = c;
                rotationMatrix[0][1] = -s;
                rotationMatrix[1][0] = s;
                rotationMatrix[1][1] = c;
                break;
            default:
                throw new Error("Invalid rotation axis: must be 1, 2, or 3.");
        }

        // Multiply the current Er with the new rotation matrix
        Er = math.multiply(Er, rotationMatrix); // math.js handles 4x4 matrix multiplication
    });
  
    this.Ematrix = Er; // Store the result
    return Er;
  }
  makeEv() {
    const Ev = math.identity(4).toArray();
    Ev[0][3] = this.joint2cm[0];
    Ev[1][3] = this.joint2cm[1];
    Ev[2][3] = this.joint2cm[2];

    console.log("Ev Matrix:", Ev);
    return Ev;
  }

  makeE(framelist) {
    let E = math.identity(4).toArray();
    for (let i = 0; i < this.framenumber; i++) {
      const frame = framelist[i];
      E = math
        .multiply(
          E,
          frame.makeEv(),
          frame.makeEr(),
          frame.makeEv()
          );
       
      frame.Ematrix = E;
    }
    this.Ematrix = E;

    console.log("E Matrix:", E);
    return this.Ematrix;
  }

  makeEdot(framelist) {
    const E = this.Ematrix || this.makeE(framelist);
    const thetaDot = this.Qcoordinates.map((q) => q[1]);
    const Edot = math.derivative(E, thetaDot);

    console.log("Edot Matrix:", Edot);
    this.Edotmatrix = Edot;
    return this.Edotmatrix;
  }

  makeO(framelist) {
    const E = this.Ematrix || this.makeE(framelist);
    const Edot = this.Edotmatrix || this.makeEdot(framelist);
    const O = math.multiply(math.inv(E), Edot).toArray();

    console.log("O Matrix:", O);
    this.Omatrix = O;
    return this.Omatrix;
  }

  makeB(framelist) {
    const Q = framelist.flatMap((frame) => frame.Qcoordinates || []);
    let B = math.zeros(this.framenumber * 6, Q.length).toArray();

    framelist.forEach((frame, i) => {
      const Edot = frame.Edotmatrix || frame.makeEdot(framelist);
      const O = frame.Omatrix || frame.makeO(framelist);

      const posvec = Edot.slice(0, 3).map((row) => row[3]);
      const Ovec = Frame.unskew(O.slice(0, 3).map((row) => row.slice(0, 3)));

      Q.forEach((q, qIndex) => {
        for (let direction = 0; direction < 3; direction++) {
          B[6 * i + direction][qIndex] = posvec[direction] || 0;
          B[6 * i + 3 + direction][qIndex] = Ovec[direction] || 0;
        }
      });
    });

    console.log("B Matrix:", B);
    this.Bmatrix = B;
    return this.Bmatrix;
  }
}

// Example usage
const frame1 = new Frame({
  framenumber: 1,
  rotationaxis: [3],
  rotationvar: [math.pi / 4],
  joint2cm: [1, 0, 0],
  Qcoordinates: [["theta1", "thetadot1", "thetaddot1"]],
});

const frame2 = new Frame({
  framenumber: 2,
  rotationaxis: [1],
  rotationvar: [math.pi / 6],
  joint2cm: [0, 1, 0],
  Qcoordinates: [["theta2", "thetadot2", "thetaddot2"]],
});

const frames = [frame1, frame2];
frame1.makeE(frames);
frame1.makeEdot(frames);
frame1.makeO(frames);
frame1.makeB(frames);
