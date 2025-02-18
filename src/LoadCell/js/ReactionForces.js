
function ReactionForces(frames) {
    try {
        // Ensure frames array is properly initialized
        if (!Array.isArray(frames) || frames.length < 3) {
            throw new Error("frames array is not properly initialized. It must contain at least three elements.");
        }

        // Define constants
        let g = 9.81; // Gravity

        // Mass properties from frames
        let m2 = frames[1].mass || 2;  
        let m3 = frames[2].mass || 5;  

        console.log("Masses:", { m2, m3 });

        // Get rotation matrices (Ensure they are not undefined)
        let R1 = frames[0].makeEr()?.slice(0, 3).map(row => row.slice(0, 3)) || math.identity(3)._data;
        let R21 = frames[1].makeEr()?.slice(0, 3).map(row => row.slice(0, 3)) || math.identity(3)._data;
        let R32 = frames[2].makeEr()?.slice(0, 3).map(row => row.slice(0, 3)) || math.identity(3)._data;

        console.log("Rotation Matrices:", { R1, R21, R32 });

        // Compute B-matrix (Ensure it's properly initialized)
        let B = frames[1].makeBdot(frames) || math.zeros(3, 2)._data;

        console.log("B Matrix:", B);

        let Qdd1 = frames[0].Qcoordinates?.[2] || 0;  
        let Qdd2 = frames[1].Qcoordinates?.[2] || 0;

        let QddMatrix = [[Qdd1], [Qdd2]]; // Ensure it's a column vector

        console.log("Qdd Matrix:", QddMatrix);

        // Compute accelerations (Xddots)
        let Xddots = math.multiply(B, QddMatrix);

        console.log("Xddots:", Xddots);

        //  Extracts components for Xddots
        let Xddot12 = Xddots?.[0]?.[0] || 0;
        let Xddot22 = Xddots?.[1]?.[0] || 0;
        let Xddot32 = Xddots?.[2]?.[0] || 0;

        // Compute reaction force equations
        let Equation = math.add(
            [0, 0, -m2 * g],
            math.multiply(R1, R21, [0, 0, -m3 * g]))
        ;

        console.log("Equation:", Equation);

        // Solve equations using math.js
        let coefficientMatrix = math.multiply(R1, R21);
        let forceVector = [Xddot12, Xddot22, Xddot32];

        // Ensure matrix is square
        if (coefficientMatrix.length !== forceVector.length) {
            throw new Error("Coefficient matrix and force vector dimensions mismatch.");
        }

        let solutions = math.lusolve(coefficientMatrix, forceVector);

        console.log("Reaction Forces:", solutions);
        return solutions;
    } catch (error) {
        console.error("Error in ReactionForces:", error.message);
    }
}

// Sample frames initialization (Fixing Frame constructor)
function createFrames() {
    let frames = [];
    for (let i = 0; i < 4; i++) {
        frames.push(new Frame(i + 1, 3, 1.5, 2, [1, 2, 3], [1, 0, 0], [0, 1, 0])); 
    }
    return frames;
}

window.calculateReactionForces = function() {
    console.log("Calculating reaction forces...");
    // Your calculation logic
};
// Attach function to button
function calculateReactionForces() {
    let frames = createFrames();
    let result = ReactionForces(frames);
    alert("Reaction Forces Computed: " + JSON.stringify(result));
}