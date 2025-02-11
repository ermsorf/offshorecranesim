fetch('../src/FrameGen/testconfig.json')
  .then(response => response.json())
  .then(system => {
    console.log("Loaded system config:", system);

    let state = {};
    // Populate state object
    for (let i = 0; i < system.Qcoordinates.length; i++) {
      for (let j = 0; j < system.Qcoordinates[i].length; j++) {
        let key = system.Qcoordinates[i][j].expr; // Get variable name
        let value = evaluateExpression(system.initconditions[i][j]); // Evaluate initial condition
        state[key] = value; // Store in state
      }
    }

    console.log("Initial Conditions:",state);

    function evaluateExpression(entry) {
      if (!entry || typeof entry.expr !== "string") {
        console.warn("Invalid entry:", entry);
        return NaN;
      }

      let expr = entry.expr;
      let vars = Array.isArray(entry.vars) ? entry.vars : (typeof entry.vars === "string" && entry.vars.trim() !== "") ? [entry.vars] : [];

      if (vars.length === 0) {
        try {
          return new Function(`return ${expr};`)();
        } catch (e) {
          console.error("Eval failed for constant expression:", expr, "Error:", e);
          return NaN;
        }
      }

      let missingVars = vars.filter(v => !(v in state));
      if (missingVars.length > 0) {
        console.error("Missing variables in state:", missingVars);
        return NaN;
      }

      try {
        return new Function(...vars, `return ${expr};`)(...vars.map(v => state[v]));
      } catch (e) {
        console.error("Eval failed for:", expr, "Error:", e);
        return NaN;
      }
    }

    function evaluateMatrix(matrix) {
      if (!Array.isArray(matrix)) return evaluateExpression(matrix);
      if (!Array.isArray(matrix[0])) return matrix.map(entry => evaluateExpression(entry));

      return matrix.map(row => row.map(entry => evaluateExpression(entry)));
    }

    console.time("Matrix Evaluation");
    let evaluatedMatrices = {};
    Object.keys(system).forEach(key => {
      if (key !== "Qcoordinates" && key !== "initconditions") {
        evaluatedMatrices[key] = evaluateMatrix(system[key]);
      }
    });
    console.timeEnd("Matrix Evaluation");

    console.log("Evaluated Matrices:", evaluatedMatrices);
  })
  .catch(error => console.error("Error loading config:", error));
