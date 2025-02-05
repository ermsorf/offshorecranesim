// Load the system config JSON file
fetch('../src/matlabprototyping/config.json')
  .then(response => response.json())
  .then(system => {
      const h = 0.01; // Time step
      let state = { x: 1.5, y: 2.0, z: Math.PI / 4 }; // Initial conditions

      function evaluateExpression(expr, vars) {
          let result = expr;
          vars.forEach(varName => {
              result = result.replace(new RegExp(`\\b${varName}\\b`, 'g'), state[varName]);
          });
          return eval(result);
      }

      function evaluateMatrix(matrix) {
          return matrix.map(row =>
              row.map(entry => evaluateExpression(entry.expr, entry.vars))
          );
      }

      // Process all matrices dynamically
      let evaluatedMatrices = {};
      Object.keys(system).forEach(key => {
          if (key !== "variables") {
              evaluatedMatrices[key] = evaluateMatrix(system[key]);
          }
      });

      console.log(evaluatedMatrices); // Matrices with numerical values
  });

