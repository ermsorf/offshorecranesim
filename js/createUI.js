import GUI from "../libs/js/lil-gui.module.min.js"; 
import { downloadCSV } from './csv.js';
import { startSimulation, stopSimulation, singleStep, controlForces } from './main_crane.js';
// Create UI buttons

export function createUI() {
  const gui = new GUI();
  gui.domElement.style.position = "absolute";
  gui.domElement.style.top = "0px";
  gui.domElement.style.right = "0px";
  gui.domElement.style.height = '100vh'; // Full vertical height
  gui.domElement.style.overflowY = 'auto'; // Enable scrolling if needed

  // Simulation controls
  const simulationFolder = gui.addFolder("RK4 Controls");
  simulationFolder.add({ Start: startSimulation }, "Start");
  simulationFolder.add({ Stop: stopSimulation }, "Stop");
  simulationFolder.add({ Step: singleStep }, "Step");
  simulationFolder.add({ "Download CSV": downloadCSV }, "Download CSV");
  simulationFolder.open();

  // Camera controls
  const cameraFolder = gui.addFolder("Camera");
  const cameraActions = {
    Global: () => console.warn("Global camera switch not implemented"),
    Cabin: () => console.warn("Cabin camera switch not implemented"),
    Trolley: () => console.warn("Trolley camera switch not implemented"),
  };
  cameraFolder.add(cameraActions, "Global");
  cameraFolder.add(cameraActions, "Cabin");
  cameraFolder.add(cameraActions, "Trolley");

  // Joystick controls folder
  const joystickFolder = gui.addFolder("Joystick Controls");
  const joystickContainer = document.createElement("div");
  joystickContainer.style.position = "relative";
  joystickContainer.style.width = "150px";
  joystickContainer.style.height = "150px";
  joystickContainer.style.margin = "10px auto";
  joystickContainer.style.background = "#424242";
  joystickContainer.style.borderRadius = "50%";
  joystickFolder.domElement.appendChild(joystickContainer);

  const joystick = nipplejs.create({
    zone: joystickContainer,
    mode: "static",
    position: { left: "50%", top: "50%" },
    color: "#27D0FF",
  }); 

  joystick.on("move", (evt, data) => {
    if (data && data.force !== undefined && data.angle) {
      const angle = data.angle.radian;
      const joystickMagnitude = data.force; // should be between 0 and 1 but isnt for some reason.
      controlForces.BoomRotationZ = -Math.cos(angle) * joystickMagnitude * 100000; // Change 100000 to adjust force magnitude
      controlForces.TrolleyTranslationX = Math.sin(angle) * joystickMagnitude * 1000; 
    }
  });

  joystick.on("end", () => {
    controlForces.BoomRotationZ = 0;
    controlForces.TrolleyTranslationX = 0;
  });

};