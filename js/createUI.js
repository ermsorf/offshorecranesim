import GUI from "../libs/js/lil-gui.module.min.js"; 
import { downloadCSV, loadCSVFile } from './csv.js';
import { startSimulation, stopSimulation, singleStep, setRunning, setStep, setPlaybackIndex } from './main_crane.js';
import { controlForces } from "./main_crane.js"; 

// Create UI buttons
export let playbackMode = false;

export function createUI() {
  const gui = new GUI();
  gui.domElement.style.position = "absolute";
  gui.domElement.style.top = "0px";
  gui.domElement.style.right = "0px";
  gui.domElement.style.height = '100vh';
  gui.domElement.style.overflowY = 'auto';

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
      const joystickMagnitude = data.force;
      controlForces.BoomRotationZ = -Math.cos(angle) * joystickMagnitude * 100000;
      controlForces.TrolleyTranslationX = Math.sin(angle) * joystickMagnitude * 1000;
    }
  });

  joystick.on("end", () => {
    controlForces.BoomRotationZ = 0;
    controlForces.TrolleyTranslationX = 0;
  });

  // Playback Controls
  const playbackFolder = gui.addFolder("Playback Controls");
  const playbackActions = {
    Start: () => {
      playbackMode = true;
      setStep(0);
      setPlaybackIndex(0);
      setRunning(true);
    },
    Stop: () => {
      playbackMode = false;
      setRunning(false);
    }
  };
  playbackFolder.add(playbackActions, "Start");
  playbackFolder.add(playbackActions, "Stop");

  // CSV Upload Input
  const csvLabel = document.createElement("label");
  csvLabel.textContent = "Load CSV:";
  csvLabel.style.display = "block";
  csvLabel.style.marginTop = "8px";
  csvLabel.style.fontSize = "14px";
  csvLabel.style.color = "#fff";

  const csvInput = document.createElement("input");
  csvInput.type = "file";
  csvInput.accept = ".csv";
  csvInput.style.display = "block";
  csvInput.style.margin = "4px 0 10px";

  csvInput.addEventListener("change", async (e) => {
    const file = e.target.files[0];
    if (file) {
      await loadCSVFile(file);  // Make sure loadCSVFile() is defined globally
      setTimeout(() => {
        playbackMode = true;
        setStep(0);
        setPlaybackIndex(0);
        startSimulation();
      }, 1000);
      
    }
  });

  playbackFolder.domElement.appendChild(csvLabel);
  playbackFolder.domElement.appendChild(csvInput);
}