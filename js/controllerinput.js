// Handle gamepad input
function updateGamepadStatus() {
    const gamepads = navigator.getGamepads();
    let statusText = '';
    
    // Loop through all connected gamepads
    for (let i = 0; i < gamepads.length; i++) {
        const gamepad = gamepads[i];
        if (gamepad) {
            // Log the buttons pressed
            statusText += `Gamepad ${i} buttons:\n`;
            gamepad.buttons.forEach((button, index) => {
                statusText += `  Button ${index}: ${button.pressed ? 'Pressed' : 'Not Pressed'}\n`;
            });

            // Log the axes values
            statusText += `Gamepad ${i} axes:\n`;
            gamepad.axes.forEach((axis, index) => {
                statusText += `  Axis ${index}: ${axis.toFixed(2)}\n`;
            });

            statusText += '\n'; // Add space between gamepads if multiple are connected
        }
    }

    // Update the controls div with the latest gamepad status
    document.getElementById('gamepadStatus').textContent = statusText;

    // Repeat the check
    requestAnimationFrame(updateGamepadStatus);
}

// Start listening for gamepad input
window.addEventListener('gamepadconnected', (event) => {
    console.log('Gamepad connected:', event.gamepad);
    updateGamepadStatus();
});

window.addEventListener('gamepaddisconnected', (event) => {
    console.log('Gamepad disconnected:', event.gamepad);
});