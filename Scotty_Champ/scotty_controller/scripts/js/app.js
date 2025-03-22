// Main application entry point
import { config, parseUrlParams } from './config.js';
import { setupScene, renderer, scene, camera, controls } from './scene.js';
import { loadRobotFromURDF } from './robotLoader.js';
import { setupJointStateSubscriber } from './rosConnection.js';

// DOM elements
const canvasContainer = document.getElementById('canvas-container');
const loadingElement = document.getElementById('loading');
const statusElement = document.getElementById('status');

// Global variables
let robot = null;
let jointStateSubscriber = null;

/**
 * Initialize the visualization
 * This is called on page load
 */
async function init() {
    try {
        // Parse URL parameters
        parseUrlParams();
        
        // Set up Three.js scene
        setupScene(canvasContainer);
        
        // Adjust renderer to match container size
        updateRendererSize();
        
        // Show loading indicator
        loadingElement.style.display = 'flex';
        
        // Wait for MODE data before loading robot
        if (!window.modeReceived) {
            statusElement.textContent = 'Waiting for robot mode...';
            window.addEventListener('modeReceived', loadRobotModel);
        } else {
            // MODE already received, load right away
            loadRobotModel();
        }
        
        // Handle window resize
        window.addEventListener('resize', updateRendererSize);
        
    } catch (error) {
        console.error('Initialization error:', error);
        statusElement.textContent = 'Error: ' + error.message;
        loadingElement.style.display = 'none';
    }
}

/**
 * Load robot model after receiving MODE
 * This is called once the mode is received
 */
async function loadRobotModel() {
    try {
        statusElement.textContent = 'Loading robot model...';
        
        // Load robot model from URDF
        robot = await loadRobotFromURDF(config.urdf_path);
        
        // Add robot to scene
        if (robot) {
            scene.add(robot.object);
            
            // Set up joint state subscriber
            jointStateSubscriber = setupJointStateSubscriber(robot);
            
            // Center the robot
            centerAndScaleRobot(robot);
            
            // Update status
            statusElement.textContent = 'Robot model loaded successfully';
            
            // Log success message in console
            if (window.addToConsoleLog) {
                window.addToConsoleLog('SUCCESS : Robot model loaded successfully');
            }
            
            // Hide loading indicator
            loadingElement.style.display = 'none';
        }
    } catch (error) {
        console.error('Error loading robot model:', error);
        statusElement.textContent = 'Error loading model: ' + error.message;
        loadingElement.style.display = 'none';
    }
}

/**
 * Center and scale the robot
 */
function centerAndScaleRobot(robot) {
    if (!robot) return;
    
    // Reset robot position and rotation
    robot.object.position.set(0, 0, 0);
    robot.object.rotation.set(-Math.PI/2, 0, Math.PI);
    
    // Update camera and controls
    camera.position.set(0, 0.5, 2.5);
    controls.target.set(0, 0.3, 0);
    controls.update();
}

/**
 * Update renderer size to match container
 */
function updateRendererSize() {
    if (canvasContainer && renderer) {
        const width = canvasContainer.clientWidth;
        const height = canvasContainer.clientHeight;
        
        renderer.setSize(width, height);
        camera.aspect = width / height;
        camera.updateProjectionMatrix();
    }
}

// Animation loop
function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}

// Start initialization and animation
init();
animate();

// Show/hide the help panel
function setupHelpPanel() {
    const helpToggle = document.getElementById('help-toggle');
    const helpPanel = document.getElementById('help-panel');
    
    if (helpToggle && helpPanel) {
        helpToggle.addEventListener('click', () => {
            if (helpPanel.style.display === 'none' || helpPanel.style.display === '') {
                helpPanel.style.display = 'block';
                helpToggle.textContent = 'Hide Help';
            } else {
                helpPanel.style.display = 'none';
                helpToggle.textContent = 'Show Help';
            }
        });
    }
}

// Start the application when the page is loaded
window.onload = init; 