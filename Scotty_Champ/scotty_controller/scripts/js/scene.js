// Setup Three.js scene, renderer, camera and controls
// Using globals from script tags included in HTML instead of ES modules

// Export global objects for use in other modules
export let scene, camera, renderer, controls;
export let robotGroup = new THREE.Group();

/**
 * Sets up the Three.js scene with lighting and camera
 * @param {HTMLElement} container - DOM element to append the renderer
 */
export function setupScene(container) {
    // Create scene
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x000000);
    scene.add(robotGroup);
    
    // Create camera
    camera = new THREE.PerspectiveCamera(
        45,                                             // Field of view
        container.clientWidth / container.clientHeight, // Aspect ratio
        0.01,                                           // Near clipping plane
        1000                                            // Far clipping plane
    );
    camera.position.set(0, 0.3, 2.0);
    
    // Create renderer
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.shadowMap.enabled = true;
    
    // Add renderer to container
    container.appendChild(renderer.domElement);
    
    // Create controls
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 0.1, 0);
    controls.enableDamping = true;
    controls.dampingFactor = 0.25;
    controls.screenSpacePanning = false;
    controls.minDistance = 0.3;
    controls.maxDistance = 10;
    controls.maxPolarAngle = Math.PI / 1.5;
    controls.update();
    
    // Add lighting
    setupLighting();
    
    // Add grid for reference
    // addGrid();
    
    // Event listeners
    window.addEventListener('resize', onWindowResize);
    
    // Start animation loop
    animate();
    
    return { scene, camera, renderer, controls };
}

/**
 * Sets up scene lighting
 */
function setupLighting() {
    // Ambient light
    const ambientLight = new THREE.AmbientLight(0x666666);
    scene.add(ambientLight);
    
    // Directional light (sun)
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(1, 1, 1);
    directionalLight.castShadow = true;
    directionalLight.shadow.mapSize.width = 1024;
    directionalLight.shadow.mapSize.height = 1024;
    
    // Adjust shadow camera frustum
    const d = 2;
    directionalLight.shadow.camera.left = -d;
    directionalLight.shadow.camera.right = d;
    directionalLight.shadow.camera.top = d;
    directionalLight.shadow.camera.bottom = -d;
    directionalLight.shadow.camera.near = 0.1;
    directionalLight.shadow.camera.far = 5;
    
    scene.add(directionalLight);
    
    // Add hemisphere light for better ambient lighting
    const hemiLight = new THREE.HemisphereLight(0xffffff, 0x444444, 0.6);
    hemiLight.position.set(0, 10, 0);
    scene.add(hemiLight);
}

/**
 * Adds a grid to the scene for reference
 */
function addGrid() {
    // Add grid helper
    const gridHelper = new THREE.GridHelper(3, 30, 0x555555, 0x333333);
    scene.add(gridHelper);
    
    // Add X, Y, Z axis helpers
    const axesHelper = new THREE.AxesHelper(1);
    scene.add(axesHelper);
}

// Animation loop
function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}

// Resize handler
function onWindowResize() {
    const container = document.getElementById('canvas-container');
    const containerWidth = container.clientWidth;
    const containerHeight = container.clientHeight;
    
    camera.aspect = containerWidth / containerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(containerWidth, containerHeight);
}

// Center and orient robot properly
export function centerAndOrientRobot() {
    if (robotGroup.children.length === 0) return;
    
    // Reset any previous transformations
    robotGroup.position.set(0, 0, 0);
    robotGroup.rotation.set(0, 0, 0);
    robotGroup.scale.set(1, 1, 1);
    
    // Calculate bounding box
    const boundingBox = new THREE.Box3().setFromObject(robotGroup);
    const center = boundingBox.getCenter(new THREE.Vector3());
    const size = boundingBox.getSize(new THREE.Vector3());
    
    // Center the robot - ensure it's standing on the origin
    robotGroup.position.set(-center.x, -boundingBox.min.y, -center.z);
    
    // Scale the robot to a reasonable size
    const maxDim = Math.max(size.x, size.y, size.z);
    if (maxDim > 0) {
        const scale = 1.0 / maxDim;
        robotGroup.scale.set(scale, scale, scale);
    }
    
    // Rotate to make the robot stand properly
    robotGroup.rotation.set(-Math.PI/2, 0, Math.PI);
} 