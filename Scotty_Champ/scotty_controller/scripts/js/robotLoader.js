// Robot model loading and building
import { config } from './config.js';
import { parseURDF } from './urdfParser.js';
import { loadSTLMesh, resolvePackageUrl, materials } from './meshLoader.js';
import { robotGroup, centerAndOrientRobot } from './scene.js';
import { jointMap } from './rosConnection.js';

// Load URDF model and build robot
export async function loadRobot() {
    try {
        // Load URDF model using the path from config
        const urdfPath = config.urdf_path;
        
        document.getElementById('loading').style.display = 'flex';
        
        const response = await fetch(urdfPath);
        if (!response.ok) {
            throw new Error(`Failed to fetch URDF: ${response.status} ${response.statusText}`);
        }
        
        const urdfText = await response.text();
        const urdf = parseURDF(urdfText);
        await processURDFData(urdf, urdfPath);
        
        document.getElementById('loading').style.display = 'none';
        return true;
    } catch (error) {
        console.error('Error loading robot:', error);
        document.getElementById('status').textContent = 'Error: ' + error.message;
        document.getElementById('loading').style.display = 'none';
        
        // Add fallback - show error details and path troubleshooting tips
        const errorDetails = document.createElement('div');
        errorDetails.style.marginTop = '20px';
        errorDetails.innerHTML = `
            <p>Troubleshooting tips:</p>
            <ul>
                <li>Check that the URDF file exists at: <code>${config.urdf_path}</code></li>
                <li>Check that mesh files exist in the scotty_description package</li>
                <li>Check browser console for detailed error messages</li>
                <li>Try adding URL parameters to specify alternate paths</li>
            </ul>
        `;
        document.getElementById('loading').appendChild(errorDetails);
        return false;
    }
}

// Load robot from URDF
async function processURDFData(urdf, baseUrl) {
    console.log("Loading robot:", urdf.name);
    
    // Base URL for relative paths
    const basePath = baseUrl.substring(0, baseUrl.lastIndexOf('/') + 1);
    
    // Load all links with meshes
    const linkPromises = [];
    const linkObjects = {};
    
    // First phase: load all meshes
    for (const linkName in urdf.links) {
        const link = urdf.links[linkName];
        
        if (link.mesh) {
            let meshPath = link.mesh;
            
            // Handle package:// URIs
            meshPath = resolvePackageUrl(meshPath);
            
            console.log(`Loading mesh for ${linkName}: ${meshPath}`);
            
            let loadPromise;
            if (meshPath.toLowerCase().endsWith('.stl') || meshPath.toLowerCase().endsWith('.STL')) {
                // Determine material type (links, joints, end effectors)
                let material = getMaterial('link');
                if (linkName.toLowerCase().includes('body') || linkName.toLowerCase().includes('base')) {
                    material = getMaterial('body');
                } else if (linkName.toLowerCase().includes('joint') || 
                          linkName.toLowerCase().includes('hip') || 
                          linkName.toLowerCase().includes('knee') || 
                          linkName.toLowerCase().includes('ankle') ||
                          linkName.toLowerCase().includes('abad')) {
                    material = getMaterial('joint');
                } else if (linkName.toLowerCase().includes('hand') || 
                          linkName.toLowerCase().includes('foot') || 
                          linkName.toLowerCase().includes('gripper') || 
                          linkName.toLowerCase().includes('end')) {
                    material = getMaterial('end');
                }
                
                loadPromise = loadSTLMesh(
                    meshPath,
                    material,
                    link.origin,
                    link.rpy,
                    null,
                    (mesh) => {
                        mesh.name = linkName;
                        // Create a group for this link
                        const linkGroup = new THREE.Group();
                        linkGroup.name = linkName + "_group";
                        linkGroup.add(mesh);
                        linkObjects[linkName] = linkGroup;
                    }
                );
            } else {
                console.warn(`Unsupported mesh type for ${linkName}: ${meshPath}`);
                const linkGroup = new THREE.Group();
                linkGroup.name = linkName + "_group";
                linkObjects[linkName] = linkGroup;
                loadPromise = Promise.resolve();
            }
            
            linkPromises.push(loadPromise);
        } else {
            // Create an empty group for links without meshes
            const linkGroup = new THREE.Group();
            linkGroup.name = linkName + "_group";
            linkObjects[linkName] = linkGroup;
        }
    }
    
    try {
        await Promise.all(linkPromises);
        console.log("All meshes loaded");
        
        // Second phase: Organize links by joint hierarchy
        // Find the root link (usually base_link)
        let rootLinkName = "base_link"; // Default
        
        // Add all links that aren't children in any joint
        const childLinks = new Set();
        for (const jointName in urdf.joints) {
            const joint = urdf.joints[jointName];
            childLinks.add(joint.child);
        }
        
        // Find links that are not children
        for (const linkName in urdf.links) {
            if (!childLinks.has(linkName)) {
                rootLinkName = linkName;
                break;
            }
        }
        
        console.log(`Root link: ${rootLinkName}`);
        
        // Start with the root link
        if (linkObjects[rootLinkName]) {
            robotGroup.add(linkObjects[rootLinkName]);
        }
        
        // Function to recursively add children links
        function addChildLinks(parentLinkName) {
            for (const jointName in urdf.joints) {
                const joint = urdf.joints[jointName];
                
                if (joint.parent === parentLinkName) {
                    const childLinkName = joint.child;
                    const childLink = linkObjects[childLinkName];
                    
                    if (childLink) {
                        const parentLink = linkObjects[parentLinkName];
                        if (parentLink) {
                            parentLink.add(childLink);
                            console.log(`Added ${childLinkName} to ${parentLinkName}`);
                            
                            // Apply joint transformations if available
                            if (joint.origin) {
                                childLink.position.set(
                                    joint.origin.x || 0,
                                    joint.origin.y || 0,
                                    joint.origin.z || 0
                                );
                            }

                            if (joint.rpy) {
                                childLink.rotation.set(
                                    joint.rpy.roll || 0,
                                    joint.rpy.pitch || 0,
                                    joint.rpy.yaw || 0,
                                    'ZYX'
                                );
                            }
                            
                            // Store joint for animation
                            if (joint.type !== 'fixed') {
                                jointMap[jointName] = {
                                    object: childLink,
                                    type: joint.type,
                                    axis: joint.axis,
                                    originalRotation: childLink.rotation.clone(),
                                    originalPosition: childLink.position.clone()
                                };
                            }
                            
                            // Recursively add children of this link
                            addChildLinks(childLinkName);
                        }
                    }
                }
            }
        }
        
        // Start building the hierarchy
        addChildLinks(rootLinkName);
        
        // If the recursive approach fails, add all remaining links directly to robot group
        for (const linkName in linkObjects) {
            const link = linkObjects[linkName];
            if (link.parent === null) {
                robotGroup.add(link);
            }
        }
        
        // Position and orient robot properly
        centerAndOrientRobot();
        
        console.log("Robot hierarchy built");
        
    } catch (error) {
        console.error("Error loading meshes:", error);
        document.getElementById('status').textContent = 'Error loading robot model: ' + error.message;
        throw error;
    }
}

/**
 * Load robot model from URDF file
 * @param {string} urdfPath - Path to the URDF file
 * @returns {Promise<Object>} Robot object with joints and links
 */
export async function loadRobotFromURDF(urdfPath) {
    try {
        // Fetch URDF file
        const response = await fetch(urdfPath);
        if (!response.ok) {
            throw new Error(`Failed to fetch URDF: ${response.statusText}`);
        }
        
        const text = await response.text();
        const parser = new DOMParser();
        const urdfXml = parser.parseFromString(text, 'text/xml');
        
        // Create robot object
        const robot = {
            name: urdfXml.querySelector('robot').getAttribute('name'),
            links: {},
            joints: {},
            object: new THREE.Group()
        };
        
        // Load links
        const linkElements = urdfXml.querySelectorAll('link');
        for (const linkElement of linkElements) {
            const linkName = linkElement.getAttribute('name');
            robot.links[linkName] = {
                name: linkName,
                mesh: null,
                origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] },
                inertial: null,
                visual: null,
                collision: null,
                parent: null,
                children: []
            };
            
            // Parse visual elements
            const visualElement = linkElement.querySelector('visual');
            if (visualElement) {
                const originElement = visualElement.querySelector('origin');
                if (originElement) {
                    const xyz = originElement.getAttribute('xyz');
                    const rpy = originElement.getAttribute('rpy');
                    
                    if (xyz) {
                        robot.links[linkName].origin.xyz = xyz.split(' ').map(parseFloat);
                    }
                    
                    if (rpy) {
                        robot.links[linkName].origin.rpy = rpy.split(' ').map(parseFloat);
                    }
                }
                
                // Get mesh geometry
                const geometryElement = visualElement.querySelector('geometry');
                if (geometryElement) {
                    const meshElement = geometryElement.querySelector('mesh');
                    if (meshElement) {
                        const meshPath = meshElement.getAttribute('filename');
                        robot.links[linkName].visual = { meshPath };
                    }
                }
            }
        }
        
        // Load joints
        const jointElements = urdfXml.querySelectorAll('joint');
        for (const jointElement of jointElements) {
            const jointName = jointElement.getAttribute('name');
            const jointType = jointElement.getAttribute('type');
            const parentElement = jointElement.querySelector('parent');
            const childElement = jointElement.querySelector('child');
            
            if (parentElement && childElement) {
                const parentLink = parentElement.getAttribute('link');
                const childLink = childElement.getAttribute('link');
                
                robot.joints[jointName] = {
                    name: jointName,
                    type: jointType,
                    parent: parentLink,
                    child: childLink,
                    origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] },
                    axis: [0, 0, 0]
                };
                
                // Parse origin
                const originElement = jointElement.querySelector('origin');
                if (originElement) {
                    const xyz = originElement.getAttribute('xyz');
                    const rpy = originElement.getAttribute('rpy');
                    
                    if (xyz) {
                        robot.joints[jointName].origin.xyz = xyz.split(' ').map(parseFloat);
                    }
                    
                    if (rpy) {
                        robot.joints[jointName].origin.rpy = rpy.split(' ').map(parseFloat);
                    }
                }
                
                // Parse axis
                const axisElement = jointElement.querySelector('axis');
                if (axisElement) {
                    const xyz = axisElement.getAttribute('xyz');
                    if (xyz) {
                        robot.joints[jointName].axis = xyz.split(' ').map(parseFloat);
                    }
                }
                
                // Set up parent-child relationships
                if (robot.links[parentLink] && robot.links[childLink]) {
                    robot.links[parentLink].children.push(childLink);
                    robot.links[childLink].parent = parentLink;
                }
            }
        }
        
        // Build robot tree starting from root links (links with no parent)
        const rootLinks = Object.values(robot.links).filter(link => !link.parent);
        for (const rootLink of rootLinks) {
            await buildLinkTree(rootLink.name, robot, null);
        }
        
        console.log('Robot loaded successfully:', robot.name);
        return robot;
        
    } catch (error) {
        console.error('Error loading robot from URDF:', error);
        throw error;
    }
}

/**
 * Build the link tree recursively
 * @param {string} linkName - Current link name
 * @param {Object} robot - Robot object
 * @param {THREE.Object3D} parentObject - Parent THREE.js object
 */
async function buildLinkTree(linkName, robot, parentObject) {
    const link = robot.links[linkName];
    
    // Create mesh for this link
    const linkObject = new THREE.Group();
    linkObject.name = linkName;
    
    // Apply link origin transformation
    applyTransform(linkObject, link.origin.xyz, link.origin.rpy);
    
    // Add to parent or to robot root
    if (parentObject) {
        parentObject.add(linkObject);
    } else {
        robot.object.add(linkObject);
    }
    
    // Load visual mesh if available
    if (link.visual && link.visual.meshPath) {
        try {
            const mesh = await loadMesh(link.visual.meshPath, link.origin);
            if (mesh) {
                linkObject.add(mesh);
            }
        } catch (error) {
            console.warn(`Failed to load mesh for link ${linkName}:`, error);
        }
    } else {
        // Create a simple shape for links without meshes
        const geometry = new THREE.BoxGeometry(0.05, 0.05, 0.05);
        const mesh = new THREE.Mesh(geometry, getMaterial('link'));
        linkObject.add(mesh);
    }
    
    // Process child links
    for (const childLinkName of link.children) {
        const childLink = robot.links[childLinkName];
        
        // Find joint connecting to child
        const joint = Object.values(robot.joints).find(
            j => j.parent === linkName && j.child === childLinkName
        );
        
        if (joint) {
            // Create joint object
            const jointObject = new THREE.Group();
            jointObject.name = joint.name;
            
            // Apply joint origin transformation
            applyTransform(jointObject, joint.origin.xyz, joint.origin.rpy);
            
            // Add joint to parent link
            linkObject.add(jointObject);
            
            // Recursively build child link tree from this joint
            await buildLinkTree(childLinkName, robot, jointObject);
        }
    }
}

/**
 * Get material for different parts
 * @param {string} type - Type of part ('link', 'joint', 'body', 'end')
 * @returns {THREE.Material} Material for the mesh
 */
function getMaterial(type) {
    // Create materials for different part types
    const materials = {
        link: new THREE.MeshPhongMaterial({
            color: 0x888899,
            specular: 0x111111,
            shininess: 200
        }),
        joint: new THREE.MeshPhongMaterial({
            color: 0x555555,
            specular: 0x111111,
            shininess: 200
        }),
        body: new THREE.MeshPhongMaterial({
            color: 0x333344,
            specular: 0x111111,
            shininess: 200
        }),
        end: new THREE.MeshPhongMaterial({
            color: 0x992222,
            specular: 0x111111,
            shininess: 200
        })
    };
    
    // Return the requested material or default to link material
    return materials[type] || materials.link;
}

/**
 * Load mesh from file
 * @param {string} meshPath - Path to mesh file
 * @param {Object} origin - Origin transformation data
 * @returns {Promise<THREE.Mesh>} Loaded mesh
 */
async function loadMesh(meshPath, origin) {
    return new Promise((resolve, reject) => {
        // Handle package:// URLs
        let resolvedPath = meshPath;
        if (meshPath.startsWith('package://')) {
            const parts = meshPath.substring(10).split('/', 1);
            const packageName = parts[0];
            const restPath = meshPath.substring(10 + packageName.length);
            
            // Check if package exists in config
            if (config.packages && config.packages[packageName]) {
                resolvedPath = config.ros_package_path + config.packages[packageName] + restPath;
            } else {
                resolvedPath = config.ros_package_path + packageName + restPath;
            }
        }
        
        // Load STL file
        const loader = new THREE.STLLoader();
        loader.load(
            resolvedPath,
            (geometry) => {
                const mesh = new THREE.Mesh(geometry, getMaterial('link'));
                mesh.castShadow = true;
                mesh.receiveShadow = true;
                resolve(mesh);
            },
            undefined,
            (error) => {
                console.error('Error loading mesh:', error);
                reject(error);
            }
        );
    });
}

/**
 * Apply position and rotation transform to an object
 * @param {THREE.Object3D} object - THREE.js object to transform
 * @param {Array} position - [x, y, z] position
 * @param {Array} rotation - [roll, pitch, yaw] rotation
 */
function applyTransform(object, position, rotation) {
    // Apply position
    if (position && position.length === 3) {
        object.position.set(position[0], position[1], position[2]);
    }
    
    // Apply rotation
    if (rotation && rotation.length === 3) {
        // Convert roll-pitch-yaw to Euler angles
        const [roll, pitch, yaw] = rotation;
        object.rotation.set(roll, pitch, yaw, 'XYZ');
    }
} 