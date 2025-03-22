// Mesh loading functionality
import { config } from './config.js';

// Materials for different robot parts
export const materials = {
    body: new THREE.MeshStandardMaterial({
        color: 0x607d8b,
        metalness: 0.7,
        roughness: 0.5
    }),
    link: new THREE.MeshStandardMaterial({
        color: 0xb0bec5,
        metalness: 0.5,
        roughness: 0.5
    }),
    joint: new THREE.MeshStandardMaterial({
        color: 0xf57c00, 
        metalness: 0.5,
        roughness: 0.7
    }),
    end: new THREE.MeshStandardMaterial({
        color: 0x4caf50,
        metalness: 0.2,
        roughness: 0.8
    })
};

// Load STL mesh
export function loadSTLMesh(url, material, origin, rpy, scale, callback) {
    return new Promise((resolve, reject) => {
        const loader = new THREE.STLLoader();
        loader.load(
            url,
            (geometry) => {
                // Create mesh with provided material
                const mesh = new THREE.Mesh(geometry, material);
                
                // Apply transformations if provided
                if (origin) {
                    mesh.position.set(origin.x || 0, origin.y || 0, origin.z || 0);
                }
                
                if (rpy) {
                    mesh.rotation.set(rpy.roll || 0, rpy.pitch || 0, rpy.yaw || 0, 'ZYX');
                }
                
                if (scale) {
                    mesh.scale.set(scale.x || 1, scale.y || 1, scale.z || 1);
                }
                
                // Enable shadows
                mesh.castShadow = true;
                mesh.receiveShadow = true;
                
                if (callback) callback(mesh);
                resolve(mesh);
            },
            (xhr) => {
                // Progress, if needed
            },
            (error) => {
                console.error('Error loading STL:', error);
                reject(error);
            }
        );
    });
}

// Resolve package URLs to actual file paths
export function resolvePackageUrl(meshPath) {
    // Handle package:// URIs (common in ROS)
    if (meshPath.startsWith("package://")) {
        // Extract the package path
        const packagePath = meshPath.substring(10);
        const slashIndex = packagePath.indexOf('/');
        if (slashIndex !== -1) {
            const packageName = packagePath.substring(0, slashIndex);
            const restPath = packagePath.substring(slashIndex + 1);
            
            // Use configuration to map package paths
            if (config.packages[packageName]) {
                meshPath = config.ros_package_path + config.packages[packageName] + '/' + restPath;
            } else {
                // Default fallback for unknown packages
                meshPath = config.ros_package_path + packageName + '/' + restPath;
            }
            
            console.log(`Resolved package path ${meshPath}`);
        }
    }
    
    return meshPath;
} 