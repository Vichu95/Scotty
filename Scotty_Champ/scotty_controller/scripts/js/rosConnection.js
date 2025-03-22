// ROS connection handling module
import { config } from './config.js';

// Store joint mapping for updates
export let jointMap = {};

/**
 * Set up joint state subscriber to update robot model
 * @param {Object} robot - Robot object with joints and links
 * @returns {Object} Joint state subscriber
 */
export function setupJointStateSubscriber(robot) {
    if (!robot || !window.ros) {
        console.error('ROS or robot not initialized');
        return;
    }
    
    try {
        // Create joint state subscriber using the global ROS connection
        const jointStateSubscriber = new ROSLIB.Topic({
            ros: window.ros,
            name: config.joint_states_topic,
            messageType: 'sensor_msgs/JointState'
        });
        
        // Process joint state messages
        jointStateSubscriber.subscribe(function(message) {
            // Update status to show we're receiving joint states
            const statusElement = document.getElementById('status');
            if (statusElement) {
                statusElement.textContent = 'Receiving joint states';
            }
            
            // Process joint positions if robot has joints
            updateJointPositions(robot, message);
        });
        
        console.log('Joint state subscriber set up successfully');
        return jointStateSubscriber;
        
    } catch (error) {
        console.error('Error setting up joint state subscriber:', error);
        throw error;
    }
}

/**
 * Updates joint positions based on received joint state message
 * @param {Object} robot - Robot object with joints and links
 * @param {Object} message - ROS joint state message
 */
function updateJointPositions(robot, message) {
    if (!robot || !message) return;
    
    const { name, position } = message;
    
    // Iterate through received joint states
    for (let i = 0; i < name.length; i++) {
        const jointName = name[i];
        const jointPosition = position[i];
        
        // Find this joint in the robot model
        const joint = robot.joints[jointName];
        
        if (joint) {
            // Find the joint object in the THREE.js scene
            const jointObject = findJointObjectByName(robot.object, jointName);
            
            if (jointObject) {
                // Apply rotation based on joint axis and position
                const axis = joint.axis;
                
                // Create rotation quaternion based on joint axis and position
                if (joint.type === 'revolute' || joint.type === 'continuous') {
                    // For revolute joints, rotate around the specified axis
                    if (axis && axis.length === 3) {
                        // Reset rotation first (to avoid accumulation)
                        jointObject.rotation.set(0, 0, 0);
                        
                        // Determine which axis to rotate around
                        if (Math.abs(axis[0]) > 0.5) {
                            // Rotating around X axis
                            jointObject.rotateX(jointPosition * axis[0]);
                        }
                        if (Math.abs(axis[1]) > 0.5) {
                            // Rotating around Y axis
                            jointObject.rotateY(jointPosition * axis[1]);
                        }
                        if (Math.abs(axis[2]) > 0.5) {
                            // Rotating around Z axis
                            jointObject.rotateZ(jointPosition * axis[2]);
                        }
                    }
                } else if (joint.type === 'prismatic') {
                    // For prismatic joints, translate along the specified axis
                    if (axis && axis.length === 3) {
                        jointObject.position.set(
                            joint.origin.xyz[0] + axis[0] * jointPosition,
                            joint.origin.xyz[1] + axis[1] * jointPosition,
                            joint.origin.xyz[2] + axis[2] * jointPosition
                        );
                    }
                }
            }
        }
    }
}

/**
 * Find a joint object by name in the robot's THREE.js hierarchy
 * @param {THREE.Object3D} parent - Parent object to search in
 * @param {string} name - Joint name to find
 * @returns {THREE.Object3D|null} Found joint object or null
 */
function findJointObjectByName(parent, name) {
    if (!parent) return null;
    
    // Check if this is the object we're looking for
    if (parent.name === name) {
        return parent;
    }
    
    // Recursively search children
    for (const child of parent.children) {
        const result = findJointObjectByName(child, name);
        if (result) {
            return result;
        }
    }
    
    return null;
} 