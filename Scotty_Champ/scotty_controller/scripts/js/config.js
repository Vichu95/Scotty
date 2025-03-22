// Configuration for robot visualizer

// Default configuration settings
export const config = {
    // ROS connection
    rosbridge_url: 'ws://0.0.0.0:9090',
    joint_states_topic: '/joint_states',
    
    // Robot model
    urdf_path: '../../scotty_description/urdf/scotty_description.urdf',
    
    // Path resolution
    ros_package_path: '../',
    
    // Package mapping
    packages: {
        'scotty_description': '../scotty_description'
    }
};

/**
 * Parse URL parameters to override configuration
 * Example URL: ?rosbridge_url=ws://192.168.1.100:9090&urdf_path=path/to/model.urdf
 */
export function parseUrlParams() {
    const urlParams = new URLSearchParams(window.location.search);
    
    // ROS connection settings
    if (urlParams.has('rosbridge_url')) {
        config.rosbridge_url = urlParams.get('rosbridge_url');
    }
    
    // Robot model path
    if (urlParams.has('urdf_path')) {
        config.urdf_path = urlParams.get('urdf_path');
    }
    
    // ROS package base path
    if (urlParams.has('ros_package_path')) {
        config.ros_package_path = urlParams.get('ros_package_path');
    }
    
    // Joint states topic
    if (urlParams.has('joint_states_topic')) {
        config.joint_states_topic = urlParams.get('joint_states_topic');
    }
    
    // Log updated configuration
    console.log('Configuration loaded with parameters:', config);
} 