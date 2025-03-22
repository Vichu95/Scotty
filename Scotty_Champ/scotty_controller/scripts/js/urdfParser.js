// URDF parsing functionality

// Parse URDF XML into a structured object
export function parseURDF(text) {
    const parser = new DOMParser();
    const xmlDoc = parser.parseFromString(text, "text/xml");
    
    // Extract robot info
    const robotEl = xmlDoc.querySelector('robot');
    const robotName = robotEl.getAttribute('name');
    
    // Parse links
    const links = {};
    xmlDoc.querySelectorAll('link').forEach(linkEl => {
        const linkName = linkEl.getAttribute('name');
        links[linkName] = { name: linkName };
        
        // Extract visual elements
        const visualEl = linkEl.querySelector('visual');
        if (visualEl) {
            const geometryEl = visualEl.querySelector('geometry');
            if (geometryEl) {
                const meshEl = geometryEl.querySelector('mesh');
                if (meshEl) {
                    links[linkName].mesh = meshEl.getAttribute('filename');
                }
            }
            
            // Extract origin if any
            const originEl = visualEl.querySelector('origin');
            if (originEl) {
                const xyz = originEl.getAttribute('xyz');
                const rpy = originEl.getAttribute('rpy');
                
                links[linkName].origin = { x: 0, y: 0, z: 0 };
                links[linkName].rpy = { roll: 0, pitch: 0, yaw: 0 };
                
                if (xyz) {
                    const xyzParts = xyz.split(' ').map(parseFloat);
                    links[linkName].origin.x = xyzParts[0] || 0;
                    links[linkName].origin.y = xyzParts[1] || 0;
                    links[linkName].origin.z = xyzParts[2] || 0;
                }
                
                if (rpy) {
                    const rpyParts = rpy.split(' ').map(parseFloat);
                    links[linkName].rpy.roll = rpyParts[0] || 0;
                    links[linkName].rpy.pitch = rpyParts[1] || 0;
                    links[linkName].rpy.yaw = rpyParts[2] || 0;
                }
            }
        }
    });
    
    // Parse joints
    const joints = {};
    xmlDoc.querySelectorAll('joint').forEach(jointEl => {
        const jointName = jointEl.getAttribute('name');
        const jointType = jointEl.getAttribute('type');
        
        const parentEl = jointEl.querySelector('parent');
        const childEl = jointEl.querySelector('child');
        
        if (parentEl && childEl) {
            joints[jointName] = {
                name: jointName,
                type: jointType,
                parent: parentEl.getAttribute('link'),
                child: childEl.getAttribute('link')
            };
            
            // Extract origin if any
            const originEl = jointEl.querySelector('origin');
            if (originEl) {
                const xyz = originEl.getAttribute('xyz');
                const rpy = originEl.getAttribute('rpy');
                
                joints[jointName].origin = { x: 0, y: 0, z: 0 };
                joints[jointName].rpy = { roll: 0, pitch: 0, yaw: 0 };
                
                if (xyz) {
                    const xyzParts = xyz.split(' ').map(parseFloat);
                    joints[jointName].origin.x = xyzParts[0] || 0;
                    joints[jointName].origin.y = xyzParts[1] || 0;
                    joints[jointName].origin.z = xyzParts[2] || 0;
                }
                
                if (rpy) {
                    const rpyParts = rpy.split(' ').map(parseFloat);
                    joints[jointName].rpy.roll = rpyParts[0] || 0;
                    joints[jointName].rpy.pitch = rpyParts[1] || 0;
                    joints[jointName].rpy.yaw = rpyParts[2] || 0;
                }
            }
            
            // Extract axis if any
            const axisEl = jointEl.querySelector('axis');
            if (axisEl) {
                const xyzAttr = axisEl.getAttribute('xyz');
                if (xyzAttr) {
                    const xyzParts = xyzAttr.split(' ').map(parseFloat);
                    joints[jointName].axis = {
                        x: xyzParts[0] || 0,
                        y: xyzParts[1] || 0,
                        z: xyzParts[2] || 0
                    };
                }
            }
            
            // Extract limits if any
            const limitEl = jointEl.querySelector('limit');
            if (limitEl) {
                joints[jointName].limits = {
                    lower: parseFloat(limitEl.getAttribute('lower') || 0),
                    upper: parseFloat(limitEl.getAttribute('upper') || 0)
                };
            }
        }
    });
    
    return {
        name: robotName,
        links: links,
        joints: joints
    };
} 