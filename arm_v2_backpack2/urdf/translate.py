import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R

def rpy_to_quat(rpy):
    r = R.from_euler('xyz', rpy)
    return r.as_quat()

def convert_urdf_to_mujoco(urdf_file, mujoco_file):
    tree = ET.parse(urdf_file)
    root = tree.getroot()

    mujoco_root = ET.Element('mujoco', model=root.attrib['name'])
    compiler = ET.SubElement(mujoco_root, 'compiler', angle='degree', coordinate='local')
    option = ET.SubElement(mujoco_root, 'option', timestep='0.002', gravity='0 0 -9.81', iterations='50', integrator='RK4')

    worldbody = ET.SubElement(mujoco_root, 'worldbody')

    for link in root.findall('link'):
        body = ET.SubElement(worldbody, 'body', name=link.attrib['name'])
        for inertial in link.findall('inertial'):
            pos = inertial.find('origin').attrib['xyz']
            mass = inertial.find('mass').attrib['value']
            inertia = inertial.find('inertia')
            diaginertia = f"{inertia.attrib['ixx']} {inertia.attrib['iyy']} {inertia.attrib['izz']}"
            ET.SubElement(body, 'inertial', pos=pos, mass=mass, diaginertia=diaginertia)
        for visual in link.findall('visual'):
            geom = ET.SubElement(body, 'geom', type='mesh', mesh=link.attrib['name'], pos='0 0 0', rgba='1 1 1 1')

    for joint in root.findall('joint'):
        parent = joint.find('parent').attrib['link']
        child = joint.find('child').attrib['link']
        pos = joint.find('origin').attrib['xyz']
        rpy = joint.find('origin').attrib['rpy']
        quat = rpy_to_quat([float(x) for x in rpy.split()])
        axis = joint.find('axis').attrib['xyz']
        joint_type = joint.attrib['type']
        if joint_type == 'revolute':
            joint_type = 'hinge'
        ET.SubElement(worldbody.find(f"body[@name='{parent}']"), 'joint', name=joint.attrib['name'], type=joint_type, pos=pos, axis=axis, quat=' '.join(map(str, quat)))

    tree = ET.ElementTree(mujoco_root)
    tree.write(mujoco_file)

convert_urdf_to_mujoco('arm_v2_backpack2.urdf', 'arm_v2_backpack2_mujoco.xml')