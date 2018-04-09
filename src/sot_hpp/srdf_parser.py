import xml.etree.ElementTree as ET

def _read_name (xml):
    return str(xml.attrib["name"])

def _read_clearance (xml):
    return float(xml.attrib.get('clearance', 0))

def _read_mask (xml):
    masksTag = xml.findall('mask')
    if len(masksTag) > 1:
        raise ValueError ("Handle needs at most one tag mask")
    elif len(masksTag) == 1:
        mask = [ bool(v) for v in masksTag[0].text.split() ]
    else:
        mask = (True, ) * 6
    if len(mask) != 6:
        raise ValueError ("Tag mask must contain 6 booleans")
    return tuple (mask)

def _read_joints (xml):
    jointTags = xml.findall('joint')
    return tuple ( [ jt.attrib["name"] for jt in jointTags ] )

def _read_position (xml):
    positionsTag = xml.findall('position')
    if len(positionsTag) != 1:
        raise ValueError ("Gripper needs exactly one tag position")
    return tuple ([ float(x) for x in positionsTag[0].text.split() ])

def _read_link (xml):
    linksTag = xml.findall('link')
    if len(linksTag) != 1:
        raise ValueError ("Gripper needs exactly one tag link")
    return str(linksTag[0].attrib['name'])

def parse_srdf (srdf, packageName = None, prefix = None):
    """
    parameters:
    - packageName: if provided, the filename is considered relative to this ROS package
    - prefix: if provided, the name of the elements will be prepended with
             prefix + "/"
    """
    import os
    if packageName is not None:
        from rospkg import RosPack
        rospack = RosPack()
        path = rospack.get_path(packageName)
        srdfFn = os.path.join(path, srdf)
    else:
        srdfFn = srdf

    # tree = ET.fromstring (srdfFn)
    tree = ET.parse (srdfFn)
    root = tree.getroot()

    grippers = {}
    for xml in root.iter('gripper'):
        n = _read_name (xml)
        g = { "robot":     prefix,
              "name":      n,
              "clearance": _read_clearance (xml),
              "link":      _read_link (xml),
              "position":  _read_position (xml),
              "joints":    _read_joints (xml),
              }
        grippers[ prefix + "/" + n if prefix is not None else n] = g

    handles = {}
    for xml in root.iter('handle'):
        n = _read_name (xml)
        h = { "robot":     prefix,
              "name":      n,
              "clearance": _read_clearance (xml),
              "link":      _read_link (xml),
              "position":  _read_position (xml),
              "mask":      _read_mask (xml),
              }
        handles[ prefix + "/" + n if prefix is not None else n] = h
    return { "grippers": grippers, "handles": handles}
