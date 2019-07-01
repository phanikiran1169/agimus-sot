Bridge between SoT and Agimus

# TODO

- The visual tags should be declared and added to URDF / SRDF. Agimus-sot could parse them so that,
  for each gripper and handle, it is possible to know whether, there is a visual feedback.
  A possible implementation:
  - URDF: add the tag frame as a link.
  - SRDF: add a XML tag *visual_tag* that specifies the link.
  The issue (although nothing too hard) is that one must both parse the URDF and SRDF to know which joints have a visual tag.
