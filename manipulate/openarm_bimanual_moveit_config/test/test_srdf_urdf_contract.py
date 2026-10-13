"""Contract test: every link and joint named in the SRDF must exist in the walkie URDF.

MoveIt only warns (and silently skips the entry) when the SRDF names a link or
joint that the URDF does not define, so stale names survive URDF refactors
unnoticed. This test expands gz_walkie.urdf.xacro from the installed
walkie_description share directory and the SRDF from this package's source
tree, then checks every reference: virtual_joint child links, group chains,
links and joints, group_state joints, end_effector parent links, passive
joints and disable_collisions pairs.
"""

import functools
import os

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro

SRDF_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..',
    'config',
    'openarm_bimanual.srdf',
)

# ros2_control modes the MoveIt launch files map hardware_type onto. The link
# and joint sets do not depend on the mode today; covering each mode keeps that
# assumption checked.
ROS2_CONTROL_MODES = ('mock', 'gazebo', 'real_robot', 'mujoco')

# The SRDF and the URDF both branch on right_joint2_fixed.
RIGHT_JOINT2_FIXED_VALUES = ('false', 'true')


def _child_elements(element, tag):
    """Return the direct child elements of a minidom element with the given tag."""
    return [
        node for node in element.childNodes
        if node.nodeType == node.ELEMENT_NODE and node.tagName == tag
    ]


@functools.lru_cache(maxsize=None)
def _urdf_names(ros2_control, right_joint2_fixed):
    """Expand the walkie URDF and return its (link names, joint names)."""
    xacro_path = os.path.join(
        get_package_share_directory('walkie_description'),
        'robots',
        'gz_walkie.urdf.xacro',
    )
    robot = xacro.process_file(
        xacro_path,
        mappings={
            'ros2_control': ros2_control,
            'use_arm': 'True',
            'right_joint2_fixed': right_joint2_fixed,
        },
    ).documentElement
    links = frozenset(e.getAttribute('name') for e in _child_elements(robot, 'link'))
    joints = frozenset(e.getAttribute('name') for e in _child_elements(robot, 'joint'))
    return links, joints


@functools.lru_cache(maxsize=None)
def _srdf_root(right_joint2_fixed):
    """Expand the SRDF (it uses xacro conditionals) and return its root element."""
    return xacro.process_file(
        SRDF_PATH, mappings={'right_joint2_fixed': right_joint2_fixed}
    ).documentElement


def _srdf_references(robot):
    """
    Collect every URDF name the SRDF refers to.

    Returns (link_refs, joint_refs), each a list of (name, context) tuples.
    The virtual_joint parent_frame is a TF frame outside the robot model, not a
    URDF link, so it is not collected.
    """
    link_refs = []
    joint_refs = []

    for vj in _child_elements(robot, 'virtual_joint'):
        context = f"virtual_joint '{vj.getAttribute('name')}' child_link"
        link_refs.append((vj.getAttribute('child_link'), context))

    for group in _child_elements(robot, 'group'):
        group_name = group.getAttribute('name')
        for chain in _child_elements(group, 'chain'):
            for attr in ('base_link', 'tip_link'):
                link_refs.append(
                    (chain.getAttribute(attr), f"group '{group_name}' chain {attr}")
                )
        for link in _child_elements(group, 'link'):
            link_refs.append((link.getAttribute('name'), f"group '{group_name}' link"))
        for joint in _child_elements(group, 'joint'):
            joint_refs.append((joint.getAttribute('name'), f"group '{group_name}' joint"))

    for state in _child_elements(robot, 'group_state'):
        context = (
            f"group_state '{state.getAttribute('name')}' "
            f"of group '{state.getAttribute('group')}'"
        )
        for joint in _child_elements(state, 'joint'):
            joint_refs.append((joint.getAttribute('name'), context))

    for ee in _child_elements(robot, 'end_effector'):
        context = f"end_effector '{ee.getAttribute('name')}' parent_link"
        link_refs.append((ee.getAttribute('parent_link'), context))

    for passive in _child_elements(robot, 'passive_joint'):
        joint_refs.append((passive.getAttribute('name'), 'passive_joint'))

    for pair in _child_elements(robot, 'disable_collisions'):
        link1 = pair.getAttribute('link1')
        link2 = pair.getAttribute('link2')
        context = f'disable_collisions {link1} / {link2}'
        link_refs.append((link1, context))
        link_refs.append((link2, context))

    return link_refs, joint_refs


@pytest.mark.parametrize('right_joint2_fixed', RIGHT_JOINT2_FIXED_VALUES)
@pytest.mark.parametrize('ros2_control', ROS2_CONTROL_MODES)
def test_srdf_names_exist_in_urdf(ros2_control, right_joint2_fixed):
    urdf_links, urdf_joints = _urdf_names(ros2_control, right_joint2_fixed)
    srdf = _srdf_root(right_joint2_fixed)
    # Groups may list the SRDF virtual joint alongside URDF joints.
    virtual_joints = {
        vj.getAttribute('name') for vj in _child_elements(srdf, 'virtual_joint')
    }
    link_refs, joint_refs = _srdf_references(srdf)

    assert link_refs and joint_refs, 'SRDF parsed without any link or joint references'

    missing = [
        f"link '{name}' ({context})"
        for name, context in link_refs
        if name not in urdf_links
    ] + [
        f"joint '{name}' ({context})"
        for name, context in joint_refs
        if name not in urdf_joints and name not in virtual_joints
    ]
    assert not missing, (
        f'SRDF references names missing from the URDF '
        f'(ros2_control={ros2_control}, right_joint2_fixed={right_joint2_fixed}):\n  '
        + '\n  '.join(missing)
    )


@pytest.mark.parametrize('right_joint2_fixed', RIGHT_JOINT2_FIXED_VALUES)
def test_srdf_group_references_are_defined(right_joint2_fixed):
    srdf = _srdf_root(right_joint2_fixed)
    groups = {g.getAttribute('name') for g in _child_elements(srdf, 'group')}

    refs = []
    for group in _child_elements(srdf, 'group'):
        for subgroup in _child_elements(group, 'group'):
            refs.append(
                (subgroup.getAttribute('name'),
                 f"subgroup of group '{group.getAttribute('name')}'")
            )
    for state in _child_elements(srdf, 'group_state'):
        refs.append((state.getAttribute('group'),
                     f"group_state '{state.getAttribute('name')}'"))
    for ee in _child_elements(srdf, 'end_effector'):
        refs.append((ee.getAttribute('group'),
                     f"end_effector '{ee.getAttribute('name')}' group"))
        if ee.getAttribute('parent_group'):
            refs.append((ee.getAttribute('parent_group'),
                         f"end_effector '{ee.getAttribute('name')}' parent_group"))

    missing = [f"group '{name}' ({context})" for name, context in refs if name not in groups]
    assert not missing, 'SRDF references undefined groups:\n  ' + '\n  '.join(missing)
