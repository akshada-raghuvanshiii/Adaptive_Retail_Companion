"""ARC assist logic: target lock, help-offer trigger and assist state machine.

Pure Python (standard library only) so it can be unit-tested without ROS,
Gazebo, a camera or a GPU. A ROS 2 node only needs to feed these classes
and publish what they return. Only arc_assist.appearance needs OpenCV.

Nothing in this package infers a person's intent or mental state. It works
with box positions, colour histograms and map-frame positions, and the help
offer is always confirmed by the shopper before the cart leads.
"""

from .target_lock import Detection, LockConfig, LockState, TargetLock  # noqa: F401
from .help_trigger import HelpTrigger, TriggerConfig  # noqa: F401
from .assist_fsm import AssistFSM, Mode  # noqa: F401
