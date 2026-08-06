# Invoked via `--exec` from runheadless.sh's CMD (see Dockerfile). Kit runs
# this after the app is fully loaded (extensions started, default blank
# stage created), so opening a stage here cleanly replaces that default.
import os
import sys

import omni.timeline
import omni.usd

sys.path.insert(0, "/isaac-sim")
from ros2_control_graph import setup_ros2_control_graph

STAGE_PATH = "/isaac-sim/content/scene.usd"

if os.path.isfile(STAGE_PATH):
    print(f"[autoload_stage] Opening saved stage: {STAGE_PATH}")
    omni.usd.get_context().open_stage(STAGE_PATH)
else:
    print(f"[autoload_stage] No saved stage at {STAGE_PATH}; starting with the default stage.")

# Runs whether or not a saved stage was opened: setup_ros2_control_graph()
# itself checks for /World/lite6 and no-ops if it isn't present.
setup_ros2_control_graph()

# The ROS 2 control graph's nodes only run on timeline ticks (OnPlaybackTick),
# and physics (so the articulation drives actually respond) only steps while
# playing, so start playback automatically rather than requiring someone to
# open the WebRTC client and press Play just to make ROS 2 control work.
omni.timeline.get_timeline_interface().play()
