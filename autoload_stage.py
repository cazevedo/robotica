# Invoked via `--exec` from runheadless.sh's CMD (see Dockerfile). Kit runs
# this after the app is fully loaded (extensions started, default blank
# stage created), so opening a stage here cleanly replaces that default.
import os

import omni.usd

STAGE_PATH = "/isaac-sim/content/scene.usd"

if os.path.isfile(STAGE_PATH):
    print(f"[autoload_stage] Opening saved stage: {STAGE_PATH}")
    omni.usd.get_context().open_stage(STAGE_PATH)
else:
    print(f"[autoload_stage] No saved stage at {STAGE_PATH}; starting with the default stage.")
