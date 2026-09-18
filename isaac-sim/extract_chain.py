# Invoked via autoload_stage.py's --exec hook, right after the ROS 2
# control graph is built (so /World/lite6 is already loaded). Walks the
# live Lite6 articulation with pxr and caches the extracted chain to
# content/lite6_chain.json, so dh_lab's ground-truth model can load it at
# tool runtime with no pxr, no Isaac Sim, and no network dependency at
# all -- see dh_lab/ground_truth/usd_walk.py for why this needs pxr and
# a loaded stage, and dh_lab/ground_truth/{chain,analytic_fk}.py for why
# everything downstream of the cached JSON does not.
import glob
import sys

# Kit's own bundled `pxr` isn't on sys.path by default (confirmed in
# CLAUDE.md's own prior USD-inspection notes); borrow it from extscache,
# same as that earlier investigation did.
for path in glob.glob("/isaac-sim/extscache/omni.usd.libs-*"):
    if path not in sys.path:
        sys.path.insert(0, path)

# The dh_lab package itself: bind-mounted (see run.sh), not baked into
# the image, so it can be edited without a container rebuild.
sys.path.insert(0, "/isaac-sim/ros2_ws/src/dh_lab")

CHAIN_OUTPUT_PATH = "/isaac-sim/content/lite6_chain.json"
ROBOT_PATH = "/World/lite6"


def extract_and_cache_chain():
    import omni.usd

    from dh_lab.ground_truth.usd_walk import walk_lite6_chain

    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath(ROBOT_PATH).IsValid():
        print(f"[extract_chain] No prim at {ROBOT_PATH}; skipping chain extraction.")
        return

    try:
        chain = walk_lite6_chain(stage, ROBOT_PATH)
    except Exception as exc:  # noqa: BLE001 -- log and continue; never crash the sim over this
        print(f"[extract_chain] FAILED to extract the Lite6 chain: {exc}")
        return

    chain.save(CHAIN_OUTPUT_PATH)
    print(f"[extract_chain] Wrote {CHAIN_OUTPUT_PATH}")
    print(f"[extract_chain] base_link={chain.base_link!r} tool_link={chain.tool_link!r}")
    for j in chain.joints:
        lo = f"{j.lower_limit:.4f}" if j.lower_limit is not None else "None"
        hi = f"{j.upper_limit:.4f}" if j.upper_limit is not None else "None"
        print(
            f"[extract_chain]   {j.name}: {j.parent_link} -> {j.child_link}, "
            f"axis={j.axis}, limits=({lo}, {hi}) rad"
        )
