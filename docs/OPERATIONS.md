# Local demonstration operations

These commands describe the intended deployment. Consult STATUS.md for what has actually been tested.

## Thor

Project root: `/home/jensen/.goske/codex/orbital-mechanic`.

ROS packages: `ros-jazzy-desktop`, `ros-jazzy-ros-gz`, `ros-jazzy-foxglove-bridge`, `ros-jazzy-gz-ros2-control`, `ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup`. GUI transport uses the existing `Xtigervnc` binary; Openbox provides window management if installed.

Run `bash scripts/launch-thor.sh` from the project root. It creates a transient user service named `orbital-demo` whose cgroup owns every Gazebo descendant, including processes that start separate sessions. It does not enable boot startup. It uses display `:42`, ROS domain `42`, software rendering and localhost-only VNC `5942` / Foxglove `8765`. It terminates its child processes when interrupted or when a required process exits. Do not run two copies; existing display/port conflicts should fail rather than replace another session.

To probe sensor delivery in a second shell:

```bash
source scripts/ros-env.sh
/usr/bin/python3 scripts/probe-sensors.py
```

Logs and runtime data are excluded from Git. The probe reports metadata and counts, not image contents.

## Mac

Run `scripts/open-mac.sh user@verified-host` on the Mac. If host keys live outside the default SSH file, set `THOR_KNOWN_HOSTS` to the verified known-hosts file. The script opens both applications and reuses its own SSH control socket. To close only this tunnel, use `ssh -S runtime/thor-view.sock -O exit user@verified-host`.

It forwards local ports `15942` to Thor `5942`, and `18765` to Thor `8765` using an authenticated SSH tunnel with verified host keys. No credential is included in this repository.

Run `scripts/setup-gazebo-viewer.sh` once on Mac, then `scripts/open-gazebo-web.sh`. The default viewer is now `http://127.0.0.1:15943/vnc.html?autoconnect=true&resize=scale`. The Mac-local `orbital-viewer` tmux session runs websockify0.13.0 and serves pinned noVNC1.7.0, forwarding only through the existing SSH tunnel. Native macOS Screen Sharing failed despite a healthy RFB connection; the browser path was confirmed visible by the user. No additional VNC password is needed. In Foxglove Studio, open a Foxglove WebSocket connection to `ws://localhost:18765`. Add a 3D panel with fixed frame `odom`, `/scan`, and transforms; an Image panel for `/camera/image_raw`; and plot `/odom` or `/imu` as needed.

No Foxglove cloud relay or upload is required for this LAN workflow. The application's own account requirements are separate from local bridge connectivity.

## Stop and recovery

Stop the demo with `bash scripts/stop-thor.sh`. Check `systemctl --user status orbital-demo.service` for status. The service stops the entire process cgroup; this matters because Gazebo's combined launcher creates independent server/GUI sessions. The internal `start-demo.sh` is not the recommended standalone entrypoint. Close the Mac SSH tunnel separately. Never use broad `pkill` commands or reboot Thor automatically after a graphics error.

The demo bridges only `/motion/guarded_cmd_vel` to the simulated drive. Send bounded stamped requests through `/motion/request`; startup is latched stopped. See [MOTION.md](MOTION.md) for reset/stop services and the trusted-local-process boundary. Autonomous navigation remains a later phase.

## Rendering and client compatibility

On the inspected Thor, Ogre2's device enumeration crashed in Mesa/EGL even with software rendering. The launcher selects `--render-engine ogre` (Ogre1), which produced camera and lidar data without rebooting the device. See [Gazebo's rendering troubleshooting](https://gazebosim.org/docs/harmonic/troubleshooting/) for the supported fallback.

The existing Mac app is Foxglove Studio 1.84.0. The APT bridge 3.6.0 uses the newer `foxglove.sdk.v1` handshake; the older client's `foxglove.websocket.v1` handshake is rejected. `scripts/build-foxglove-compat.sh` builds official bridge 0.8.5 at a fixed revision in an ignored overlay. The launcher sources that overlay if present, leaving the APT package installed. This is a local compatibility route, not a recommendation to expose the legacy bridge publicly. New Foxglove versions have different licensing/account requirements; do not upgrade or purchase a subscription automatically.

Gazebo 8 uses `~/.gz` for some caches regardless of XDG settings. On this host, the directory created by our first test was moved into `runtime/gazebo-home`, with `~/.gz` retained only as a symlink. The launcher refuses to silently move an unrelated pre-existing cache. ROS logs, Qt/XDG cache and launch temporary files also live under `runtime/`.

The installed Studio1.84 app also failed during Mac startup, independently of bridge compatibility. Current Foxglove3.4.2 is installed separately in the workspace; `open-mac.sh` prefers it (override with `FOXGLOVE_APP`). The user completed sign-in; live image/calibration and TF subscriptions are verified. Detailed panel layout remains a follow-up. Do not create an account, activate a subscription or upload data on the user’s behalf.

Stop only the Mac browser proxy with `tmux kill-session -t orbital-viewer`; this does not stop Thor. Keep Thor operations in its persistent `orbital-work` tmux session; the demo remains owned by the orbital-demo user-service cgroup.

## Local DDS image transport

Source `scripts/ros-env.sh` for every Thor project shell. The demo launcher sources it automatically. It selects domain42, local discovery, the Gazebo partition and `config/fastdds-local.xml`. The profile uses shared-memory transport with a 16 MiB segment per participant and disables builtin transports. It changes only these project processes; no global kernel buffer settings are changed. All ROS participants for this demo must run as the same local user and source this environment. Remote viewing continues through the loopback Foxglove bridge and authenticated SSH tunnel.

The source camera produces 640×480 RGB8 frames (~0.92 MB each). Default transport measurements showed heavy loss/stale images; a QoS-only change did not pass freshness. The project profile delivered100frames/20s with final image age~14ms in the first before/after trial. The sensor probe now rejects any live topic not received in the last second and images more than one simulation second behind `/clock`; static transforms are exempt from receive freshness.

Transport rationale: [Fast DDS shared-memory descriptor and segment-size warning](https://fast-dds.docs.eprosima.com/en/2.x/fastdds/transport/shared_memory/shared_memory.html). The measured result establishes the project configuration's behavior; cumulative host UDP error counters alone do not identify the root cause of every lost frame.
