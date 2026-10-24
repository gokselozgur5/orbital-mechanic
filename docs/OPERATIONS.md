# Local demonstration operations

These commands describe the intended deployment. Consult STATUS.md for what has actually been tested.

## Thor

Project root: `/home/jensen/.goske/codex/orbital-mechanic`.

ROS packages: `ros-jazzy-desktop`, `ros-jazzy-ros-gz`, `ros-jazzy-foxglove-bridge`, `ros-jazzy-gz-ros2-control`, `ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup`. GUI transport uses the existing `Xtigervnc` binary; Openbox provides window management if installed.

Run `bash scripts/launch-thor.sh` from the project root. It creates a transient user service named `orbital-demo` whose cgroup owns every Gazebo descendant, including processes that start separate sessions. It does not enable boot startup. It uses display `:42`, ROS domain `42`, software rendering and localhost-only VNC `5942` / Foxglove `8765`. It terminates its child processes when interrupted or when a required process exits. Do not run two copies; existing display/port conflicts should fail rather than replace another session.

To probe sensor delivery in a second shell:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 scripts/probe-sensors.py
```

Logs and runtime data are excluded from Git. The probe reports metadata and counts, not image contents.

## Mac

Run `scripts/open-mac.sh user@verified-host` on the Mac. If host keys live outside the default SSH file, set `THOR_KNOWN_HOSTS` to the verified known-hosts file. The script opens both applications and reuses its own SSH control socket. To close only this tunnel, use `ssh -S runtime/thor-view.sock -O exit user@verified-host`.

It forwards local ports `15942` to Thor `5942`, and `18765` to Thor `8765` using an authenticated SSH tunnel with verified host keys. No credential is included in this repository.

Open `vnc://localhost:15942` using macOS Screen Sharing. In Foxglove Studio, open a Foxglove WebSocket connection to `ws://localhost:18765`. Add a 3D panel with fixed frame `odom`, `/scan`, and transforms; an Image panel for `/camera/image_raw`; and plot `/odom` or `/imu` as needed.

No Foxglove cloud relay or upload is required for this LAN workflow. The application's own account requirements are separate from local bridge connectivity.

## Stop and recovery

Stop the demo with `bash scripts/stop-thor.sh`. Check `systemctl --user status orbital-demo.service` for status. The service stops the entire process cgroup; this matters because Gazebo's combined launcher creates independent server/GUI sessions. The internal `start-demo.sh` is not the recommended standalone entrypoint. Close the Mac SSH tunnel separately. Never use broad `pkill` commands or reboot Thor automatically after a graphics error.

The current demo intentionally does not bridge `/cmd_vel`: autonomous drive needs the command guard milestone first.

## Rendering and client compatibility

On the inspected Thor, Ogre2's device enumeration crashed in Mesa/EGL even with software rendering. The launcher selects `--render-engine ogre` (Ogre1), which produced camera and lidar data without rebooting the device. See [Gazebo's rendering troubleshooting](https://gazebosim.org/docs/harmonic/troubleshooting/) for the supported fallback.

The existing Mac app is Foxglove Studio 1.84.0. The APT bridge 3.6.0 uses the newer `foxglove.sdk.v1` handshake; the older client's `foxglove.websocket.v1` handshake is rejected. `scripts/build-foxglove-compat.sh` builds official bridge 0.8.5 at a fixed revision in an ignored overlay. The launcher sources that overlay if present, leaving the APT package installed. This is a local compatibility route, not a recommendation to expose the legacy bridge publicly. New Foxglove versions have different licensing/account requirements; do not upgrade or purchase a subscription automatically.

Gazebo 8 uses `~/.gz` for some caches regardless of XDG settings. On this host, the directory created by our first test was moved into `runtime/gazebo-home`, with `~/.gz` retained only as a symlink. The launcher refuses to silently move an unrelated pre-existing cache. ROS logs, Qt/XDG cache and launch temporary files also live under `runtime/`.

The installed Studio1.84 app also failed during Mac startup, independently of bridge compatibility. Current Foxglove3.4.2 is installed separately in the workspace; `open-mac.sh` prefers it (override with `FOXGLOVE_APP`). User sign-in and panel verification remain pending. Do not create an account, activate a subscription or upload data on the user’s behalf.
