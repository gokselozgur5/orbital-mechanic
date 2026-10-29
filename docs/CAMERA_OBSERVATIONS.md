# Bounded camera evidence for machine observations

OM-048 extends the live [machine observation reader](INSPECTION_OBSERVATION.md)
with optional local camera evidence. It uses the existing camera and bridge;
there is no renderer, image-generation model, vision diagnosis or new ROS action.
Local software acceptance is separate from native camera commissioning.

After commissioning the owned runtime and sourcing ROS:

```bash
ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  python3 scripts/machine-observation.py pump --with-camera --timeout 5
```

`cooler` and `power_unit` are also allowed. The normal finite 0.1–10 second
timeout and localhost domain rules remain. There is no output-path, input-image,
calibration, topic or tolerance override. Without `--with-camera`, the existing
schema-1 observation is unchanged and creates no capture bundle. The library
entry point for camera capture is `capture_machine_observation`.

## Source, pixels and calibration

`CameraContract` reads the registry-bound SDF and the source URDF/bridge bytes.
It requires one supported 5 Hz RGB camera, the configured image/info topics and
`camera_optical_frame`. The SDF body/sensor poses must agree with the fixed URDF
body, camera and optical mounts. The optical rotation and measured TF must agree
with those sources within 1e-5 in translation/rotation-matrix components. This is
a source-consistency tolerance, not measured physical camera calibration.

The current source is 640×480 `R8G8B8`. The bridge's RGB conversion produces
packed ROS `rgb8` data. The reader requires the configured dimensions, three
bytes per pixel, exact row stride, little-endian flag zero and exactly the
declared byte count. It supports source dimensions up to 1024×768 only when the
payload fits the fixed 1 MiB cap. Layout and buffer size are checked before
copying bytes. This bounds application copying; ROS/DDS already allocated the
incoming message. The reader keeps at most two received images and two infos,
plus one frozen selected pair. See the upstream [Image contract](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/Image.msg)
and [bridge conversion](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/src/convert/sensor_msgs.cpp).

Image and CameraInfo must have identical positive acquisition stamps and the
configured optical frame. Each stream must advance; duplicates, regressions,
future/pre-admission stamps and invalid input remain fatal for the query.
CameraInfo dimensions must match the image. Finite bounded K/R/P/D arrays must
describe the configured undistorted pinhole camera: focal lengths within 0.1%
of the SDF horizontal FOV calculation, principal point within one pixel of the
center, identity rectification and matching monocular projection. Zero distortion,
no subsampling and a full-frame ROI are required. Empty/default distortion and
full-frame ROI representations are supported. A calibration change within the
query is rejected. The meanings of these fields follow the [CameraInfo contract](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/CameraInfo.msg);
these local checks do not prove the installed renderer or bridge matches them.

The unique image/info publishers must both be the expected `ros_gz_bridge`, with
their exact types, namespaces and nonzero endpoint GIDs. Their sampled identities
join the existing clock, telemetry, map, health, odometry and TF checks. Image
subscription uses reliable depth one, matching the project's earlier native
transport configuration; info uses the sensor-data profile. Native delivery and
timing still require commissioning.

## Two real acquisition times

The public model produces telemetry at 2 Hz while the camera produces images at
5 Hz. Their timestamps are preserved independently. A selected image/info pair
must be at or after the pinned robot-settling stamp and within ±100 ms of the
selected telemetry. The signed difference is included in the record; the two
samples are never described as simultaneous unless their stamps actually match.

The existing telemetry-time TF and settled pose remain required. The observer
also requests exact-time `map` to `base_footprint` and `camera_optical_frame` TF
at the image timestamp. That robot pose must independently meet the inspection
position/heading/stationarity gates, and the optical TF must match its configured
mount. There is no latest-TF or fabricated camera-pose fallback.

Both selected image and info must remain at most 200 ms old in receipt time;
image simulation age is also at most 200 ms. The existing telemetry/robot/map
bounds still apply. The selected frame is immutable: later valid frames may
arrive but do not replace it, while invalid input or changed calibration rejects
the query. Missing pairs or transforms can wait only inside the original sample
lifetime and overall deadline. Motion after settling, even during file storage,
invalidates the record.

## Record and local storage

Successful camera capture returns outer schema 2. The schema-1 public machine
and telemetry-time robot fields remain, with a `camera` object containing:

- Actual image stamp, optical frame, RGB dimensions/stride and signed telemetry
  offset with its fixed limit.
- Image/info receipt ages, image simulation age and validated CameraInfo values.
- Image-time robot state and the measured optical camera pose.
- URDF/bridge source hashes; the outer record retains world/map identities.
- Image filename, media type, byte count, whole-file SHA-256 and pixel SHA-256.

The ignored directory `runtime/inspection/<observation_id>/` holds `image.ppm`
and `observation.json`. P6 PPM stores the RGB bytes losslessly with a short
header; no image library, conversion, crop or annotation changes the pixels.
The CLI prints metadata and references rather than image payloads. Source Git
receives neither captures nor recordings.

Directories/files use owner-only modes, directory-relative descriptors,
no-follow opens and exclusive creation. A nonblocking writer lock serializes
quota admission. At most 64 capture entries, including incomplete attempts, are
allowed. Each accepted image is at most 1 MiB plus its header and each manifest
at most 64 KiB. Full storage refuses new captures; it never silently deletes old
evidence or overwrites an observation ID. Operators must review and archive
retained captures before making space.

The writer computes identities, writes/syncs the image and pending manifest,
reads back their exact bytes, and checks live validity before publishing the
manifest. It checks directory identity, bytes and live validity again afterward.
Managed failure or interruption removes only its own accepted-manifest link;
bounded partial evidence may remain. A regression reproduces stale acceptance
when manifest hashing followed the last freshness check in the initial new
writer; preparing that digest before the checks rejects the identical case.

`capture_artifact.manifest_sha256` is returned outside the stored manifest to
avoid a circular self-hash. `read_capture(root, observation_id, digest)` verifies
the retained manifest and image against that returned reference using bounded,
no-follow reads. It does not renew freshness, establish authenticity or authorize
an action. A file found on disk alone is not an inspection endpoint's successful
terminal result. Abrupt process/power loss and ownership recovery require the
future endpoint protocol; managed cleanup is not a crash-atomicity claim.

Exit 0 requires the complete capture; evidence/storage failure returns 2 with
invalid JSON and no partial measurements. Ctrl-C returns 130. Node/listener/ROS
cleanup still runs if storage fails. The timeout bounds accepted results, not
hard real-time completion of a blocked kernel or runtime call. Reported ages are
those at measurement construction; validity is rechecked through file publication.

## Validation and remaining work

Tests execute production source parsing, evidence selection, the actual combined
reader/CLI and filesystem writer/reader. Controlled transport covers all three
machines, both timestamp orderings, source/calibration/TF mismatch, missing and
malformed data, bounded allocation, fixed selection, expiry/motion during writes,
post-publication tampering, directory replacement, interruption and cleanup.
Filesystem cases include symlinks, hardlinks, special files, duplicate IDs,
exclusive writers, quota and later digest verification. Synthetic RGB bytes in
these tests do not establish rendered visibility or image-based diagnosis.

A separate real-monotonic-clock paced transport probe accepts the complete local
capture and rejects an injected 250 ms storage delay with no accepted manifest.
This exercises actual local file writes and stopped-history expiry; it is not
native DDS or a filesystem-performance guarantee.

Before native acceptance, recheck Thor ownership, exact sources and pending
map/localization/navigation commissioning. Verify actual image/info stamps,
calibration, QoS, TF mounts and temporal bounds first on an empty isolated
domain, then at each independently verified machine arrival. Record actual
rendered evidence without hidden scenario labels. Preserve the trial guardian
for armed movement. Full cancellable inspection ownership and native three-machine
acceptance remain open under issue #8.
