# tuw_libcamera

ROS 2 nodes for capturing images from a [libcamera](https://libcamera.org/)-supported
camera (e.g. a Raspberry Pi camera module) and republishing them through
`image_transport`. The package provides two composable nodes, meant to run
together inside one `component_container`:

- **`CaptureNode`** (executable `tuw_libcamera_capture`) — drives libcamera
  directly, configures one or more streams, and publishes raw or
  compressed `sensor_msgs/Image` messages plus (optionally) `camera_info`.
- **`TransportNode`** (executable `tuw_libcamera_transport`) — subscribes to
  the raw image published by `CaptureNode` and republishes it through all
  available `image_transport` plugins (e.g. `compressed`, `theora`), so
  downstream tools can pick whichever transport they need.

## Launching

```bash
ros2 launch tuw_libcamera capture.launch.py
```

This starts both nodes inside a `rclcpp_components` container
(`camera_processing_container`), namespaced under `camera`, with intra-process
communication enabled between them. See
[`launch/capture.launch.py`](launch/capture.launch.py) for the exact
configuration.

### Running `CaptureNode` standalone

For debugging it can also be run directly, passing parameters via
`--ros-args -p`. Nested parameters (`streams.<role>.*`) are addressed with
dotted names:

```bash
ros2 run tuw_libcamera tuw_libcamera_capture --ros-args \
  -r __ns:=/camera \
  -p camera_info_name:=camera \
  -p stream_roles:="[video]" \
  -p streams.video.format:=YUYV \
  -p streams.video.target_format:=yuyv \
  -p streams.video.width:=1536 \
  -p streams.video.height:=864
```

## CaptureNode

### Topics published

| Topic | Type | Condition |
|---|---|---|
| `<sub_topic>/image` (default `image`) | `sensor_msgs/msg/Image` | one per configured stream role, if the resolved format maps to a raw ROS encoding |
| `<sub_topic>/image/compressed` | `sensor_msgs/msg/CompressedImage` | one per configured stream role, if the resolved format maps to a compressed encoding (e.g. MJPEG → `jpeg`) |
| `camera_info` | `sensor_msgs/msg/CameraInfo` | only if `camera_info_name` is non-empty |

### Parameters

Defined via `generate_parameter_library` in
[`capture_node_parameters.yaml`](src/capture/capture_node_parameters.yaml).
All parameters below are `read_only` (set at startup, not runtime-changeable).

| Parameter | Type | Default | Description |
|---|---|---|---|
| `camera` | string | `""` | libcamera camera ID to use. Empty selects the first camera libcamera enumerates. |
| `camera_info_name` | string | `""` | Camera name used to look up/publish `camera_info`. Empty disables the `camera_info` topic entirely. |
| `camera_info_url` | string | `""` | URL of the camera calibration file. Empty uses camera_info_manager's default (`file:///<home>/.ros/camera_info/<camera_info_name>.yaml`). |
| `frame_id` | string | `""` | `frame_id` set in the header of published image/camera_info messages. |
| `stream_roles` | string array | `["video"]` | Which libcamera stream roles to configure and publish. Must be non-empty, unique, and a subset of `raw`, `viewfinder`, `still`, `video`. Each entry gets a corresponding `streams.<role>` group and its own topic. |
| `streams.<role>.width` | int | `0` | Requested stream width in pixels. `0` leaves it to libcamera's default for the role. |
| `streams.<role>.height` | int | `0` | Requested stream height in pixels. `0` leaves it to libcamera's default for the role. |
| `streams.<role>.format` | string | `""` | Requested libcamera pixel format (e.g. `YUYV`, `RGB888`, `MJPEG`), parsed via `libcamera::PixelFormat::fromString`. Empty leaves it to libcamera's default for the role. |
| `streams.<role>.sub_topic` | string | `""` | Topic prefix for this stream's image topic(s), e.g. `video` → `video/image`. Empty publishes directly on `image`. |
| `streams.<role>.target_format` | string | `""` | Which ROS encoding to convert the captured pixel format into (see [Supported pixel/target formats](#supported-pixeltarget-formats) below). Empty picks the first match for the pixel format. |

`<role>` is one of the entries listed in `stream_roles` (e.g. `streams.video.width`).

If the requested stream configuration is invalid or can't be matched to a
known ROS encoding, the node logs the pixel formats libcamera actually
supports for that role, and/or the ROS target formats available for the
requested pixel format, before throwing at startup.

> **Tip:** to find out which pixel formats and resolutions your camera
> actually supports before setting `streams.<role>.format`/`width`/`height`,
> use `rpicam-hello --list-cameras` (see
> [`docs/raspberry_pi.md`](../../../../docs/raspberry_pi.md)) or
> `v4l2-ctl --list-formats-ext -d <device>`. Both list the sensor's native
> modes independently of this node.

#### Camera controls (`controls.*`)

In addition to the static parameters above, `CaptureNode` inspects the
camera's supported libcamera controls at startup (e.g. `AeEnable`,
`ExposureTime`, `AnalogueGain`, `Brightness`, `Contrast`, `Saturation`,
`AwbMode`, ...) and declares one ROS parameter per control, named
`controls.<ControlName>`, seeded with that control's libcamera default.
These *are* settable at runtime via `ros2 param set` and take effect on the
next capture request. The exact set of available controls depends on the
camera/sensor; run the node and check its startup log
(`Camera supported control: ...`) or `ros2 param list` for the definitive
list on your hardware.

Array-valued controls (e.g. `AfWindows`, `FrameDurationLimits`) are not
currently supported and are skipped (logged as a warning) rather than
exposed as parameters.

### Supported pixel/target formats

The mapping between libcamera pixel formats and ROS image encodings is
defined in [`convert.cpp`](src/capture/convert.cpp):

| libcamera `streams.<role>.format` | ROS `streams.<role>.target_format` | Message type |
|---|---|---|
| `R8` | `mono8` | `Image` |
| `R16` | `mono16` | `Image` |
| `RGB888` | `bgr8` | `Image` |
| `BGR888` | `rgb8` | `Image` |
| `ABGR8888` | `rgba8` | `Image` |
| `ARGB8888` | `bgra16` | `Image` |
| `RGB161616` | `bgr16` | `Image` |
| `BGR161616` | `rgb16` | `Image` |
| `MJPEG` | `bgr8` (decoded) | `Image` |
| `MJPEG` | `jpeg` (passthrough) | `CompressedImage` |
| `UYVY` | `yuv422` | `Image` |
| `YUYV` | `yuyv` | `Image` |

A given `format` may map to more than one `target_format` (see `MJPEG`
above); `target_format` disambiguates which one to use. Leaving
`target_format` empty picks the first matching entry for that pixel format.

> **Note:** the ROS image encoding `yuv422_yuy2` is deprecated (see
> `sensor_msgs/image_encodings.hpp`) and is no longer emitted by this
> package; use `yuyv` instead as the `target_format` for the `YUYV` pixel
> format above.

## TransportNode

Subscribes to the `image` topic (in its namespace) and republishes it
through every discovered `image_transport` publisher plugin other than
`raw` (since `CaptureNode` already publishes the raw image directly). The
set of enabled plugins is exposed as the standard `image_transport`
parameter `<resolved topic, dots for slashes>.enable_pub_plugins`, e.g. for
the default `capture.launch.py` setup this is `image.enable_pub_plugins`.
