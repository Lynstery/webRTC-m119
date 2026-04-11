# WebRTC-based peer-to-peer video streaming research platform

This repository contains a peer-to-peer video streaming platform built on **WebRTC branch-heads/6045 (M119)**, with a **fine-grained logging system** for academic research.

> This codebase is intended for **personal academic research on video streaming systems only**.

---

## Build and Run

### Prerequisites

Before building, add `depot_tools` to your environment variables and enter the project directory:

```bash
export PATH=/path/to/depot_tools:$PATH
cd webrtc-m119
```
Please check out depot_tools at the following commit: c2e00617237c90aca99868c57425aa046229e57b

### Dependencies

Install the required libraries as follows:

```base
cd third_party/gf-complete-master
./autogen.sh
./configure
make
sudo make install

cd ../Jerasure-master
autoreconf --force --install
./configure
make
sudo make install

sudo sh -c 'echo "/usr/local/lib" > /etc/ld.so.conf.d/usr-local-lib.conf'
sudo ldconfig

# Check whether /usr/local/lib has been registered successfully
ldconfig -p | grep /usr/local/lib
```

### First-Time Build Configuration

For the first build, run:

```bash
gn args out/Exp0 --export-compile-commands
```

A configuration editor will pop up. Use the following build arguments:
```
use_custom_libcxx = false
use_custom_libunwind = false

rtc_include_tests = false
rtc_build_examples = false

# video
rtc_use_h264 = true
ffmpeg_branding = "Chrome"

# debug
is_debug = true
symbol_level = 2
enable_iterator_debugging = true
use_lld = true
use_rtti = true
treat_warnings_as_errors = false
rtc_disable_logging = false

# system
target_os = "linux"
target_cpu = "x64"
```
For ARM cross-compilation, change:
```
target_cpu = "arm"
```
Save and exit after editing. This step is only required once. You do not need to repeat it for subsequent builds.

### Compile
```
ninja -C out/Exp0
```
After compilation, the following executables will be generated under out/Exp0:
- video_streaming_signal_server
- video_streaming_client
- video_streaming_client_headless

### Usage
1. Start the signaling server

```
./out/Exp0/video_streaming_signal_server
```

2. Start the receiver client

```
./out/Exp0/video_streaming_client_headless --server=<signal-server-ip> \
  --video=/path/to/yuvs/%06d.yuv \
  --end_index=1000 \
  --width=1280 \
  --height=720 \
  --fps=60 \
  --trace_file=/path/to/trace_receiver.json
```

3. Start the sender client

```
./out/Exp0/video_streaming_client_headless --server=<signal-server-ip> --autocall \
  --video=/path/to/yuvs/%06d.yuv \
  --end_index=1000 \
  --width=1280 \
  --height=720 \
  --fps=60 \
  --trace_file=/path/to/trace_sender.json
```

Notes:
- If `--autocall` is not specified, you need to manually select another client connected to the same signaling server from the GUI to start streaming.
- If `--autocall` is specified, the client will automatically connect to another already connected client. Therefore, make sure the receiver has already started.
- video_streaming_client_headless is the headless version of the client, suitable for SSH environments.
- Since the headless client has no GUI, the sender must use `--autocall`.

After running the experiment, analyze the collected traces with:
```
python scripts/analysis.py trace_sender.json trace_receiver.json result_save_path
```
