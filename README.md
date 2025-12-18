# MYNT EYE SDK + ROS 2 Node (Build Sources)

This repository contains:
- **MYNT EYE SDK sources** (used to build and install the native SDK and run sample applications)
- **ROS 2 package (node)** for interacting with MYNT EYE cameras from ROS 2

The instructions below describe a typical **Ubuntu / Linux** workflow where **OpenCV 3.4.3** is built from sources and installed into `/opt`, then the **MYNT EYE SDK** is built and installed, and finally the **ROS 2 node** is built in a ROS 2 workspace.

---

## 1. Prerequisites

- Build tools: `gcc/g++`, `cmake`, `make`
- A working **ROS 2** installation (e.g., Humble) and a ROS 2 workspace
- MYNT EYE camera connected via USB

Recommended:
- Use a clean workspace and ensure you do not mix multiple OpenCV versions unintentionally.

---

## 2. Build and Install OpenCV 3.4.3 (from sources)

> This step installs OpenCV into `/opt/opencv-3.4.3` and registers its libraries in the system dynamic linker cache.

From the OpenCV source directory:

```bash
mkdir -p _build
cd _build

cmake .. \
  -DCMAKE_BUILD_TYPE=RELEASE \
  -DCMAKE_INSTALL_PREFIX=/opt/opencv-3.4.3 \
  -DWITH_CUDA=OFF \
  -DBUILD_DOCS=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_TESTS=OFF \
  -DBUILD_PERF_TESTS=OFF

make -j$(nproc)
sudo make install

echo /opt/opencv-3.4.3/lib | sudo tee /etc/ld.so.conf.d/opencv-3.4.3.conf
sudo ldconfig

