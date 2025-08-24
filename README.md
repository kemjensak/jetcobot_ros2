# Installation

```bash
cd ~/colcon_ws/src
git clone https://github.com/kemjensak/jetcobot.git
cd ~/colcon_ws
rosdep install --from-paths src --ignore-src -r -y # or 'rd'
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install # or 'cb'
source ./install/local_setup.bash # or 'sb'
```
