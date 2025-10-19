## Week 11  Status Update: Isaac Sim Project Remains Unresolved


I had a crush on Isaac Sim, and this solved the device full problem:
```
sudo sysctl fs.inotify.max_user_watches=524288
echo "fs.inotify.max_user_watches=524288" | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

This one for setting the terminal:

```
cd ~/{isaacsimfolder}
conda activate {envname}
source /opt/ros/{ros-distro}/setup.bash
source setup_conda_env.sh
source ~/{ros workspace}/install/setup.bash 
```

This one's for checking if we have any problems with libraries:
```
echo "PyTorch: $(python -c 'import torch; print(torch.__version__)')"
 echo "NumPy: $(python -c 'import numpy; print(numpy.__version__)')"
echo "Python: $(which python)"

```

Omnigraph is currently not functioning due to some deprecated nodes, but I am working on fixing it.

I am facing an issue with my driver, and although I plan to resolve it, it looks like it will take longer than I initially expected. My lessons have made it a little harder to focus on this journey, but I am determined to continue. I look forward to seeing you next week.