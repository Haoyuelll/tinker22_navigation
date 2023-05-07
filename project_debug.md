# cartographer
## 1. `rosdep update`
### 1.1
```
ERROR: Not all sources were able to be updated.
...
ERROR: unable to process source [https://raw.githubusercontent.com/ros/rosdistro/master/releases/fuerte.yaml]:
	Failed to download target platform data for gbpdistro:
	Remote end closed connection without response
```
##### You may fix this by downloading the source mentioned manually following this: https://zhuanlan.zhihu.com/p/397966333
##### or try packed tool 'rosdepc' with the following commands:
```
pip install rosdepc
sudo rosdepc init
rosdepc update
```
### 1.2
`cartographer: [libabsl-dev] defined as "not available" for OS version [focal]`
You can safely skip this by commenting down the line of `libabsl-dev` in packages.xml

## 2 `google/protobuf`
If you comes across an error suggesting that a file (e.g. port_def.inc) under `google/protobuf`, this is not an error inside the project you are currently working with. **The error arises from a conflict in protoc version in your compiling environment.** 
Use the command `apt search protobuf | grep install` to get your current installed version. A normal output in Ubuntu 20.04 looks like this:
```
libignition-msgs5/focal,now 5.10.0-4~focal amd64 [installed,automatic]
libignition-msgs5-dev/focal,now 5.10.0-4~focal amd64 [installed,automatic]
libprotobuf-dev/focal,now 3.6.1.3-2ubuntu5 amd64 [installed]
libprotobuf-lite17/focal,now 3.6.1.3-2ubuntu5 amd64 [installed]
libprotobuf17/focal,now 3.6.1.3-2ubuntu5 amd64 [installed]
libprotoc-dev/focal,now 3.6.1.3-2ubuntu5 amd64 [installed]
protobuf-compiler/focal,now 3.6.1.3-2ubuntu5 amd64 [installed]
python3-protobuf/focal,now 3.6.1.3-2ubuntu5 amd64 [installed]
```
The current version is 3.6.1 in the sample (3.6 is the highest version available for Ubuntu 20.04), while the file `port_def.inc` only exist after 3.7.

If there is a version like 3.19 exists in your device, it would be highly possible then that there is a fake package in your **conda environment**.
Check if there is a file named `protoc` in `{your dir}/anaconda3/bin`. The problem will be solved by removing this package or simply deleting this file (not recommended).


# moveit
## 1. qhull
Use `qhull -V` to check if you have already installed qhull.
Refers to [here](http://www.qhull.org/)
After downloading the source code, you can see its readme file for installation instruction.
A suggested way is to use Cmake with the following command:
```
cd build
cmake --help               # List build generators
cd .. && cmake -G "<generator>" ..  # e.g., for MINGW-w64 -- cmake -G "MSYS Makefiles"
make
ctest
make install		# If MSYS or UNIX, default CMAKE_INSTALL_PREFIX is '/usr/local'
make uninstall		# Delete the files in install_manifest.txt
```

### 1.1
```
fatal error: libqhull_r.h: No such file or directory
   44 | #include <libqhull_r.h>
```
Use `whereis libqhull_r.h` to find the location of this file:
· the file possibly falls in `/usr/include/libqhull_r`, in this case you can solve it by adding `libqhull_r`before the file name: `#include <libqhull_r/libqhull_r.h>`
· if the file is not found by `whereis`, you may install libqhull manually

### 1.2
`undefined reference to 'uuid_generate@UUID_1.0'`
`undefined reference to 'curl_global_cleanup@CURL_OPENSSL_4'`
ALL these error comes from the conflict between conda env and system lib.
Use the following command to find the file under both dir (e.g. for uuid problem, its package name is libuuid):
`ls -l ~/anaconda/lib | grep libuuid` and `ls -l /lib/x86_64-linux-gnu | grep libuuid`
Caution there are several files among which only one is a solid file and others symbolic links refering to it. For example:
```
-rw-r--r--   1 root root    47314 2月   7  2022 libuuid.a
lrwxrwxrwx   1 root root       38 2月   7  2022 libuuid.so -> /lib/x86_64-linux-gnu/libuuid.so.1.3.0
lrwxrwxrwx   1 root root       16 10月  7 22:15 libuuid.so.1 -> libuuid.so.1.3.0
-rw-r--r--   1 root root    30936 2月   7  2022 libuuid.so.1.3.0
```
the file libuuid.so.1.3.0 is the solid one here.
Remove the solid file in conda lib and use `ln -s /lib/x86_64-linux-gnu/libuuid.so.xxxx ~/anaconda/lib/libuuid.so.xxxx` to make it a symbolic link refering to the file in system lib.
#### But, this error may be reproduced for several times.
#### You can check the output in your initial make (or reproduce it by deleting the devel and build folder and make again). There are warnings of all packages that has an ambiguious reference.
### If your conda env is not heavily relied by other programs, it is highly recommended to remove the whole anaconda folder and remove PATH in .bashrc (or .zshrc etc.), and reinstall `miniconda`. After this you might need to rebuild all workspaces, but be permanently saved from similar errors.


[ERROR] [1669433604.779273585]: Client [/move_base] wants topic /scan_matched_points2 to have datatype/md5sum [sensor_msgs/LaserScan/90c7ef2dc6895d81024acba2ac42f369], but our version has [sensor_msgs/PointCloud2/1158d486dd51d683ce2f1be655c3c181]. Dropping connection.