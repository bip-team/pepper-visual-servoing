initial setup
=============

    1) mkdir /path/to/qibuild-workspace/
    2) cd /path/to/qibuild-workspace/; qibuild init
    3) cp -R /path/to/remote-sdk /path/to/qibuild-workspace/remote-sdk
    4) qitoolchain create mytoolchain-remote /path/to/qibuild-workspace/sdk/toolchain.xml
    5) qibuild add-config mytoolchain-remote -t mytoolchain-remote
    6) cd /path/to/qibuild-workspace/; git clone <pepper-vs>


compilation
===========

    1) qibuild configure -c mytoolchain-remote 
    2) qibuild make -c mytoolchain-remote

It's enough to type '`make TC=name-of-my-toolchain`' inside the root directory.
If your toolchain is called 'mytoolchain-remote', it's enough to just type '`make`'.

possible issues
===============

Similar issues rooted in mixing ViSP with naoqi toolchain may arise as described in http://jokla.me/robotics/visp_naoqi/.

Possible workarounds:

1) In case of conflicting libraries in /path/to/sdk/lib/, remove the problematic libraries from
the toolchain (rm libz and libusb-1.0.so.0).

2) Conflicts related to boost libs need visp to be rebuilt without OGRE, (cmake -DUSE_OGRE=OFF /path/to/visp).
