# IOP Node Manager

[![Build Status](https://travis-ci.org/fkie/iop_node_manager.svg?branch=master)](https://travis-ci.org/fkie/iop_node_manager)

## Summary

The communication between the IOP components occurs via a Node Manager. The Node Manager performs like a router, detects local and remote components and routes the messages to the right destination. In an IOP system one Node Manager should run on each host.

This Node Manager is written in Python and can be used as ROS package or as standalone component as well. It is a part of the [ROS/IOP Bridge](ros_iop_bridge).

## Design

This version of Node Manager is compatible to the `JuniorRTE` of the [JrMiddleware](jrmiddleware) and can be used as an alternative.

In the current version the routing was improved by extended priority handling and use of multiple threads to handle receive, send and queue tasks.

The local communication uses Unix Domain Socket (UDS). Node Manager listen on `/tmp/JuniorRTE` for incoming connection and all other messages. For received messages each component creates it's own socket with integer value of their JAUS id (subsystem.node.component), e.g. `/tmp/6619407` for `101.1.15`.

For remote communication UDP is used. The remote components are discovered on receiving a message with new source address. On routing to unknown destination the message will be send to multicast group with ACK requested. These messages are send with reduced rate. It is also possible to create an address book in configuration file if discovering is not possible.

The default configuration is located in `~/.config/iop.fkie/iop_node_manager.yaml` and is stored in [YAML](yaml) format. If configuration file does not exists a new with default values will be created on next start of Node Manager.

The Node Manager provides also an interface to change logging and statistics at runtime. On enabled statistics all handled messages are written to a file (by default `~/.iop/statistics/last.msgs`). There are also scripts to parse and analyze the statistics at runtime. They show current load or connection state of components.

## Install

Clone this repository to your preferred destination.

- If you use it with ROS put this repository into ROS workspace and call  
`catkin build`

- **Without** ROS support you can use setup.py to install the code:

  ```console
  cd iop_node_manager/fkie_iop_node_manager
  python setup.py install --user --record installed_files.txt
  ```

  The executables are now located in `~/.local/bin`.

  **Note:** to remove installed files call

  ```console
  xargs rm -rf < installed_files.txt
  ```

## Run

Use **rosiopnodemanager.py** to launch Node Manager as a ROS-Node:

```console
rosrun fkie_iop_node_manager rosiopnodemanager.py
```

Or you use **iopnodemanager.py** to start Node Manager as standalone script:

```console
~/.local/bin/iopnodemanager.py
```

### `iopparam.py`

Using this script you can change parameter during the runtime, e.g. change log level or enable/disable statistics output.

```console
rosrun fkie_iop_node_manager iopparam.py --loglevel debug
```

or

```console
rosrun fkie_iop_node_manager iopparam.py --statistic true
```

### `iopeval.py`

Analyse the output located in `~/.iop/statistics/last.msgs` and create statistics specified by parameter.

```console
rosrun fkie_iop_node_manager iopeval.py connections ~/.iop/statistics/last.msgs
```

> Statistics output in Node Manager should be enabled!  

## Notice

Currently communication over TCP is experimental and Serial is under development.

[jrmiddleware]: https://github.com/jaustoolset/jrmiddleware
[yaml]: https://yaml.org
[ros_iop_bridge]: https://github.com/fkie/iop_core
