IOP Node Manager
================

Summary
-------

The communication between the IOP components occurs via a Node Manager. The Node Manager performs like a router, detects local and remote components and routes the messages to the right destination. In an IOP system one Node Manager should run on each host.

This Node Manager is written in Python and can be used as ROS package or as standalone component as well. It is a part of the [ROS/IOP Bridge](ros_iop_bridge).

Design
------

This version of Node Manager is compatible to the `JuniorRTE` of the [JrMiddleware](jrmiddleware) and can be used as an alternative.

In the current version the routing was improved by extended priority handling and use of multiple threads to handle receive, send and queue tasks.

The local communication uses Unix Domain Socket (UDS). Node Manager listen on `/tmp/JuniorRTE` for incomming connection and all other messages. For received messages each component creates it's own socket with integer value of their JAUS id (subsystem.node.component), e.g. `/tmp/6619407` for `101.1.15`.

For remote communication UDP is used. The remote components are discovered on receiving a message with new source address. On routing to unknown destination the message will be send to multicast group with ACK requested. These messages are send with reduced rate. It is also possible to create an address book in configuation file if discovering is not possible.

The default configuration is located in `~/.config/iop.fkie/iop_node_manager.yaml` and is stored in [YAML](yaml) format. If configuration file does not exists a new with default values will be created on next start of Node Manager.

The Node Manager provides also an interface to change logging and statistics at runtime. On enbaled statistics all handled messages are written to a file (by default `~/.iop/statistics/last.msgs`). There are also scripts to parse and analyse the statistics at runtime. They show current load or connection state of components.

Notice
------

Currently communication over TCP is experimental and Serial is under development.


[jrmiddleware]: https://github.com/jaustoolset/jrmiddleware
[yaml]: https://yaml.org/
[ros_iop_bridge]: https://github.com/fkie/iop_core