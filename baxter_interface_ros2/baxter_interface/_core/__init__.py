"""Internal plumbing shared by all classes. You don't need to read this to use the library.

    node.py      the shared ROS 2 node, its receiving thread, and how threads are used
    callbacks.py runs each connected function in a thread of its own
    connecting.py setup(): create several robot parts, wait for all at once
    locks.py     one lock per robot part: commands from different threads don't mix
    dataflow.py  wait_for(), wait_for_message(): wait with a timeout
    signals.py   Signal: call your functions when something changes
    services.py  call_service(): call a ROS 2 service and wait for the answer
    images.py    pictures: load, resize, convert to and from ROS 2 messages (OpenCV)
"""
