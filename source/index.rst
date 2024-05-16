.. assignment_2_2023-main documentation master file, created by
   sphinx-quickstart on Mon May  6 13:15:40 2024.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to assignment_2_2023-main's documentation!
====================================================

1. Action Client Node: This node allows the user to set a target position (x, y) for the robot to reach or cancel an existing target. It communicates with an action server to send goals and receive feedback/status about the target reaching process. Additionally, it publishes information about the robot's position and velocity (x, y, vel_x, vel_z) using a custom message, based on data received from the /odom topic.

2. Service Node for Last Target: When called, this service node returns the coordinates (x, y) of the last target position that the user sent to the robot. It stores and retrieves this information upon request.

3. Service Node for Position and Velocity: This service node subscribes to the robot's position and velocity data, provided in a custom message format. It implements a service server to calculate and provide information about the distance of the robot from the target and the robot's average speed. This information is retrieved upon request.

.. toctree::
   :maxdepth: 2
   :caption: Contents:



Indices and tables
====================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

assignment_2_2023-main's documentation!
****************************************
This is the documentation of the assignment_2_2023-main package!

**Action Client**
==============================================
.. automodule:: scripts.action_client
  :members:
  
**Last Target Service**
===============================================
.. automodule:: scripts.last_target_service
   :members:
  
**Average Service**
===============================================
.. automodule:: scripts.avg_service
   :members:
