.. move_bot documentation master file, created by
   sphinx-quickstart on Tue Nov 15 10:08:54 2022.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

MoveBot
=======

A Python API for MoveIt! in ROS2  
--------------------------------

The Movebot package includes a simple_move node that offers a sequence of services and actions that allow the 
Franka Emika robotic arm to plan and execute a trajectory through space while avoiding stationary obstacles. 
Note that the API currently  has joint values specific to the Franka Emilka arm. These can be removed when trying 
to use the API for any other robot model.  

.. toctree::
   :maxdepth: 1
   :caption: Contents:

   movebot
   movebot_interfaces

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
