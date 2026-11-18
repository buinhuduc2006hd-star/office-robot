from setuptools import setup
from glob import glob

setup(name="office_robot_edge_gateway", version="0.1.0", packages=["office_robot_edge_gateway"],
      data_files=[("share/ament_index/resource_index/packages", ["resource/office_robot_edge_gateway"]),
                  ("share/office_robot_edge_gateway", ["package.xml"]),
                  ("share/office_robot_edge_gateway/launch", glob("launch/*.launch.py"))],
      entry_points={"console_scripts": ["gateway = office_robot_edge_gateway.node:main"]})
