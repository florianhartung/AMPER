import os

file_path = os.path.dirname(os.path.dirname(__file__)) # == src/AMPER
hpp_path = os.path.join(file_path, 'src/controller/const_labyrinth.hpp') # == src/AMPER/src/controller/const_labyrinth.hpp
world_path = os.path.join(file_path, 'urdf/labyrinth.xml')

array = [
  [False, False, False, False, False, False, False, False, False, False, False, False],
  [False, True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  False],
  [False, True,  False, False, False, False, True,  True,  False, False, True,  False],
  [False, True,  True,  True,  True,  False, True,  True,  True,  False, True,  False],
  [False, True,  True,  False, True,  False, True,  False, False, False, True,  False],
  [False, True,  False, False, True,  False, True,  True,  True,  True, True,   False],
  [False, False, False, False, True,  True,  True,  True,  True,  False, True,  False],
  [False, True,  True,  False, True,  False, False, False, True,  False, True,  False],
  [False, True,  True,  True,  True,  True,  True,  True,  False, False, False, False],
  [False, False, False, True,  True,  True,  True,  True,  False, True,  True,  False],
  [False, True,  True,  True,  True,  False, True,  True,  False, False, True,  False],
  [False, False, False, False, False, False, False, False, False, False, False, False]]


#######################################################################################################

header = """#pragma once
#include <array>
#include "labyrinth.hpp"

const static Labyrinth<%d> LABYRINTH ({

""" % len(array)

for line in array:
  header += "\t"
  for i, element in enumerate(line):
      if element is False:
        header += "false, "
      else:
        header += "true, "
  header += "\n"      

header = header[:-3] + "\n});\n" # header[-3] to remove last comma and line break


with open(hpp_path, 'w') as f1:
  f1.write(header)


head = """<sdf version='1.7'>
  <world name='labyrinth'>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>200 200</size>
            </plane>
          </geometry>
          <surface>
            <contact>
              <collide_bitmask>65535</collide_bitmask>
              <ode/>
            </contact>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>200 200</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>
    <wind/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
"""

middle = """<state world_name='labyrinth'>
      <sim_time>1887 266000000</sim_time>
      <real_time>2747 188788856</real_time>
      <wall_time>1718021803 799918533</wall_time>
      <iterations>1887266</iterations>
      <model name='ground_plane'>
        <pose>0 0 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>0 0 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        </model>
        """
    
tail = """<light name='sun'>
        <pose>0 0 10 0 -0 0</pose>
      </light>
    </state>
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>8.02571 0.844781 7.89325 0 0.633796 -2.85899</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
  </world>
</sdf>"""

k = 0
x = 0.5
y = 0.5
middle1 = ""
middle2 = ""
for i, line in enumerate(array):
    for j, element in enumerate(line):
        if element is False:
          middle1 += f"""\n<model name='unit_box_{k}'>
          <pose>{x + j} {y + i} 0.5 0 -0 0</pose>
          <link name='link'>
            <inertial>
              <mass>1</mass>
              <inertia>
                <ixx>0.166667</ixx>
                <ixy>0</ixy>
                <ixz>0</ixz>
                <iyy>0.166667</iyy>
                <iyz>0</iyz>
                <izz>0.166667</izz>
                </inertia>
                <pose>0 0 0 0 -0 0</pose>
              </inertial>
              <collision name='collision'>
                <geometry>
                  <box>
                    <size>1 1 1</size>
                  </box>
                </geometry>
                <max_contacts>10</max_contacts>
                <surface>
                  <contact>
                    <ode/>
                  </contact>
                  <bounce/>
                  <friction>
                    <torsional>
                      <ode/>
                    </torsional>
                    <ode/>
                  </friction>
                </surface>
              </collision>
              <visual name='visual'>
                <geometry>
                  <box>
                    <size>1 1 1</size>
                  </box>
                </geometry>
                <material>
                  <script>
                    <name>Gazebo/Grey</name>
                    <uri>file://media/materials/scripts/gazebo.material</uri>
                  </script>
                </material>
              </visual>
              <self_collide>0</self_collide>
              <enable_wind>0</enable_wind>
              <kinematic>0</kinematic>
            </link>
          </model>\n"""
          middle2 += f"""\n<model name='unit_box_{k}'>
          <pose>{x + j} {y + i} 0.5 0 -0 0</pose>
          <scale>1 1 1</scale>
          <link name='link'>
              <pose>{x + j} {y + i} 0.5 0 -0 0</pose>
              <velocity>0 0 0 0 -0 0</velocity>
              <acceleration>-0.004709 -9.78112 9.78158 0.712677 -0.009414 -4.3e-05</acceleration>
              <wrench>-0.004709 -9.78112 9.78158 0 -0 0</wrench>
          </link>
          </model>\n"""
          k += 1

with open(world_path, 'w') as f2:
  f2.write(head+middle1+middle+middle2+tail)
