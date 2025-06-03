URDF File from
https://github.com/Daniella1/urdf_files_dataset/blob/main/urdf_files/ros-industrial/xacro_generated/universal_robots/ur_description/urdf/ur3e.urdf


change to proto
https://github.com/cyberbotics/urdf2webots/blob/master/docs/tutorial.md

pip install --no-cache-dir --upgrade urdf2webots
python -m urdf2webots.importer --input=ur3e.urdf --box-collision --tool-slot=tool0
