""" ASCII stl to BINARY stl"""

import stl
from stl import mesh
import os 
from glob import glob

# for all files in stl folder, convert

directory = 'meshes/stl'

# for file_name in os.scandir(directory):
#     if file_name.is_file():
#         print(file_name.path)

# print("ok")

output = glob("meshes/stl/*.stl")
print(output)

for i, file_name in enumerate(output):
    ascii_mesh = mesh.Mesh.from_file(file_name)

    ascii_mesh.save(file_name, mode=stl.Mode.BINARY)

    print(f"file {file_name} binary convert success")
