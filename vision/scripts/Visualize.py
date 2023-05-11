import open3d as o3d
# with open('C:/Users/Administrator/Desktop/true_point_cloud.txt') as f:
#     # contents = f.read()
with open('c:/Users/Administrator/Desktop/point_cloud.txt', 'r') as file:
    data = file.read()

with open('true_point_cloud.txt', 'w') as fp:
    fp.write('\n'.join(f'{tup[0]} {tup[1]} {tup[2]}' for tup in eval(data)))
    fp.close()
pcd = o3d.io.read_point_cloud('C:/Users/Administrator/Desktop/true_point_cloud.txt',format('xyz'))
print(pcd)
o3d.visualization.draw_geometries([pcd])