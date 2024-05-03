# KDTree

Simple C++ static KD-Tree implementation with minimal functionality.

- points are given as STL vectors (and inserted in their own STL vector) so supports n-dimensional points for any n
- makes full trees, (i.e. does not cut-off the branching at some arbitrary level) giving the nearest neighbor query have (strong) logarithmic complexity.
- builds the tree in one go (does not support adding nodes, the tree is built from a list of points and cannot be altered afterwards)
- points are assumed to be STL vectors
- it provides the following queries:
	- nearest neighbor
	- neighbors within a given distance

- 实现思路可以简单理解为通过前序遍历的方式遍历一颗“有序”二叉树（每层以不同的维度上的分量有序）。
	- 计算根节点和目标点的距离，以此更新当前记录的最优距离。
	- 通过在每一层对应维度上的中位数将节点划分成两颗子树，遍历过程中子树的选择依据目标点和中位数的大小关系。
	- 在回溯的过程中，确定是否查看另一边兄弟节点时，可以通过比较当前最短距离和点到当前根对应维度的距离的大小关系，确定是否可以剪枝。

## License and copyright

© J. Frederico Carvalho
Licensed under the [BSD3 License](LICENSE)
