# SLAM Box

## 1. 纯ndt里程计
基于手写的多线程ndt匹配算法实现简单的里程计
<table>
  <thead>
    <tr>
      <th colspan="6">On KITTI Datasets</th>
    </tr>
  </thead>
  <!-- <tbody>
    <tr align="center">
      <td> 00 </td><td> 01 </td><td> 02 </td><td> 03 </td><td> 05 </td><td> 04 </td>
    </tr>
  </tbody> -->
  <tbody>
    <tr>
      <td> 05 </td>
      <td> <p align="center"><img src="data/ndt_lo/05/map.png" alt="animated" height="200" /></p> </td>
      <td> <p align="center"><img src="data/ndt_lo/05/pose.png" alt="animated" height="200" /></p> </td>
      <td> <p align="center"><img src="data/ndt_lo/05/ape_pose.png" alt="animated" height="200" /></p> </td>
    </tr>
  </tbody>
</table>

## 2. 基于ESKF的松耦合里程计

<table>
  <thead>
    <tr>
      <th colspan="6">Test Results On Datasets</th>
    </tr>
  </thead>
  <!-- <tbody>
    <tr align="center">
      <td> 00 </td><td> 01 </td><td> 02 </td><td> 03 </td><td> 05 </td><td> 04 </td>
    </tr>
  </tbody> -->
  <tbody>
    <tr>
      <th rowspan="3" scope="rowgroup">ulhk</th>
      <th scope="row">ndt_lo</th>
      <td> <p align="center"><img src="data/ndt_lo/ulhk/global_map.png" alt="animated" height="200" /></p> </td>
      <td> <p align="center"><img src="data/ndt_lo/ulhk/drift.png" alt="animated" height="150" /></p> </td>
    </tr>
    <tr>
      <th scope="row">loosely_lio</th>
      <td> <p align="center"><img src="data/loose_lio/ulhk/global_map.png" alt="animated" height="200" /></p> </td>
      <td> <p align="center"><img src="data/loose_lio/ulhk/drift.png" alt="animated" height="200" /></p> </td>
    </tr>
    <tr>
      <th scope="row">dlo</th>
      <td> <p align="center"><img src="data/dlo/ulhk.png" alt="animated" height="200" /></p> </td>
      <td> <p align="center"><img src="data/dlo/little_drift.png" alt="animated" height="150" /></p> </td>
    </tr>
  </tbody>
</table>

## Reference

**direct_lidar_odometry (dlo)**: [**Code**: https://github.com/vectr-ucla/direct_lidar_odometry](https://github.com/vectr-ucla/direct_lidar_odometry)