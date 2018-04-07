#!/usr/bin/env python  
# -*- coding:utf-8 -*-  

import rospy  
import math  
import tf   
  
if __name__ == '__main__':  
    rospy.init_node('py_coordinate_transformation')
#第1部分，定义空间点和空间向量
    print '第1部分，定义空间点和空间向量'
#1.1 返回均匀随机单位四元数
    q=tf.transformations.random_quaternion(rand=None)
    print '定义均匀随机四元数：'
    print q
#1.2 返回均匀随机单位旋转矩阵
    m=tf.transformations.random_rotation_matrix(rand=None)
    print '定义均匀随机单位旋转矩阵：'
    print m
#1.3 返回均匀随机单位向量
    v=tf.transformations.random_vector(3)
    print '定义均匀随机单位向量：'
    print v
#1.4 通过向量来求旋转矩阵
    v_m=tf.transformations.translation_matrix(v)
    print '通过向量来求旋转矩阵：'
    print v_m
#1.5 通过旋转矩阵来求向量
    m_v=tf.transformations.translation_from_matrix(m)
    print '通过旋转矩阵来求向量：'
    print  m_v
#第2部分，定义四元数
    print '第2部分，定义四元数'
#2.1 通过旋转轴和旋转角返回四元数
    axis_q=tf.transformations.quaternion_about_axis(0.123, (1, 0, 0))
    print '通过旋转轴和旋转角返回四元数：'
    print  axis_q
#2.2 返回四元数的共轭
    n_q=tf.transformations.quaternion_conjugate(q)
    print '返回四元数q的共轭：'
    print  n_q
#2.3 从欧拉角和旋转轴，求四元数
    o_q=tf.transformations.quaternion_from_euler(1, 2, 3, 'ryxz')
    print '从欧拉角和旋转轴，求四元数：'
    print  o_q    
#2.4 从旋转矩阵中，返回四元数
    m_q=tf.transformations.quaternion_from_matrix(m)
    print '从旋转矩阵中，返回四元数：'
    print  m_q 
#2.5 两个四元数相乘
    qxq=tf.transformations.quaternion_multiply(q,n_q)
    print '两个四元数相乘'
    print  qxq   
#第3部分，定义欧拉角度
    print '第3部分，定义欧拉角'
#3.1 从欧拉角和旋转轴返回旋转矩阵
    rpy_m=tf.transformations.euler_matrix(1, 2, 3, 'syxz')
    print '从欧拉角和旋转轴返回旋转矩阵'
    print  rpy_m 
#3.2 由旋转矩阵和特定的旋转轴返回欧拉角
    m_rpy=tf.transformations.euler_from_matrix(m)
    print '由旋转矩阵和特定的旋转轴返回欧拉角'
    print  m_rpy 
#3.3 由四元数和特定的轴得到欧拉角
    q_rpy=tf.transformations.euler_from_quaternion(q)
    print '由四元数和特定的轴得到欧拉角'
    print q_rpy



