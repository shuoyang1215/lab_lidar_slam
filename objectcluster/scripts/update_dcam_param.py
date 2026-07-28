#!/usr/bin/env python
#coding=utf-8
'''
readme:
run the code to update param of dcam
open a terminal in /objectcluster/scripts

run:
python update_dcam_param.py

note:python should be 2.x
after update the param, use ctrl+c to exit the program
'''

import rospy
from sensor_msgs.msg import CameraInfo
import yaml

#回调函数输入的应该是msg
def callback(data):
    dfx = data.K[0]
    dfy = data.K[4]
    dcx = data.K[2]
    dcy = data.K[5]

    bbox_yaml_dir = '../../bboxmerge/config/bbox_merge.yaml'
    obj_yaml_dir = '../../objectcluster/config/objseg_voxelgrid.yaml'
    slope_yaml_dir = '../../slopeSeg/config/DCameraBackUp.yaml'
    segfusion_yaml_dir = '../../segfusion/config/segfusion.yaml'
    slope_yaml_dir_abs = '/home/nc/ab_ws/src/ObjDetMerge/slopeSeg/config/DCameraBackUp.yaml'

    # bbox
    with open(bbox_yaml_dir,'r') as f:
        result = f.read()
        x = yaml.load(result, Loader=yaml.FullLoader)
        flag = 0
        if x['bbox_merge']['fx_dc'] != dfx:
            print('bbox_old, dfx:'+str(x['bbox_merge']['fx_dc'])+' dfy:'+str(x['bbox_merge']['fy_dc'])+' dcx:'+str(x['bbox_merge']['cx_dc'])+' dcy:'+str(x['bbox_merge']['cy_dc']))
            flag = 1
        
        # 修改的值
        x['bbox_merge']['fx_dc'] = dfx
        x['bbox_merge']['fy_dc'] = dfy
        x['bbox_merge']['cx_dc'] = dcx
        x['bbox_merge']['cy_dc'] = dcy
        if flag == 1:
            print('bbox update successfully, dfx:'+str(x['bbox_merge']['fx_dc'])+' dfy:'+str(x['bbox_merge']['fy_dc'])+' dcx:'+str(x['bbox_merge']['cx_dc'])+' dcy:'+str(x['bbox_merge']['cy_dc']))

        with open(bbox_yaml_dir,'w') as w_f:
            # 覆盖原先的配置文件
            yaml.dump(x,w_f)


    # obj
    with open(obj_yaml_dir,'r') as f:
        result = f.read()
        x = yaml.load(result, Loader=yaml.FullLoader)
        flag = 0
        if x['objseg_gridclass']['dfx'] != dfx:
            print('obj_old, dfx:'+str(x['objseg_gridclass']['dfx'])+' dfy:'+str(x['objseg_gridclass']['dfy'])+' dcx:'+str(x['objseg_gridclass']['dcx'])+' dcy:'+str(x['objseg_gridclass']['dcy']))
            flag = 1

        # 修改的值
        x['objseg_gridclass']['dfx'] = dfx
        x['objseg_gridclass']['dfy'] = dfy
        x['objseg_gridclass']['dcx'] = dcx
        x['objseg_gridclass']['dcy'] = dcy
        if flag == 1:
            print('obj update successfully, dfx:'+str(x['objseg_gridclass']['dfx'])+' dfy:'+str(x['objseg_gridclass']['dfy'])+' dcx:'+str(x['objseg_gridclass']['dcx'])+' dcy:'+str(x['objseg_gridclass']['dcy']))

        with open(obj_yaml_dir,'w') as w_f:
            # 覆盖原先的配置文件
            yaml.dump(x,w_f)


    # slope
    with open(slope_yaml_dir,'r') as f:
        result = f.read()
        x = yaml.load(result, Loader=yaml.FullLoader)
        flag = 0
        if x['slopeSegDCamera']['dfx'] != dfx:
            print('slope_old, dfx:'+str(x['slopeSegDCamera']['dfx'])+' dfy:'+str(x['slopeSegDCamera']['dfy'])+' dcx:'+str(x['slopeSegDCamera']['dcx'])+' dcy:'+str(x['slopeSegDCamera']['dcy']))
            flag = 1

        # 修改的值
        x['slopeSegDCamera']['dfx'] = dfx
        x['slopeSegDCamera']['dfy'] = dfy
        x['slopeSegDCamera']['dcx'] = dcx
        x['slopeSegDCamera']['dcy'] = dcy
        if flag == 1:
            print('slope update successfully, dfx:'+str(x['slopeSegDCamera']['dfx'])+' dfy:'+str(x['slopeSegDCamera']['dfy'])+' dcx:'+str(x['slopeSegDCamera']['dcx'])+' dcy:'+str(x['slopeSegDCamera']['dcy']))

        with open(slope_yaml_dir,'w') as w_f:
            # 覆盖原先的配置文件
            yaml.dump(x,w_f)

    # segfusion
    with open(segfusion_yaml_dir,'r') as f:
        result = f.read()
        x = yaml.load(result, Loader=yaml.FullLoader)
        flag = 0
        if x['segfusion']['fx_dc'] != dfx:
            print('segfusion_old, dfx:'+str(x['segfusion']['fx_dc'])+' dfy:'+str(x['segfusion']['fy_dc'])+' dcx:'+str(x['segfusion']['cx_dc'])+' dcy:'+str(x['segfusion']['cy_dc']))
            flag = 1
        
        # 修改的值
        x['segfusion']['fx_dc'] = dfx
        x['segfusion']['fy_dc'] = dfy
        x['segfusion']['cx_dc'] = dcx
        x['segfusion']['cy_dc'] = dcy
        if flag == 1:
            print('segfusion update successfully, dfx:'+str(x['segfusion']['fx_dc'])+' dfy:'+str(x['segfusion']['fy_dc'])+' dcx:'+str(x['segfusion']['cx_dc'])+' dcy:'+str(x['segfusion']['cy_dc']))

        with open(segfusion_yaml_dir,'w') as w_f:
            # 覆盖原先的配置文件
            yaml.dump(x,w_f)


    print('update successfully, use ctrl+c to exit the program')

def listener():
    rospy.init_node('update_dcam_param')
    rospy.Subscriber('/camera/depth/camera_info', CameraInfo, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()