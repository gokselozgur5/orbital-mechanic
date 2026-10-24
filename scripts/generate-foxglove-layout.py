#!/usr/bin/env python3
"""Generate the operator view with foxglove-sdk==0.30.2 (development-only)."""
import json
from pathlib import Path
import foxglove.layouts as f

layout=f.Layout(content=f.SplitContainer(direction='row',items=[
    f.SplitItem(content=f.SplitContainer(direction='column',items=[
        f.SplitItem(content=f.ImagePanel(title='Robot camera',config=f.ImageConfig(
            image_mode=f.ImageModeConfig(image_topic='/camera/image_raw',calibration_topic='/camera/camera_info',synchronize=False)))),
        f.SplitItem(content=f.ThreeDeePanel(title='Lidar and robot frame',config=f.ThreeDeeConfig(
            follow_tf='base_link',fixed_frame='odom',
            camera_state=f.ThreeDeeCameraState(distance=8,perspective=False),
            topics={'/scan':f.BaseRendererLaserScanTopicSettings(visible=True,point_size=3,color_mode='flat',flat_color='#50e3c2')}))),
    ])),
    f.SplitItem(content=f.SplitContainer(direction='column',items=[
        f.SplitItem(content=f.RawMessagesPanel(title='Stop latch and reason',config=f.RawMessagesConfig(topic_path='/motion/state',default_expanded=True))),
        f.SplitItem(content=f.PlotPanel(title='Commanded and measured velocity',config=f.PlotConfig(
            following_view_width=20,show_legend=True,paths=[
                f.PlotSeries(value='/motion/guarded_cmd_vel.linear.x',label='Command linear (m/s)'),
                f.PlotSeries(value='/odom.twist.twist.linear.x',label='Measured linear (m/s)'),
                f.PlotSeries(value='/motion/guarded_cmd_vel.angular.z',label='Command angular (rad/s)'),
                f.PlotSeries(value='/odom.twist.twist.angular.z',label='Measured angular (rad/s)'),
            ]))),
    ])),
]))
output=Path(__file__).resolve().parents[1]/'config/foxglove-operator.json'
# The SDK serializes notebook layouts. Desktop import expects mosaic layout data.
notebook=json.loads(layout.to_json())
configs={}
def desktop_node(node):
    if node['type']=='panel':
        panel_type={"ThreeDee":"3D"}.get(node["panelType"],node["panelType"])
        panel_id=f"{panel_type}!orbital{len(configs)+1}"
        configs[panel_id]=node.get('config',{})
        return panel_id
    assert node['type']=='split'
    items=node['items']
    assert len(items)==2
    total=sum(item.get('proportion',1) for item in items)
    return {'direction':node['direction'],
            'first':desktop_node(items[0]['content']),
            'second':desktop_node(items[1]['content']),
            'splitPercentage':100*items[0].get('proportion',1)/total}
root=desktop_node(notebook['content'])
desktop={'version':1,'configById':configs,'layout':root,
         'globalVariables':{},'userNodes':{},'linkedGlobalVariables':[],
         'playbackConfig':{'speed':1}}
output.write_text(json.dumps(desktop,indent=2)+'\n')
print(output)
