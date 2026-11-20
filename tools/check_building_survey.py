"""Evaluate the Building round trip; truth is opened only after reconstruction."""
import argparse
import json
import math
import sqlite3
from pathlib import Path

import numpy as np
from sensor_msgs_py.point_cloud2 import dtype_from_fields
from export_survey import messages


def trajectory(path, topic):
    rows = []
    for _, message in messages(path, topic):
        pos, rot = message.pose.pose.position, message.pose.pose.orientation
        stamp = message.header.stamp
        yaw = math.atan2(2*(rot.w*rot.z + rot.x*rot.y), 1-2*(rot.y**2 + rot.z**2))
        rows.append([stamp.sec + stamp.nanosec*1e-9, pos.x, pos.y, pos.z, yaw])
    return np.asarray(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    root = args.directory.resolve()
    truth = trajectory(root/'evaluation', '/Odometry_gazebo')
    bag = root/'replay/registered'
    estimate = trajectory(bag, '/fast_lio/odometry_base')
    events = [(stamp, json.loads(msg.data)) for stamp, msg in messages(root/'evaluation', '/navigation/task_status')]
    start = next(stamp for stamp, task in events if task['phase']=='planning' and task['points'])
    turn = next(stamp for stamp, task in events if task['index']==1 and task['phase']=='at_waypoint')
    terminal = next(((stamp, task) for stamp, task in events if task['phase']=='succeeded'), None)
    if terminal:
        end, result = terminal
    else:
        stop, result = next((stamp,task) for stamp,task in events if task['phase']=='stopping' and task['reason'] and task['index']==1)
        end = stop+3.
    if min(truth[-1,0], estimate[-1,0]) < end-.2:
        raise RuntimeError(f'Replay or truth ends before the return evaluation at {end:.2f} s')
    def reference(stamp):
        return np.stack([np.interp(stamp, truth[:,0], truth[:,i]) for i in [1,2,3,4]], axis=-1)
    ref = reference(estimate[:,0])
    anchor = (estimate[:,0] >= start-1.5) & (estimate[:,0] < start-.2)
    yaw = np.median(np.angle(np.exp(1j*(ref[anchor,3]-estimate[anchor,4]))))
    rotate = np.array([[np.cos(yaw),np.sin(yaw),0],[-np.sin(yaw),np.cos(yaw),0],[0,0,1]])
    translation = np.median(ref[anchor,:3]-estimate[anchor,1:4]@rotate, axis=0)
    pose = estimate[:,1:4]@rotate + translation
    delta = pose-ref[:,:3]
    final = (estimate[:,0]>end-.6) & (estimate[:,0]<=end)
    circuit = (estimate[:,0]>=start) & (estimate[:,0]<=end)
    if not np.any(final) or not np.any(anchor):
        raise RuntimeError('Replay does not span settled start and finish')
    displacement = np.median(ref[final,:3], axis=0)-np.median(ref[anchor,:3], axis=0)
    stats = {'start_s':start,'turn_s':turn,'end_s':end,'odom_frames':len(estimate),
             'task_phase':result['phase'],'task_reason':result['reason'],
             'return_offset_to_requested_xy_m':float(np.linalg.norm(np.median(ref[final,:2],axis=0)-[7.5,6.])),
             'evaluation_anchor_xyz':translation.tolist(),'evaluation_anchor_yaw_rad':float(yaw),
             'return_offset_true_xy_m':float(np.linalg.norm(displacement[:2])),
             'closure_error_xy_m':float(np.linalg.norm(np.median(delta[final,:2],axis=0))),
             'closure_error_z_m':float(np.median(delta[final,2])),
             'trajectory_error_xy_p95_m':float(np.percentile(np.linalg.norm(delta[circuit,:2],axis=1),95)),
             'trajectory_length_true_m':float(np.linalg.norm(np.diff(ref[circuit,:3][::5],axis=0),axis=1).sum())}
    stats['measurements'] = {}
    with sqlite3.connect(f'{(root/"raw/raw_0.db3").as_uri()}?mode=ro', uri=True) as database:
        for topic,count,first,last in database.execute('SELECT name,count(*),min(timestamp),max(timestamp) FROM topics JOIN messages ON topics.id=topic_id GROUP BY name'):
            stats['measurements'][topic] = dict(messages=count, first_s=first*1e-9, last_s=last*1e-9)
    stats['trajectory_error_z_p95_m'] = float(np.percentile(abs(delta[circuit,2]),95))
    for label,mask in [('outbound',(estimate[:,0]>=start)&(estimate[:,0]<=turn)),('inbound',(estimate[:,0]>turn)&(estimate[:,0]<=end))]:
        stats[label+'_error_xy_p95_m']=float(np.percentile(np.linalg.norm(delta[mask,:2],axis=1),95))
    stats['horizontal_error_over_0_5m_first_s'] = next((float(t) for t,error in zip(estimate[circuit,0],np.linalg.norm(delta[circuit,:2],axis=1)) if error>.5),None)
    stats['replay_coverage_s'] = [float(estimate[0,0]),float(estimate[-1,0])]
    online = trajectory(root/'evaluation', '/fast_lio/odometry_base')
    settled = truth[truth[:,0]>=end]
    if len(settled):
        stats['after_return'] = {'from_s':end, 'true_xy_excursion_m':float(np.linalg.norm(settled[:,1:3]-settled[0,1:3],axis=1).max())}
        for name,track in [('online',online),('replay',estimate)]:
            rest=track[track[:,0]>=end]
            if len(rest):
                stats['after_return'][name+'_xy_excursion_m']=float(np.linalg.norm(rest[:,1:3]-rest[0,1:3],axis=1).max())
    stairs = [(-5.25,2.1,5.65,-3.12,0.),(8.8,2.4,-3.45,6.,1.5),(8.8,2.4,-3.52,5.16,3.)]
    segments = ['entry','lower','middle','upper','stairs_01','stairs_12','stairs_23']
    buckets = {name:[{},{}] for name in segments}
    for _, message in messages(bag,'/fast_lio/cloud_registered'):
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if not start <= stamp <= end: continue
        data = np.frombuffer(message.data,dtype=dtype_from_fields(message.fields,point_step=message.point_step))
        xyz = np.column_stack([data[axis] for axis in 'xyz'])@rotate+translation
        body = reference(stamp)
        xyz = xyz[np.linalg.norm(xyz[:,:2]-body[:2],axis=1)<2.5]
        x,y,z = xyz.T
        x = x-13.0
        stair_masks = []
        stair_projections = []
        for cx,width,first,last,base in stairs:
            along = (y-first)/(last-first)
            inside = (abs(x-cx)<width/2)&(along>0)&(along<1)
            stair_projections.append(inside & (abs(z-(base+1.5*along))<.55))
            stair_masks.append(inside & (abs(x-cx)<width/2-.16)&(along>.04)&(along<.96)&(abs(z-(base+1.5*along))<.32))
        outside = ~np.any(stair_projections,axis=0)
        masks = [outside & (abs(z-level)<.32) & (abs(body[2]-(level+.31))<.25) for level in [0.,1.5,3.,4.5]] + stair_masks
        for name,mask in zip(segments,masks):
            if not mask.any(): continue
            bins = np.floor(np.column_stack((x[mask],y[mask]))/.2).astype(int)
            values=z[mask]
            cells=bins[:,0]*10000+bins[:,1]
            order=np.lexsort((values,cells));keys,counts=np.unique(cells[order],return_counts=True)
            starts=np.cumsum(counts)-counts
            medians=(values[order[starts+counts//2]]+values[order[starts+(counts-1)//2]])*.5
            target=buckets[name][int(stamp>=turn)]
            for key,height in zip(keys,medians):target.setdefault(int(key),[]).append(float(height))
    stats['repeat_surface']={}
    for name,(outward,inward) in buckets.items():
        keys=[key for key in outward.keys()&inward.keys() if min(len(outward[key]),len(inward[key]))>=3]
        difference=np.array([np.median(inward[key])-np.median(outward[key]) for key in keys]);absolute=abs(difference)
        stats['repeat_surface'][name]={'cells':len(keys),'median_absolute_m':float(np.median(absolute)) if len(keys) else None,
            'p95_absolute_m':float(np.percentile(absolute,95)) if len(keys) else None}
    np.savetxt(root/'trajectory.csv',np.column_stack((estimate[:,0],ref[:,:3],pose)),delimiter=',',header='stamp,truth_x,truth_y,truth_z,replay_x,replay_y,replay_z',comments='')
    print(json.dumps(stats,indent=2,allow_nan=False))


if __name__ == '__main__':
    main()
