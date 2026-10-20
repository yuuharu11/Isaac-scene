# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Render measured common-base travel and work at separated library stations."""

import argparse
import json
from pathlib import Path

from isaaclab_arena_examples.library.render_video import LibraryRenderer


class MobileDualRenderer(LibraryRenderer):
    """Use one world-fixed shadow projection over the entire mobile route."""

    def shadow_projection(self, positions):
        camera=(positions-self.shadow_center)@self.shadow_rotation.T
        return self.torch.stack((camera[...,0]/6.,camera[...,1]/6.,-camera[...,2]/6.),dim=-1)


def main():
    import imageio_ffmpeg
    import numpy as np
    from PIL import Image,ImageDraw,ImageFont
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture_dir',type=Path)
    parser.add_argument('--width',type=int,default=1280)
    parser.add_argument('--height',type=int,default=720)
    parser.add_argument('--preview-only',action='store_true')
    args=parser.parse_args()
    assert args.width>0 and args.height>0 and args.width%2==0 and args.height%2==0
    output=args.capture_dir/'mobile-dual.mp4'
    result=json.loads((args.capture_dir/'result.json').read_text())
    labels={'drive_gap':'Carry the returned book to shelf A','drive_return':'Reposition the shared base for insertion',
        'drive_loan':'Move both arms to shelf B','drive_boxes':'Carry the requested book back to the checkout box',
        'gap_spread':'Right gripper opens a gap by pushing adjacent books',
        'return_insert':'Left gripper inserts the returned book into shelf A',
        'loan_extract':'Right gripper withdraws the requested book from shelf B',
        'box_lower':'Place the requested book in the separate checkout box','done':'Mobile dual-arm return and loan completed'}
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',22)
    with np.load(args.capture_dir/'scene.npz',allow_pickle=False) as scene:
        renderer=MobileDualRenderer(scene,args.width,args.height)
        def frame(index,overview=False):
            phase=str(scene['phases'][index]);extra=json.loads(str(scene['extras_json'][index]))
            fallback=[a-b for a,b in zip(result['arms']['right']['root_pose'][:3],result['scene']['arm_mounts']['right'])]
            offset=extra.get('base_offset',fallback)
            if overview or phase.startswith("drive_"):
                renderer.camera_eye=[5.8,-7.5,4.8];renderer.camera_target=[.2,-.3,1.0];renderer.fov=48.
            else:
                renderer.camera_eye=[offset[0]+1.6,offset[1]-3.2,2.0]
                renderer.camera_target=[offset[0],offset[1]+.25,1.15];renderer.fov=53.
            renderer.set_camera()
            image=Image.fromarray(renderer.render(scene['poses'][index]));draw=ImageDraw.Draw(image)
            label=labels.get(phase,phase.replace('_',' '))
            if phase=='done' and not result.get('success'):label='Cycle ended; final validation failed'
            if index==len(scene['poses'])-1 and not result.get('success'):label='Trial state: '+result['reason']
            draw.rectangle((0,args.height-72,args.width,args.height),fill=(22,29,34))
            draw.multiline_text((12,args.height-66),f'{label}\nBase: ({offset[0]:.2f}, {offset[1]:.2f}) m | t={scene["times"][index]:.1f}s',font=font,fill=(240,244,247),spacing=5)
            return np.asarray(image)
        first,last=frame(0,True),frame(len(scene['poses'])-1,True)
        Image.fromarray(first).save(args.capture_dir/'mobile-dual-first.png')
        Image.fromarray(last).save(args.capture_dir/'mobile-dual-last.png')
        for phase in ('drive_gap','gap_spread','return_insert','drive_loan','loan_extract','drive_boxes'):
            indices=np.flatnonzero(scene['phases']==phase)
            if len(indices):Image.fromarray(frame(int(indices[len(indices)//2]))).save(args.capture_dir/(phase+'.png'))
        Image.fromarray(frame(len(scene['poses'])-1)).save(args.capture_dir/'mobile-dual-detail.png')
        if args.preview_only:return
        writer=imageio_ffmpeg.write_frames(str(output),(args.width,args.height),fps=30,codec='libx264',
            pix_fmt_in='rgb24',pix_fmt_out='yuv420p',quality=8,macro_block_size=1,ffmpeg_log_level='error',output_params=['-movflags','+faststart'])
        writer.send(None);count=0
        try:
            for _ in range(60):writer.send(first);count+=1
            for i in range(len(scene['poses'])):
                writer.send(frame(i));count+=1
                if i%200==0:print(f'Rendered {i+1}/{len(scene["poses"])}',flush=True)
            for _ in range(60):writer.send(last);count+=1
        finally:writer.close()
        metadata={'frames':count,'fps':30,'duration_seconds':count/30,'width':args.width,'height':args.height,
            'simulation_success':result.get('success',False),'renderer':'actual USD geometry and PhysX poses; CUDA rasterization',
            'motion_speedup':float((scene['times'][-1]-scene['times'][0])*30/max(len(scene['times'])-1,1)),'source_time_end':float(scene['times'][-1])}
        output.with_suffix('.json').write_text(json.dumps(metadata,indent=2)+'\n');print(metadata,flush=True)


if __name__=='__main__':main()
