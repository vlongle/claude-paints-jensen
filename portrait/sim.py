"""Independent MuJoCo joint-chain and paper-contact replay. Runs without OpenGL."""
import json
from pathlib import Path
from xml.etree.ElementTree import Element, SubElement, tostring
import mujoco
import numpy as np
from PIL import Image,ImageDraw
from .model import BASE, ORIGINS, JOINTS, LIMITS, TIP, forward
from .path import SIZE, strokes, virtual_grid, trajectory

INK={'black':'#242423','green':'#45a35a','gray':'#888885','orange':'#de752a'}


def numbers(a):
    return ' '.join(f'{x:.10g}' for x in a)


def scene():
    root=Element('mujoco',model='SO-101 marker')
    SubElement(root,'compiler',angle='degree')
    world=SubElement(root,'worldbody')
    SubElement(world,'geom',name='paper',type='box',size='.054 .068 .0005',
               pos='.285 -.018 -.0225',contype='1',conaffinity='2')
    body=world
    for i,(name,matrix) in enumerate(zip(JOINTS,BASE)):
        quat=np.empty(4)
        mujoco.mju_mat2Quat(quat,np.ascontiguousarray(matrix[:3,:3]).flatten())
        body=SubElement(body,'body',name=name,pos=numbers(matrix[:3,3]/1000),quat=numbers(quat))
        SubElement(body,'joint',name=name,type='hinge',axis='0 0 1',range=numbers(LIMITS[i]))
        end=np.array(ORIGINS[i+1][0]) if i<4 else TIP
        SubElement(body,'geom',type='capsule',size='.008',fromto='0 0 0 '+numbers(end/1000),
                   contype='0',conaffinity='0',rgba='.65 .65 .67 1')
    SubElement(body,'geom',name='nib',type='sphere',pos=numbers(TIP/1000),size='.0008',contype='2',conaffinity='1')
    return tostring(root,encoding='unicode')


def run(directory):
    out=Path(directory)
    out.mkdir(parents=True,exist_ok=True)
    plan=strokes()
    q,_,mark,ids=trajectory(plan,virtual_grid())
    xml=scene()
    model=mujoco.MjModel.from_xml_string(xml)
    data=mujoco.MjData(model)
    nib=model.geom('nib').id
    tip=np.empty((len(q),3)); contact=np.zeros(len(q),bool)
    for i,angles in enumerate(q):
        data.qpos[:]=np.deg2rad(angles)
        mujoco.mj_forward(model,data)
        tip[i]=data.geom_xpos[nib]*1000
        contact[i]=data.ncon>0
    residual=np.linalg.norm(forward(q)-tip,axis=1)
    if np.any(mark & ~contact) or residual.max()>.01:
        raise ValueError('MuJoCo contact or forward kinematics disagrees with the plan')
    # World millimetres to the printed working rectangle, with the far edge at the top.
    xy=np.column_stack([50-tip[:,1],339-tip[:,0]])
    scale=7; pad=18
    im=Image.new('RGB',tuple((SIZE*scale+2*pad).astype(int)),'#faf9f5')
    draw=ImageDraw.Draw(im)
    points=(xy*scale+pad).round().astype(int)
    for i in np.flatnonzero(mark & contact):
        draw.line([tuple(points[i-1]),tuple(points[i])],INK[plan[ids[i]][0]],width=6)
    im.save(out/'portrait.png')
    np.savez_compressed(out/'trajectory.npz',joint_degrees=q,paper_mm=xy,tip_mm=tip,contact=contact,
                        drawing=mark,stroke=ids)
    (out/'so101.xml').write_text(xml+'\n')
    stats={'strokes':len(plan),'per_colour':{c:sum(s[0]==c for s in plan) for c in INK},
           'joint_poses':len(q),'confirmed_ink_segments':int((mark&contact).sum()),
           'max_model_disagreement_mm':float(residual.max())}
    (out/'report.json').write_text(json.dumps(stats,indent=2)+'\n')
    # A GIF uses the simulated nib path for ink and the five simulated joint angles for the side view.
    board=Image.new('RGB',(300,252),'#faf9f5')
    pen=ImageDraw.Draw(board); snapshots=[]; anchors=set(np.linspace(0,len(q)-1,90,dtype=int))
    chain=forward(q,links=True)
    for i in range(len(q)):
        if mark[i] and contact[i]:
            seg=(xy[i-1:i+1]*2+12).round().astype(int)
            pen.line([tuple(seg[0]),tuple(seg[1])],INK[plan[ids[i]][0]],width=2)
        if i not in anchors: continue
        frame=Image.new('RGB',(600,252),'#ebeae6');frame.paste(board)
        side=ImageDraw.Draw(frame);side.text((319,12),'SO-101  /  side view',fill='#262627')
        side.line((305,200,596,200),fill='#c6beb2',width=2)
        links=[(int(312+x*.68),int(186-z*.68)) for x,_,z in chain[i]]
        side.line(links,fill='#4d5360',width=5)
        for x,y in links[:-1]:side.ellipse((x-3,y-3,x+3,y+3),fill='#b0b1ac')
        x,y=links[-1];side.ellipse((x-3,y-3,x+3,y+3),fill=INK[plan[ids[i]][0]] if ids[i]>=0 else '#272727')
        snapshots.append(frame)
    snapshots[0].save(out/'replay.gif',save_all=True,append_images=snapshots[1:],duration=110,loop=0)
    print(json.dumps(stats,indent=2))
