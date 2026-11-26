import argparse
from pathlib import Path
from .path import COLORS


def template(path):
    dest=Path(path);dest.parent.mkdir(parents=True,exist_ok=True)
    content=['<svg xmlns="http://www.w3.org/2000/svg" width="297mm" height="210mm" viewBox="0 0 297 210">',
             '<g font-family="sans-serif" fill="#222"><text x="80" y="33" font-size="5">SO-101 registration sheet</text>',
             '<text x="80" y="40" font-size="3.3">Landscape A4 · print at 100% · verify 136 × 108 mm between outer crosses</text>']
    for r in range(3):
        for c in range(3):
            x,y=80+c*68,50+r*54
            content.append(f'<path d="M{x-3} {y}h6 M{x} {y-3}v6" stroke="black" stroke-width=".24"/>')
            content.append(f'<text x="{x+3.5}" y="{y-2}" font-size="3">R{r+1}C{c+1}</text>')
    dest.write_text('\n'.join([*content,'</g></svg>'])+'\n');print(dest)


def main():
    cli=argparse.ArgumentParser(prog='python -m portrait')
    sub=cli.add_subparsers(dest='command',required=True)
    sim=sub.add_parser('sim',help='replay all strokes on the MuJoCo follower');sim.add_argument('--out',default='out/sim')
    guide=sub.add_parser('template',help='write printable registration sheet');guide.add_argument('--out',default='out/registration.svg')
    for name in ('teach','run','release'):
        p=sub.add_parser(name)
        p.add_argument('--id',required=True);p.add_argument('--port',required=name!='run')
        if name!='release':p.add_argument('--color',required=True,choices=COLORS)
        if name=='release':p.add_argument('--supported',action='store_true')
        if name=='run':
            p.add_argument('--mode',required=True,choices=['air','swatch','draw'])
            p.add_argument('--start',type=int,default=0);p.add_argument('--count',type=int,default=1)
            p.add_argument('--execute',action='store_true')
    args=cli.parse_args()
    try:
        if args.command=='template':template(args.out)
        elif args.command=='sim':
            from .sim import run
            run(args.out)
        else:
            from . import hardware
            {'teach':hardware.teach,'run':hardware.run,'release':hardware.release}[args.command](args)
    except (OSError, ValueError, RuntimeError, KeyError) as exc:
        cli.exit(1,f'Stopped: {exc}\n')

if __name__=='__main__':main()
