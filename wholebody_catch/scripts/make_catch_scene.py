#!/usr/bin/env python3
"""Generate single-object, markerless simulation trials; never used by control."""
import argparse
from pathlib import Path
import random
import xml.etree.ElementTree as ET


def generate(template, output, shape, seed):
    rng = random.Random(seed)
    tree = ET.parse(template)
    world = tree.getroot().find('world')
    target = world.find("model[@name='catch_target']")
    support = world.find("model[@name='catch_pedestal']")
    x, y = rng.uniform(1.05, 1.23), rng.uniform(-.10, .10)
    width = rng.uniform(.032, .042)
    mass = .015
    if shape == 'sphere':
        geom = '<sphere><radius>{}</radius></sphere>'.format(width/2)
        inertia = [mass*width**2/10]*3
    elif shape == 'box':
        geom = '<box><size>{0} {0} {0}</size></box>'.format(width)
        inertia = [mass*width**2/6]*3
    elif shape == 'cylinder':
        geom = '<cylinder><radius>{}</radius><length>{}</length></cylinder>'.format(width/2, width)
        inertia = [mass*(3*(width/2)**2+width**2)/12]*2+[mass*(width/2)**2/2]
    else:
        raise ValueError('Unsupported shape')
    target.find('pose').text = '{} {} {} 0 0 0'.format(x, y, 1.2801+width/2)
    support.find('pose').text = '{} {} 0 0 0 0'.format(x, y)
    link = target.find('link')
    for kind in ('collision', 'visual'):
        geometry = link.find(kind+'/geometry')
        geometry.clear()
        geometry.append(ET.fromstring(geom))
    color = {'sphere': '.8 .65 .12 1', 'box': '.12 .65 .8 1', 'cylinder': '.15 .75 .3 1'}[shape]
    link.find('visual').set('name', 'unmarked_object')
    for tag in ('ambient', 'diffuse'):
        link.find('visual/material/'+tag).text = color
    for key, value in zip(('ixx', 'iyy', 'izz'), inertia):
        link.find('inertial/inertia/'+key).text = str(value)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding='utf-8', xml_declaration=True)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--shape', choices=['sphere', 'box', 'cylinder'], default='box')
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--output', default='/tmp/wholebody_catch_object.world')
    parser.add_argument('--template')
    args = parser.parse_args()
    if args.template:
        template = args.template
    else:
        from ament_index_python.packages import get_package_share_directory
        template = Path(get_package_share_directory('wholebody_catch'))/'worlds/catch.world'
    print(generate(template, args.output, args.shape, args.seed))


if __name__ == '__main__':
    main()
