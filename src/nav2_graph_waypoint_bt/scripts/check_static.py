#!/usr/bin/env python3
from pathlib import Path
import xml.etree.ElementTree as ET
root=Path(__file__).resolve().parents[1]
ET.parse(root/'package.xml')
for x in (root/'behavior_trees').glob('*.xml'):
    tree=ET.parse(x)
    outer=tree.getroot()
    assert outer.attrib.get('BTCPP_format')=='4',x
    assert 'ComputePathThroughPoses' in [n.tag for n in outer.iter()],x
    assert 'FollowPath' in [n.tag for n in outer.iter()],x
    assert 'PipelineSequence' in [n.tag for n in outer.iter()],x
    assert 'RateController' in [n.tag for n in outer.iter()],x
    print('XML OK:',x.name)
print('static checks passed')
