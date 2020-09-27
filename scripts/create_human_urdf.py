import sys
import argparse
import xacro



TPL = "../urdf/human-tpl.xacro"

parser = argparse.ArgumentParser(description='Generate the URDF of a human.')
parser.add_argument('-i', '--id', type=str, default="",
                    help='the person\'s unique ID (default: empty)')
parser.add_argument('-u', '--upperarm-length', type=float, default=20.,
                    help='length of the upperarm, in cm (default: 20)')
parser.add_argument('-f', '--forearm-length', type=float, default=30.,
                    help='length of the forearm, in cm (default: 30)')


args = parser.parse_args()

params = {
          'id': args.id,
          'upperarm_length': str(args.upperarm_length/100.),
          'forearm_length': str(args.forearm_length/100.)
          }

print(xacro.process_file(TPL, mappings=params).toprettyxml(indent="   "))
