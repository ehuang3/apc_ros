#!/usr/bin/env python
import collada
import argparse


def main():
    parser = argparse.ArgumentParser(description='Convert a Collada document to meters.')
    parser.add_argument('dae', help='DAE file to convert.')
    args = parser.parse_args()
    mesh = collada.Collada(args.dae)

    print mesh


if __name__ == '__main__':
    main()
