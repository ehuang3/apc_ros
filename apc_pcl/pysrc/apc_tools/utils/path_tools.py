import os

def path_to_file(_file):
    return os.path.dirname(os.path.realpath(_file))

def path_to_root():
    fpath = os.path.dirname(os.path.realpath(__file__))
    return os.path.realpath(os.path.join(fpath, '..', '..', '..', '..'))

# Test
if __name__ == '__main__':
    print path_to_root()
