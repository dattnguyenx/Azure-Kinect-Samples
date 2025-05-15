import numpy as np
from pyntcloud import PyntCloud
from octree_partitioning import partition_octree, partition_octree_rec, departition_octree
import time
import pickle
import sys
import os
import glob
import math
def timing(f):
    def wrap(*args, **kwargs):
        time1 = time.time()
        ret = f(*args, **kwargs)
        time2 = time.time()
        print('{:s} function took {:.3f} ms'.format(f.__name__, (time2-time1)*1000.0))
        return ret
    return wrap
def find_expo2_bbox(box_max):
    n=math.log2(box_max)
    return math.ceil(n)

if __name__ == '__main__':
    # if(len(sys.argv)!=3):
    #    print('command: test.py path-to-file outputname level')
    # path,outputname,level=sys.argv[1:]
    input_glob="C:\\Users\\ke76boqe\\Projects\\Body_tracking\\body-tracking-samples\\PointCloud\\frame*.ply"
    output="C:\\Users\\ke76boqe\\Projects\\Body_tracking\\body-tracking-samples\\pc_bin\\"
    filenames=glob.glob(input_glob)
    for pc_path in filenames:

        filename = os.path.basename(pc_path)
        filename = filename[:-4]
        pc = PyntCloud.from_file(pc_path)
        points = pc.points.values
        points=points-np.min(points)
        points=np.round(points)

        level=find_expo2_bbox(np.max(points))
        box=math.pow(2,level)
        blocks2, binstr2 = timing(partition_octree)(points, [0, 0, 0], [box, box, box], level)
        with open(output  + filename + '.bin', 'wb') as f:
            newFileByteArray = bytearray(binstr2)
            f.write(newFileByteArray)
            f.close()
        #with open(output  + filename + '.bin', 'rb') as f:
            #    bin=f.read()
            #    binstr2=bytearray(bin)
        #    f.close()
        #decoded_blocks= departition_octree(blocks2, binstr2,[0, 0, 0], [box, box, box], level)
        #print('done')

        # print(len(binstr2),'\n',binstr2)


    #print(pc.points.describe())
    # Double call to account for numba compilation time
    #blocks2, binstr2 = timing(partition_octree)(points, [0, 0, 0], [1024, 1024, 1024], 10)
    #blocks2, binstr2 = timing(partition_octree)(points, [0, 0, 0], [1024, 1024, 1024], 1)
    #blocks, binstr = timing(partition_octree_rec)(points, [0, 0, 0], [1024, 1024, 1024], 2)

    #len1 = len(blocks)
    #len2 = len(blocks2)
    #lenb1 = len(binstr)
    #lenb2 = len(binstr2)
    #print(len1, len2, lenb1, lenb2)
    #assert len1 == len2, f'Found {len1} != {len2}'
    #assert lenb1 == lenb2, f'Found {lenb1} != {lenb2}'
    #np.testing.assert_array_equal(binstr, binstr2)
    #for i, (b1, b2) in enumerate(zip(blocks, blocks2)):
    #    np.testing.assert_array_equal(b1, b2, err_msg=f'Block {i} error')
    #print('Test successful.')
    #saving block and binstr in to file
