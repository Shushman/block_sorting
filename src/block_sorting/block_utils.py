#!/usr/bin/env python
import prpy, os, random

def GetBinForBlock(block, bins):
    """
    Return the appropriate bin for the block to be placed in
    based on the color of the block
    @param block The block to choose a bin for
    @param bins A list of bins to choose from
    """
    # Load the bin metadata from yaml
    bin_config_dir = prpy.util.FindCatkinResource('block_sorting', 'config')
    bin_config = os.path.join(bin_config_dir, 'bin_metadata.yaml')
    with open(bin_config, 'r') as f:
        import yaml
        metadata = yaml.load(f.read())

    # Get the color of the block
    geom = block.GetLinks()[0].GetGeometries()[0]
    color = geom.GetDiffuseColor()

    # Next go through the bins and figure out which 
    #  is correct
    for b in bins:
        bin_name = b.GetName()

        # TODO: Use the metadata to determine if this is the correct bin

    return_bin = bins[0] # TODO: Remove and replace with actual logic

    return return_bin

def SelectBlock(blocks, bins):
    """
    Randomly select a block that is not already in one of the bins
    @param blocks A list of all detected blocks
    @param bins A list of all the bins that blocks can be placed into
    @return None if there are no blocks not already in bins, otherwise, the selected block
    """
    if blocks is None:
        return None

    valid_blocks = []
    for b in blocks:
        # Get the pose of the block
        b_pose = b.GetTransform()

        valid = True
        for bn in bins:
            bin_aabb = bn.ComputeAABB()

            # Check if its inside the bin
            if b_pose[0,3] > bin_aabb.pos()[0] - bin_aabb.extents()[0] and \
               b_pose[0,3] < bin_aabb.pos()[0] + bin_aabb.extents()[0] and \
               b_pose[1,3] > bin_aabb.pos()[1] - bin_aabb.extents()[1] and \
               b_pose[1,3] < bin_aabb.pos()[1] + bin_aabb.extents()[1]:
                valid = False
                break

        if valid:
            valid_blocks.append(b)
        

    if len(valid_blocks) == 0:
        return None
    idx = random.randint(0, len(valid_blocks) - 1)
    return valid_blocks[idx]
