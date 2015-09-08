#!/usr/bin/env python
import argparse, herbpy, logging, numpy, os, prpy, random
from prpy.planning import PlanningError

logger = logging.getLogger('block_sorting')
logger.setLevel(logging.INFO)
ch = logging.StreamHandler()
logger.addHandler(ch)

def select_block(blocks, block_bin):
    """
    Randomly select a block that is not already in the bin
    """
    bin_aabb = block_bin.ComputeAABB()

    valid_blocks = []
    for b in blocks:
        # Get the pose of the block
        b_pose = b.GetTransform()

        # Check if its inside the tray
        if b_pose[0,3] < bin_aabb.pos()[0] - bin_aabb.extents()[0] or \
           b_pose[0,3] > bin_aabb.pos()[0] + bin_aabb.extents()[0] or \
           b_pose[1,3] < bin_aabb.pos()[1] - bin_aabb.extents()[1] or \
           b_pose[1,3] > bin_aabb.pos()[1] + bin_aabb.extents()[1]:
            valid_blocks.append(b)
        

    if len(valid_blocks) == 0:
        return None
    idx = random.randint(0, len(valid_blocks) - 1)
    return valid_blocks[idx]

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Detect and sort blocks on a table")
    parser.add_argument("-r", "--nosim", dest="nosim", action="store_true", default=False,
                      help="Run on the real robot rather than in simulation mode.")
    parser.add_argument("--perception-sim", action="store_true", default=None,
                      help="Use simulated perception, even if --nosim was specified.")
    parser.add_argument("-v", "--viewer", dest="viewer", default=None,
                        help="The viewer to attach")
    parser.add_argument("-a", "--arm", dest="arm", default='right',
                        help="The arm to grab the blocks with")

    args = parser.parse_args()
    sim = not args.nosim

    herbpy_args = {'left_arm_sim': sim,
                   'left_hand_sim': sim,
                   'right_arm_sim': sim,
                   'right_hand_sim': sim,
                   'head_sim': sim,
                   'segway_sim': True,
                   'perception_sim': sim}
    env, robot = herbpy.initialize(**herbpy_args)
    if args.viewer:
        env.SetViewer(args.viewer) # 14.04 weirdness

    # Default to using real perception on the real robot, but allow the user to
    # override this by passing the --perception-sim option.
    if args.perception_sim is not None:
        perception_sim = args.perception_sim
    else:
        perception_sim = not args.nosim

    # Make the manipulator active
    manip = robot.GetManipulator(args.arm)
    robot.SetActiveManipulator(manip)

    # Detect static objects in the environment
    if perception_sim:
        from block_sorting.perception_simulator import PerceptionSimulator
        robot.detector = PerceptionSimulator()
    else:
        import rospy
        from prpy.perception.block_detector import BlockDetector

        rospy.init_node('block_sorting', anonymous=True)
        robot.detector = BlockDetector(
            point_cloud_topic='/head/kinect2/qhd/points',
            detection_frame='/head/kinect2_rgb_optical_frame',
            destination_frame='/herb_base'
        )

    robot.DetectObjects()

    # Grab the table
    table = env.GetKinBody('conference_table')
    block_bin = env.GetKinBody('wicker_tray')

    running = True
    blocks = []

    while running:
        logger.info('Redecting objects')
        blocks = robot.DetectBlocks(table)

        # Get a block
        block = select_block(blocks, block_bin)
        if block is None:
            logger.info('No blocks on table')
            continue
        
        logger.info('Grabbing block %s' % block.GetName())
        raw_input('Press enter to continue')
        try:
            robot.GrabBlock(block, table, manip=manip)
            robot.PlaceBlock(block, block_bin, manip=manip)
        except PlanningError, e:
            logger.error('Failed to grab block')
            raw_input('Press any key to continue')
            
        try:
            with prpy.rave.Disabled(block_bin, padding_only=True):
                with prpy.rave.Disabled(block):
                    manip.PlanToNamedConfiguration('home')
        except PlanningError, e:
#            robot.Say("I am stuck. Can you help me?")
            if sim:
                indices, config = robot.configurations.get_configuration('home')
                robot.SetDOFValues(config, dofindices=indices)
            else:
                raw_input('Press enter to continue')
#            robot.Say("Thank you.")

    import IPython; IPython.embed()


        
