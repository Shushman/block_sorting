#!/usr/bin/env python
import argparse, herbpy, logging, numpy, os, prpy, random
import block_sorting.block_utils as block_utils
from prpy.planning import PlanningError


logger = logging.getLogger('block_sorting')
logger.setLevel(logging.INFO)
ch = logging.StreamHandler()
logger.addHandler(ch)

def setup_limits(robot):
    # TODO: Hack to get around owd/openrave joint limit mismatch
    owd_lower_limits = [ -2.60+numpy.pi, -1.96, -2.73, -0.86, -4.79, -1.56, -2.99 ]
    owd_upper_limits = [  2.60+numpy.pi,  1.96,  2.73,  3.13,  1.30,  1.56,  2.99 ]
    or_lower_limits, or_upper_limits = robot.GetDOFLimits()
    or_lower_limits[robot.right_arm.GetArmIndices()] = owd_lower_limits
    or_lower_limits[robot.left_arm.GetArmIndices()] = owd_lower_limits
    or_upper_limits[robot.right_arm.GetArmIndices()] = owd_upper_limits
    or_upper_limits[robot.left_arm.GetArmIndices()] = owd_upper_limits
    robot.SetDOFLimits(or_lower_limits, or_upper_limits)


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

    robot.right_arm.SetStiffness(1)
    robot.left_arm.SetStiffness(1)

    # Default to using real perception on the real robot, but allow the user to
    # override this by passing the --perception-sim option.
    if args.perception_sim is not None:
        perception_sim = args.perception_sim
    else:
        perception_sim = not args.nosim

    # Set the joint limits used by the planners
    setup_limits(robot)

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

    # Grab the table
    table = robot.DetectTable()

    # Detect the bins
    bins = robot.DetectBins(table)

    running = True
    blocks = []

    while running:
        logger.info('Redecting objects')
        blocks = robot.DetectBlocks(table)
        v = raw_input('Press enter to continue, r to redetect')
        if v == 'r':
            continue
        
        # Get a block
        block = block_utils.SelectBlock(blocks, bins)
        if block is None:
            logger.info('No blocks on table')
            raw_input('Press enter to continue')
            continue
        
        # Select an appropriate bin for the block
        block_bin = block_utils.GetBinForBlock(block, bins)

        logger.info('Grabbing block %s' % block.GetName())
        try:
            # Grab the block and put it into the bin
            robot.GrabBlock(block, table, manip=manip)
            block_utils.VerifyGrasp(block, manip)
            robot.PlaceBlock(block, block_bin, manip=manip)
        
            # If successful, take this block out of the environment
            env.Remove(block)

        except PlanningError, e:
            logger.error('Failed to grab block')
            if robot.IsGrabbing(block):
                robot.Release(block)
            raw_input('Press any key to continue')
        except prpy.exceptions.TrajectoryAborted, e:
            logger.error('Trajectory aborted. Joint limit?')
            robot.Say("I think I am stuck in joint limit. Can you help me?")
            raw_input('Check joint limits and press any key to continue')
        except block_utils.FailedGrasp, e:
            logger.error('Failed to grasp the block')
            robot.Say("Whoops, I missed.")
            
        try:
            with prpy.rave.Disabled(block_bin, padding_only=True):
                with prpy.rave.Disabled(block):
                    manip.PlanToNamedConfiguration('home')
        except (PlanningError, prpy.exceptions.TrajectoryAborted):
            robot.Say("I am stuck. Can you help me?")
            if sim:
                indices, config = robot.configurations.get_configuration('home')
                robot.SetDOFValues(config, dofindices=indices)
            else:
                raw_input('Press enter to continue')
            robot.Say("Thank you.")
        
    import IPython; IPython.embed()


        
