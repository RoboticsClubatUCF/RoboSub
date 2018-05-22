
#!/usr/bin/env python

import rospy
import smach
import smach_ros

import actionlib
import actionlib_msgs.msg

import vision_manager
import trajectory_planner

import vision_manager.msg
import trajectory_planner.msg

import monitor


import gate

class SubStates:

        def __init__(self):
                # Initialize node for the State Machine
                rospy.loginfo("State Machine has started.")

                self.movement_client = actionlib.SimpleActionClient('movement_server')
                rospy.loginfo("Created SimpleActionClient.")

                self.vision_client = actionlib.SimpleActionClient('track_object')
                rospy.loginfo("Created SimpleActionClient.")

                # Declare the top level state
                self.tasks = smach.StateMachine(outcomes=['success'])

                # Open top level state machine
                with tasks:

                        # Gate task
                        smach.StateMachine.add("GATE", gate(),
                                               transitions={'preempted':'preempted',
                                                            'success':'PATH',
                                                            'failure':'GATE'})

                        # Path to bouy task
                        smach.StateMachine.add("PATHTOBOUY", pathToBouy(),
                                               transitions={'preempted':'preempted',
                                                            'success':'BOUY',
                                                            'failure':'PATHTOBOUY'})

                        # Bouy task
                        smach.StateMachine.add("BUOY", bouy(),
                                               transitions={'preempted':'preempted',
                                                            'success':'PATHTOBUYCHIP',
                                                            'failure':'BUOY'})

                        # Path to buy chips task
                        smach.StateMachine.add("PATHTOBUYCHIP", pathToByChip(),
                                               transitions={'preempted':'preempted',
                                                            'success':'CHIP',
                                                            'failure':'PATHTOBUYCHIP'})

                        # Chips task
                        smach.StateMachine.add("CHIP", chip(),
                                               transitions={'preempted':'preempted',
                                                            'success':'PATHTOSLOTS',
                                                            'failure':'CHIP'})

			# Path to Slot machine task
			smach.StateMachine.add("PATHTOSLOTS", pathToSlots(),
					       transitions={'preempted':'preempted',
							    'success':''SLOTS',
							    'failure':'PATHTOSLOTS'})

			# Slot machine task
			smach.StateMachine.add("SLOTS", slots(),
					       transitions={'preempted':'preempted',
							    'success':'PINGERTOROULETTE',
							    'failure':'SLOTS'})

			# Path to Roulette task
			smach.StateMachine.add("PINGERTOROULETTE", pingerToRoulette(),
					       transitions={'preempted':'preempted',
							    'success':'ROULETTE',
							    'failure':'PINGERTOROULETTE'})
			# Roulette Task
			smach.StateMachine.add("ROULETTE", roulette(),
					       transitions={'preempted':'preempted',
							    'success':'PINGERTOCASHOUT',
							    'failure':'ROULETTE'})
			# Pinger to Cash out
			smach.StateMachine.add("PINGERTOCASHOUT", pingerToCashOut(),
					       transitions={'preempted':'preempted',
							    'success':'CASHOUT',
							    'failure':'PINGERTOCASHOUT'

                        # Cashout task
                        smach.StateMachine.add("CASHOUT", cashout(),
                                               transitions={'preempted':'preempted',
                                                            'success':'success',
                                                            'failure':'CASHOUT'})

                        # Add a concurrent MonitorState (or better option) in order to watch for E-stop (physical and logical) battery level, etc.
                        # Will need to add a preemption transition to all states, and should auto start at the preempted state if the reason for
                        # the preemption changes (i.e. E-stop is turned off). Need to figure out how and when State Machine is starting during competition.

                        # Gate task state transitions
                        gate = smach.StateMachine(outcomes=['preempted', 'succeess', 'failure'])

                        with gate:

                                smach.StateMachine.add('LOCATE', locate(),
                                                       transitions={'preempted':'preempted',
                                                                    'success': 'ALIGN',
                                                                    'failure': 'locate'})

                                smach.StateMachine.add('ALIGN', align(),
                                                       transitions={'preempted':'preempted',
                                                                    'success': 'THROUGH',
                                                                    'failure': 'locate'})

                                smach.StateMachine.add('THROUGH', through(),
                                                       transitions={'preempted':'preempted',
                                                                    'success': 'GATETOBOUY',
                                                                    'failure': 'locate'})

                        return gate

                outcome = tasks.execute()

if __name__ == '__main__':
        rospy.init_node('ness_sm')
        sm = SubStates()
        rospy.spin()
