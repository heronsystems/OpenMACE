robot1 = roboticAgents(1,1).m_MasterTaskList.getTaskWithKey('0,1');
robot1.m_CurrentBids.m_BidDescriptors('1')
robot1 = roboticAgents(1,1).m_MasterTaskList.getTaskWithKey('0,2');
robot1.m_CurrentBids.m_BidDescriptors('1')

robot2 = roboticAgents(2,1).m_MasterTaskList.getTaskWithKey('0,1');
robot2.m_CurrentBids.m_BidDescriptors('2')
robot2 = roboticAgents(2,1).m_MasterTaskList.getTaskWithKey('0,2');
robot2.m_CurrentBids.m_BidDescriptors('2')

robot1 = roboticAgents(1,1).m_CurrentTaskQueue.sizeOfTaskList()
robot2 = roboticAgents(2,1).m_CurrentTaskQueue.sizeOfTaskList()
