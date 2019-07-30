# BUNDLE_CONTENT
# Provide bundle information

# New bundle msg will contain the following information:
# 1. bundleID, still in uint8.
# 2. For each quad, there will be 10 variables for location (northing and easting, assuming the bundle length remains 5) and 1 variable for altitude (assuming fixed altitude flight), all in float64.
# 3. We set the space for 6 quads in total. So there will be 66 float64 variables.
# 4. If there are more quads added, we will extend the total number of float64 variables accordingly.

uint8 bundleID
uint8 bundleStatus
float64 agent1_n1
float64 agent1_e1
float64 agent1_n2
float64 agent1_e2
float64 agent1_n3
float64 agent1_e3
float64 agent1_n4
float64 agent1_e4
float64 agent1_n5
float64 agent1_e5
float64 agent1_al
float64 agent2_n1
float64 agent2_e1
float64 agent2_n2
float64 agent2_e2
float64 agent2_n3
float64 agent2_e3
float64 agent2_n4
float64 agent2_e4
float64 agent2_n5
float64 agent2_e5
float64 agent2_al
float64 agent3_n1
float64 agent3_e1
float64 agent3_n2
float64 agent3_e2
float64 agent3_n3
float64 agent3_e3
float64 agent3_n4
float64 agent3_e4
float64 agent3_n5
float64 agent3_e5
float64 agent3_al
float64 agent4_n1
float64 agent4_e1
float64 agent4_n2
float64 agent4_e2
float64 agent4_n3
float64 agent4_e3
float64 agent4_n4
float64 agent4_e4
float64 agent4_n5
float64 agent4_e5
float64 agent4_al
float64 agent5_n1
float64 agent5_e1
float64 agent5_n2
float64 agent5_e2
float64 agent5_n3
float64 agent5_e3
float64 agent5_n4
float64 agent5_e4
float64 agent5_n5
float64 agent5_e5
float64 agent5_al
float64 agent6_n1
float64 agent6_e1
float64 agent6_n2
float64 agent6_e2
float64 agent6_n3
float64 agent6_e3
float64 agent6_n4
float64 agent6_e4
float64 agent6_n5
float64 agent6_e5
float64 agent6_al
