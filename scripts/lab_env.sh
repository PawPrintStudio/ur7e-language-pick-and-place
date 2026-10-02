# Source this in every lab terminal/container shell before launching anything.
#
# CycloneDDS on this laptop ends up discovering over the loopback interface
# by unicast, where it probes a fixed range of "participant indices". The
# default range allows about ten processes per machine; the driver launch
# alone uses eight, and the eleventh node (a console, the perception node)
# fails with "Failed to find a free participant index for domain 42".
# Raising the range is enough -- but every process must be started with the
# same setting, so the driver has to be launched with it too.
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
