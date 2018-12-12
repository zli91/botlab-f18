import sys
import lcm

# put lcm types you need here
# run make once to generate python lcm types
from lcmtypes import pose_xyt_t

if len(sys.argv) < 2:
    sys.stderr.write("usage: decode-log <logfile>\n")
    sys.exit(1)

log = lcm.EventLog(sys.argv[1], "r")

f = open("TvsS.csv","w")

for event in log:
    if event.channel == "TRUE_POSE":
        msgT = pose_xyt_t.decode(event.data)
        # haveTrue = 1
        # print("TruePose:")
        # print("timestamp= %d" % msg.utime)
        # print("pose: (%f, %f, %f)" % (msg.x, msg.y, msg.theta))

    if event.channel == "SLAM_POSE":
        msgS = pose_xyt_t.decode(event.data)
        # haveSlam = 1
        # print("SlamPose:")
        # print("timestamp= %d" % msgS.utime)
        # print("pose: (%f, %f, %f)" % (msgS.x, msgS.y, msgS.theta))
        # print("%d , %f , %f , %f , %f , %f , %f" % (msgT.utime, msgT.x, msgT.y, msgT.theta, msgS.x, msgS.y, msgS.theta))
        f.write("%d , %f , %f , %f , %f , %f , %f \n" % (msgT.utime, msgT.x, msgT.y, msgT.theta, msgS.x, msgS.y, msgS.theta))


f.close()

# if msgT.utime == msgS.utime:

# if haveTrue == 1 and haveSlam == 1:
# 	print("%d , %f , %f , %f , %f , %f , %f" % (msgT.utime, msgT.x, msgT.y, msgT.theta, msgS.x, msgS.y, msgS.theta))