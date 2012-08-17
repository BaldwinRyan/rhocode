import subprocess

#var = raw_input("Connected to hotspot? (y/n)");
#if var == "y": 

return_string = subprocess.check_output("ifconfig", shell=True)
wlan_section = return_string[return_string.find('wlan0'):]
start = wlan_section.find("inet addr:")+10
end = wlan_section.find(" ",start)
address = wlan_section[start:end]
print "http://"+address+":11311/"
#    text = "export ROS_MASTER_URI=http://"+address+":11311/"
#    subprocess.call(text)
 #   subprocess.call("rostopic echo /android/imu")
 #   print text
#else:
 #   print "you said no"
