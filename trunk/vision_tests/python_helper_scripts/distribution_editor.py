#probnexttoit/(1-probold+probnextnewly) = probnextnew
#probnexttoit= probnextnew(1-probold) + probnextnew^2
filename = open("Cone_Sat.txt", 'r')
line = filename.readline()
filename.close()
numList = []
num = ''
for char in line:
	if char in [',',' ', '[', ']']:
		if num != '':
			numList.append(num)
		num = ''
	else:
		num += char
#root = (-(1.0-float(numList[30]))+((1.0-float(numList[30]))**2.0+4.0*1.0*float(numList[29]))**.5)/(2.0)
#for i in range(len(numList)):
#	numList[i] = float(numList[i])/(1.0-float(numList[30])+0.058339203559027775)
#numList[30] = 0.058339203559027775
for i in range(len(numList)):
	numList[i] = float(numList[i])/(1.0-float(numList[0]))*(1.0-0.05300935329861111)
numList[0] = 0.05300935329861111
count = 0
for elt in numList:
	count += elt
print count
newfile = open("Editted_Cone_Hue.txt", 'w')
newfile.write(str(numList))
newfile.close()

'''currently the 1-255 represent 1-0thvalue probability, we want it to represent 1-newprob probability'''
