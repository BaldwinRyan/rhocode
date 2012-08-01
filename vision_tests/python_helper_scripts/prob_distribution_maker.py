
import matplotlib.pyplot as plt

#hue, 6:38
#sat, 39:71
#int 72:
freq = []
for i in range(256):
	freq.append(0)
def get_hist_info(doc, lo, hi):
	filename = open(doc, 'r')
	lines = filename.readlines()
	filename.close()
	count = 0
	newList = []
	for line in lines[lo:hi]:
		num = ''
		for char in line:
			if char == ' ':
				i = 0
				while i < (int(num)):
					newList.append(count)
					i += 1
				freq[count] += int(num)
				num = ''
				count += 1
			else:
				num += char 
	return newList
hist_info = []
hist_info2 = []
for name in ['test_wo_2.txt', 'test_wo_3.txt', 'test_wo_4.txt', 'test_wo_5.txt','test_wo_6.txt']:
	hue = get_hist_info(name, 73, 105) 
	hist_info += hue
hist_info += get_hist_info('test_wo.txt', 72, 104)
for name in ['test_w_2.txt', 'test_w_3.txt', 'test_w_4.txt', 'test_w_5.txt','test_w_6.txt']:
	hue = get_hist_info(name, 7, 39) 
	hist_info2 += hue
hist_info2 += get_hist_info('test_w.txt', 6, 38)
#for i in range(256):
#	freq[i] = freq[i]/float(len(hist_info))
#endfile = open("Cone_Int_test.txt", 'w')
#endfile.write(str(freq))
#endfile.close() 
#print sum(freq)

plt.hist(hist_info, bins=256, normed=True, rwidth= 1)
#plt.hist(hist_info2, bins=256, normed=True, rwidth= 1, label = "Cone", alpha=.5, color= "red")
plt.legend()
#plt.hist(hueWO, bins=256, normed=True, alpha=.5)


plt.title("Background Int")
plt.xlabel("Value")
plt.ylabel("Probability")
plt.savefig("Background_Int.png")
plt.show()

