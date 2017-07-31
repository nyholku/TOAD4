file = open("serialno.txt","r")
serno=int(file.readline())
file.close()

serno = serno+1

file = open("serialno.txt","w")
print >>file, serno
file.close

t="107FC0001003"
for c in '{:07d}'.format(serno):
	t+='{:02X}'.format(ord(c))
	t+='00'

s=0
for i in xrange(0, len(t), 2):
	b = t[i:i+2]
	s += int(b,16)
s = (-s)&0xFF

t = ":"+t+'{:02X}'.format(s)
print t

