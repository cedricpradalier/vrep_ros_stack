#!/usr/bin/env python
from sys import exit
from subprocess import call
from subprocess import Popen
from subprocess import PIPE
from os import chdir
from urllib import urlopen
from os.path import isfile

if __name__ == "__main__":
	dot = Popen("rospack find ar_recog".split(' '), stdout=PIPE).communicate()[0]
	dot = dot[0:-1]

	if not isfile(dot + '/src/ARToolKit-2.72.1.tgz'):
		print "Downloading ARToolkit..."

		chdir(dot + "/src");

		try:
			stream = urlopen('http://sourceforge.net/projects/artoolkit/files/artoolkit/2.72.1/ARToolKit-2.72.1.tgz/download')
			archive = open('ARToolKit-2.72.1.tgz','w')
			archive.write(stream.read())
			stream.close()
			archive.close()
		except IOError:
			print "Problem retrieving ARToolkit!"
			exit(-1)

	print "Decompressing..."

	chdir(dot + "/src");
	call("tar xzf ARToolKit-2.72.1.tgz".split(' '))

	print "Patching..."

	chdir(dot + "/src/ARToolKit")

	try:
		patch = open('staticConfig.patch','w')
		patch.write("""40c40
<     read ANS
---
>     ANS="5"
124c124
<     read ANS
---
>     ANS="n"
182c182
< read ANS
---
> ANS="y"
""")
		patch.close()

		call("patch Configure staticConfig.patch".split(' '))
	except:
		print "Problem patching Configure!"
		exit(-1)

	print "Building ARToolkit..."

	call("./Configure")
	call("make")

	print "ARToolkit retrieved and Built. Copying object.c"

	call(('mkdir -p ' + dot + '/build/src/ARToolKit/examples/loadMultiple').split(' '))
	call(('cp ' + dot + '/src/ARToolKit/examples/loadMultiple/object.c ' + dot + '/build/src/ARToolKit/examples/loadMultiple/.').split(' '))
