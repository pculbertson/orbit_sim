#!/usr/bin/env python
import sys
import os
import xml.etree.ElementTree as ET

filepath = os.path.dirname(os.path.realpath(sys.argv[0]))
launchTree = ET.parse('%s/../launch/create_world.launch' %filepath)
if len(sys.argv) > 1:
	if len(sys.argv) > 4:
		print('improper num of arguments')
		sys.exit()
	sdfTree = ET.parse('%s/../urdf/%s'%(filepath,sys.argv[1]))
else:
	sdfTree = ET.parse('%s/../urdf/payload.sdf'%filepath)


model=sdfTree.find('model')
launch = launchTree.getroot()

cubeSatSize = .1
linkNum = 0

config_file = open("%s/../config/config.txt"%filepath,"r")
for line in config_file:
	currName = line.split(":")[0]
	poseVal = line.split(":")[1].split("\n")[0]

	####set up model file###
	link = ET.SubElement(model,'link')
	link.set('name',currName)
	pose = ET.SubElement(link,'pose')
	pose.text = poseVal
	
	visual = ET.SubElement(link,'visual')
	visual.set('name','%s-vis'%currName)
	geom = ET.SubElement(visual,'geometry')
	box = ET.SubElement(geom,'box')
	size = ET.SubElement(box,'size')
	size.text = '%f %f %f'%(cubeSatSize,cubeSatSize,cubeSatSize)

	collision = ET.SubElement(link,'collision')
	collision.set('name','%s-col'%currName)
	geom = ET.SubElement(collision,'geometry')
	box = ET.SubElement(geom,'box')
	size = ET.SubElement(box,'size')
	size.text = '%f %f %f'%(cubeSatSize,cubeSatSize,cubeSatSize)

	gravity = ET.SubElement(link,'gravity')
	gravity.text = '0'

	joint = ET.SubElement(model,'joint')
	joint.set('name','%s-joint'%currName)
	joint.set('type','fixed')
	parent = ET.SubElement(joint,'parent')
	parent.text = 'body'
	child = ET.SubElement(joint,'child')
	child.text = currName

	node = ET.SubElement(launch,'node')
	node.set('name','%sCtrl'%currName)
	node.set('pkg','orbit_sim')
	node.set('type','6dofcontroller')
	node.set('output','screen')
	node.set('args','%d /wrenchPlugin/wrenchCommands:=/wrenchPlugin/wrenchCommands%d controllerState:=controllerState%d modelState:=modelState%d'%(linkNum,linkNum,linkNum,linkNum))

	linkNum += 1

##set up launch file##
if len(sys.argv) > 1:
	worldArg = launch.findall("./include/arg[@name='world_name']")
	worldArg[0].set('value','worlds/%s'%(sys.argv[2]))
	print worldArg[0].get('value')


sdfTree.write('%s/../models/fullModel.sdf'%filepath)
launchTree.write('%s/../launch/orbit_sim_6dof.launch'%filepath)
	
